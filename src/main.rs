#![no_std]
#![no_main]

// THROWAWAY SPIKE: WiFi D-pad latency probe, modern stack edition.
// ESP32-C3 as an open WiFi access point with a DHCP server, serving a
// web D-pad. Hold a direction to glide the laser; release to stop.
// Probes felt press-to-motion latency for manual servo control.
//
// Join `laser-dpad`, then open http://192.168.2.1/

use core::cell::Cell;
use core::net::Ipv4Addr;

use embassy_executor::Spawner;
use embassy_net::tcp::TcpSocket;
use embassy_net::{IpListenEndpoint, Ipv4Cidr, Runner, Stack, StackResources, StaticConfigV4};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_hal::pwm::SetDutyCycle;
use embedded_io_async::Write;
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{DriveMode, Level, Output, OutputConfig};
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::ledc::channel::ChannelIFace;
use esp_hal::ledc::timer::TimerIFace;
use esp_hal::ledc::{channel, timer, LSGlobalClkSource, Ledc, LowSpeed};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{ram, rng::Rng};
use esp_radio::wifi::ap::AccessPointConfig;
use esp_radio::wifi::{Config, ControllerConfig, Interface, WifiController};

esp_bootloader_esp_idf::esp_app_desc!();

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

const AP_SSID: &str = "laser-dpad";
const GATEWAY_IP: Ipv4Addr = Ipv4Addr::new(192, 168, 2, 1);

const SERVO_MIN_DUTY_PERCENTAGE: f32 = 0.03;
const SERVO_MAX_DUTY_PERCENTAGE: f32 = 0.12;

// Full-range sweep takes (max_duty - min_duty) / RAMP_COUNTS_PER_SEC seconds
const RAMP_COUNTS_PER_SEC: f32 = 900.0;
const SERVO_TICK_MS: u64 = 20;
// Stop if no keepalive arrives, guards against a lost release packet
const DEADMAN_MS: u64 = 500;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Dir {
    Stop,
    Up,
    Down,
    Left,
    Right,
}

#[derive(Clone, Copy)]
struct Cmd {
    dir: Dir,
    at: Instant,
}

static CMD: Mutex<CriticalSectionRawMutex, Cell<Cmd>> = Mutex::new(Cell::new(Cmd {
    dir: Dir::Stop,
    at: Instant::from_ticks(0),
}));

fn set_command(dir: Dir) {
    CMD.lock(|c| {
        if c.get().dir != dir {
            log::info!("cmd {:?} at {} ms", dir, Instant::now().as_millis());
        }
        c.set(Cmd {
            dir,
            at: Instant::now(),
        });
    });
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 36 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let _laser_diode = Output::new(peripherals.GPIO5, Level::High, OutputConfig::default());
    log::info!("Laser Diode On");

    // Instantiate PWM infra
    let ledc = mk_static!(Ledc<'static>, Ledc::new(peripherals.LEDC));
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    let pwm_timer = mk_static!(
        timer::Timer<'static, LowSpeed>,
        ledc.timer::<LowSpeed>(timer::Number::Timer0)
    );
    pwm_timer
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty14Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_hz(50),
        })
        .unwrap();
    let pwm_timer: &'static timer::Timer<'static, LowSpeed> = pwm_timer;

    let mut servo_x_pwm_channel = ledc.channel::<LowSpeed>(channel::Number::Channel0, peripherals.GPIO3);
    servo_x_pwm_channel
        .configure(channel::config::Config {
            timer: pwm_timer,
            duty_pct: 0,
            drive_mode: DriveMode::PushPull,
        })
        .unwrap();

    let mut servo_y_pwm_channel = ledc.channel::<LowSpeed>(channel::Number::Channel1, peripherals.GPIO2);
    servo_y_pwm_channel
        .configure(channel::config::Config {
            timer: pwm_timer,
            duty_pct: 0,
            drive_mode: DriveMode::PushPull,
        })
        .unwrap();

    // Bring up WiFi access point
    let ap_config = Config::AccessPoint(AccessPointConfig::default().with_ssid(AP_SSID));
    let device = Interface::access_point();
    let controller = WifiController::new(
        peripherals.WIFI,
        ControllerConfig::default().with_initial_config(ap_config),
    )
    .unwrap();
    log::info!("WiFi AP starting");

    let net_config = embassy_net::Config::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(GATEWAY_IP, 24),
        gateway: Some(GATEWAY_IP),
        dns_servers: Default::default(),
    });

    let rng = Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    let (stack, runner) = embassy_net::new(
        device,
        net_config,
        mk_static!(StackResources<4>, StackResources::<4>::new()),
        seed,
    );

    spawner.spawn(connection(controller).unwrap());
    spawner.spawn(net_task(runner).unwrap());
    spawner.spawn(run_dhcp(stack).unwrap());
    spawner.spawn(servo_task(servo_x_pwm_channel, servo_y_pwm_channel).unwrap());

    stack.wait_config_up().await;
    log::info!("Join open network `{AP_SSID}`, then open http://{GATEWAY_IP}/");

    let mut rx_buffer = [0; 1536];
    let mut tx_buffer = [0; 4096];
    let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
    socket.set_timeout(Some(Duration::from_secs(10)));

    loop {
        let r = socket
            .accept(IpListenEndpoint {
                addr: None,
                port: 80,
            })
            .await;
        if let Err(e) = r {
            log::warn!("accept error: {:?}", e);
            socket.abort();
            continue;
        }

        let mut buffer = [0u8; 1024];
        let mut pos = 0;
        let mut complete = false;
        loop {
            match socket.read(&mut buffer[pos..]).await {
                Ok(0) => break,
                Ok(len) => {
                    pos += len;
                    if buffer[..pos].windows(4).any(|w| w == b"\r\n\r\n") {
                        complete = true;
                        break;
                    }
                    if pos == buffer.len() {
                        break;
                    }
                }
                Err(e) => {
                    log::warn!("read error: {:?}", e);
                    break;
                }
            }
        }

        if complete {
            let request = unsafe { core::str::from_utf8_unchecked(&buffer[..pos]) };
            let response: &[u8] = match parse_path(request) {
                Some("/") => PAGE.as_bytes(),
                Some(path) if path.starts_with("/d?dir=") => {
                    match &path[7..] {
                        "up" => set_command(Dir::Up),
                        "down" => set_command(Dir::Down),
                        "left" => set_command(Dir::Left),
                        "right" => set_command(Dir::Right),
                        "stop" => set_command(Dir::Stop),
                        other => log::warn!("Unknown dir: {other}"),
                    }
                    b"HTTP/1.0 204 No Content\r\nConnection: close\r\n\r\n"
                }
                _ => b"HTTP/1.0 404 Not Found\r\nConnection: close\r\n\r\n",
            };
            if let Err(e) = socket.write_all(response).await {
                log::warn!("write error: {:?}", e);
            }
            if let Err(e) = socket.flush().await {
                log::warn!("flush error: {:?}", e);
            }
        }

        socket.close();
        _ = socket.flush().await;
        socket.abort();
    }
}

fn parse_path(request: &str) -> Option<&str> {
    let first_line = request.lines().next()?;
    let mut parts = first_line.split(' ');
    match parts.next()? {
        "GET" => parts.next(),
        _ => None,
    }
}

#[embassy_executor::task]
async fn servo_task(
    mut x_channel: channel::Channel<'static, LowSpeed>,
    mut y_channel: channel::Channel<'static, LowSpeed>,
) {
    let max_duty = x_channel.max_duty_cycle() as f32;
    let min = SERVO_MIN_DUTY_PERCENTAGE * max_duty;
    let max = SERVO_MAX_DUTY_PERCENTAGE * max_duty;
    let center = min + ((max - min) / 2.0);
    let mut x_duty = center;
    let mut y_duty = center;
    x_channel.set_duty_cycle(x_duty as u16).unwrap();
    y_channel.set_duty_cycle(y_duty as u16).unwrap();
    log::info!("Servo task: duty range {min} .. {max}");

    let mut ticker = Ticker::every(Duration::from_millis(SERVO_TICK_MS));
    loop {
        ticker.next().await;
        let cmd = CMD.lock(|c| c.get());
        let dir = if Instant::now() - cmd.at > Duration::from_millis(DEADMAN_MS) {
            Dir::Stop
        } else {
            cmd.dir
        };

        let delta = RAMP_COUNTS_PER_SEC * SERVO_TICK_MS as f32 / 1000.0;
        let (prev_x, prev_y) = (x_duty as u16, y_duty as u16);
        // Inverted to match the servo mounting orientation
        match dir {
            Dir::Stop => continue,
            Dir::Up => y_duty -= delta,
            Dir::Down => y_duty += delta,
            Dir::Right => x_duty -= delta,
            Dir::Left => x_duty += delta,
        }
        x_duty = x_duty.clamp(min, max);
        y_duty = y_duty.clamp(min, max);

        if x_duty as u16 != prev_x {
            x_channel.set_duty_cycle(x_duty as u16).unwrap();
        }
        if y_duty as u16 != prev_y {
            y_channel.set_duty_cycle(y_duty as u16).unwrap();
        }
    }
}

#[embassy_executor::task]
async fn run_dhcp(stack: Stack<'static>) {
    use core::net::SocketAddrV4;

    use edge_dhcp::io::{self, DEFAULT_SERVER_PORT};
    use edge_dhcp::server::{Server, ServerOptions};
    use edge_nal::UdpBind;
    use edge_nal_embassy::{Udp, UdpBuffers};

    let mut buf = [0u8; 1500];
    let mut gw_buf = [Ipv4Addr::UNSPECIFIED];

    let buffers = UdpBuffers::<3, 1024, 1024, 10>::new();
    let unbound_socket = Udp::new(stack, &buffers);
    let mut bound_socket = unbound_socket
        .bind(core::net::SocketAddr::V4(SocketAddrV4::new(
            Ipv4Addr::UNSPECIFIED,
            DEFAULT_SERVER_PORT,
        )))
        .await
        .unwrap();

    loop {
        _ = io::server::run(
            &mut Server::<_, 64>::new_with_et(GATEWAY_IP),
            &ServerOptions::new(GATEWAY_IP, Some(&mut gw_buf)),
            &mut bound_socket,
            &mut buf,
        )
        .await
        .inspect_err(|e| log::warn!("DHCP server error: {e:?}"));
        Timer::after(Duration::from_millis(500)).await;
    }
}

#[embassy_executor::task]
async fn connection(controller: WifiController<'static>) {
    loop {
        let ev = controller
            .wait_for_access_point_connected_event_async()
            .await;
        match ev {
            Ok(esp_radio::wifi::ap::EventInfo::Connected(info)) => {
                log::info!("Station connected: {:?}", info);
            }
            Ok(esp_radio::wifi::ap::EventInfo::Disconnected(info)) => {
                log::info!("Station disconnected: {:?}", info);
            }
            _ => (),
        }
        Timer::after(Duration::from_millis(1000)).await
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, Interface>) {
    runner.run().await
}

const PAGE: &str = concat!(
    "HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n",
    r##"<!DOCTYPE html><html><head>
<meta name=viewport content="width=device-width,initial-scale=1,user-scalable=no">
<title>Laser D-pad</title><style>
body{margin:0;font-family:system-ui;background:#111;color:#eee;display:flex;flex-direction:column;align-items:center;justify-content:center;min-height:100vh;touch-action:none;-webkit-user-select:none;user-select:none}
#pad{display:grid;grid-template-columns:repeat(3,88px);grid-template-rows:repeat(3,88px);gap:8px}
button{font-size:34px;border:0;border-radius:14px;background:#333;color:#eee;touch-action:none}
button:active{background:#0a84ff}
#rtt{display:flex;align-items:center;justify-content:center;font-size:14px;color:#888;white-space:pre;text-align:center}
#st{color:#888}
</style></head><body>
<h3>Laser D-pad</h3>
<div id=pad>
<i></i><button data-d=up>&#9650;</button><i></i>
<button data-d=left>&#9664;</button><div id=rtt>--</div><button data-d=right>&#9654;</button>
<i></i><button data-d=down>&#9660;</button><i></i>
</div>
<p id=st>released</p>
<script>
let held=null,timer=null,busy=false,pend=null,rtts=[];
function show(r){rtts.push(r);if(rtts.length>30)rtts.shift();
let a=rtts.reduce((x,y)=>x+y,0)/rtts.length;
document.getElementById('rtt').textContent=r.toFixed(0)+' ms\navg '+a.toFixed(0);}
function send(d){if(busy){pend=d;return}busy=true;let t=performance.now();
fetch('/d?dir='+d).then(()=>show(performance.now()-t)).catch(()=>{}).finally(()=>{busy=false;if(pend){let p=pend;pend=null;send(p)}});}
function dn(e,d){e.preventDefault();held=d;document.getElementById('st').textContent=d;send(d);
if(timer)clearInterval(timer);timer=setInterval(()=>{if(held)send(held)},150);}
function up(e){e.preventDefault();if(!held)return;held=null;if(timer)clearInterval(timer);timer=null;
document.getElementById('st').textContent='released';pend=null;send('stop');}
for(const b of document.querySelectorAll('button')){
b.addEventListener('pointerdown',e=>dn(e,b.dataset.d));
b.addEventListener('pointerup',up);b.addEventListener('pointercancel',up);b.addEventListener('pointerleave',up);}
</script></body></html>"##
);
