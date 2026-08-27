#![no_std]
#![no_main]

// THROWAWAY SPIKE: WiFi D-pad latency probe.
// Brings the ESP32-C3 up as a WiFi access point serving a web D-pad.
// Hold a direction to glide the laser; release to stop. Measures felt
// press-to-motion latency for manual servo control over WiFi.
//
// No DHCP server: configure the phone with static IP 192.168.2.2,
// gateway 192.168.2.1, subnet /24, then browse to http://192.168.2.1/

use embedded_hal::pwm::SetDutyCycle;
use embedded_io::{Read, Write};
use esp_backtrace as _;
use esp_hal::gpio::OutputPin;
use esp_hal::ledc::timer::TimerSpeed;
use esp_hal::ledc::{self, channel, timer, LSGlobalClkSource, Ledc, LowSpeed};
use esp_hal::rng::Rng;
use esp_hal::timer::PeriodicTimer;
use esp_hal::{
    clock::ClockControl,
    gpio::{Io, Level, Output},
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
};
use esp_wifi::wifi::utils::create_network_interface;
use esp_wifi::wifi::{AccessPointConfiguration, AuthMethod, Configuration, WifiApDevice};
use esp_wifi::wifi_interface::WifiStack;
use esp_wifi::{current_millis, initialize, EspWifiInitFor};
use smoltcp::iface::SocketStorage;

const AP_SSID: &str = "laser-dpad";
const AP_PASSWORD: &str = "laserlaser";

const SERVO_MIN_DUTY_PERCENTAGE: f32 = 0.03;
const SERVO_MAX_DUTY_PERCENTAGE: f32 = 0.12;

// Full-range sweep takes (max_duty - min_duty) / RAMP_COUNTS_PER_SEC seconds
const RAMP_COUNTS_PER_SEC: f32 = 900.0;
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

struct DpadState {
    dir: Dir,
    last_cmd_ms: u64,
    last_tick_ms: u64,
    x_duty: f32,
    y_duty: f32,
    min_duty: f32,
    max_duty: f32,
}

impl DpadState {
    fn command(&mut self, dir: Dir) {
        let now = current_millis();
        if dir != self.dir {
            log::info!("cmd {:?} at {} ms", dir, now);
        }
        self.dir = dir;
        self.last_cmd_ms = now;
    }
}

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::max(system.clock_control).freeze();
    esp_println::logger::init_logger_from_env();

    let _laser_diode = Output::new(io.pins.gpio5, Level::High);
    log::info!("Laser Diode On");

    let pin_for_servo_x = io.pins.gpio3;
    let pin_for_servo_y = io.pins.gpio2;

    // Instantiate PWM infra
    let mut ledc_pwm_controller = Ledc::new(peripherals.LEDC, &clocks);
    ledc_pwm_controller.set_global_slow_clock(LSGlobalClkSource::APBClk);
    let mut pwm_timer = ledc_pwm_controller.get_timer::<LowSpeed>(timer::Number::Timer0);
    pwm_timer
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty14Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: 50.Hz(),
        })
        .unwrap();

    let mut servo_x_pwm_channel =
        ledc_pwm_controller.get_channel(channel::Number::Channel0, pin_for_servo_x);
    servo_x_pwm_channel
        .configure(channel::config::Config {
            timer: &pwm_timer,
            duty_pct: 0,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();

    let mut servo_y_pwm_channel =
        ledc_pwm_controller.get_channel(channel::Number::Channel1, pin_for_servo_y);
    servo_y_pwm_channel
        .configure(channel::config::Config {
            timer: &pwm_timer,
            duty_pct: 0,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();

    let pwm_channel_max_duty = servo_x_pwm_channel.max_duty_cycle();
    let min_duty = SERVO_MIN_DUTY_PERCENTAGE * pwm_channel_max_duty as f32;
    let max_duty = SERVO_MAX_DUTY_PERCENTAGE * pwm_channel_max_duty as f32;
    let center_duty = min_duty + ((max_duty - min_duty) / 2.0);

    servo_x_pwm_channel
        .set_duty_cycle(center_duty as u16)
        .unwrap();
    servo_y_pwm_channel
        .set_duty_cycle(center_duty as u16)
        .unwrap();

    let mut state = DpadState {
        dir: Dir::Stop,
        last_cmd_ms: 0,
        last_tick_ms: 0,
        x_duty: center_duty,
        y_duty: center_duty,
        min_duty,
        max_duty,
    };

    // Bring up WiFi access point
    let wifi_timer = PeriodicTimer::new(
        esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0, &clocks, None)
            .timer0
            .into(),
    );
    let init = initialize(
        EspWifiInitFor::Wifi,
        wifi_timer,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
        &clocks,
    )
    .unwrap();

    let wifi = peripherals.WIFI;
    let mut socket_set_entries: [SocketStorage; 3] = Default::default();
    let (iface, device, mut controller, sockets) =
        create_network_interface(&init, wifi, WifiApDevice, &mut socket_set_entries).unwrap();
    let mut wifi_stack = WifiStack::new(iface, device, sockets, current_millis);

    let ap_config = Configuration::AccessPoint(AccessPointConfiguration {
        ssid: AP_SSID.try_into().unwrap(),
        auth_method: AuthMethod::WPA2Personal,
        password: AP_PASSWORD.try_into().unwrap(),
        ..Default::default()
    });
    controller.set_configuration(&ap_config).unwrap();
    controller.start().unwrap();
    log::info!("WiFi AP started: {:?}", controller.is_started());

    wifi_stack
        .set_iface_configuration(&esp_wifi::wifi::ipv4::Configuration::Client(
            esp_wifi::wifi::ipv4::ClientConfiguration::Fixed(
                esp_wifi::wifi::ipv4::ClientSettings {
                    ip: esp_wifi::wifi::ipv4::Ipv4Addr::new(192, 168, 2, 1),
                    subnet: esp_wifi::wifi::ipv4::Subnet {
                        gateway: esp_wifi::wifi::ipv4::Ipv4Addr::new(192, 168, 2, 1),
                        mask: esp_wifi::wifi::ipv4::Mask(24),
                    },
                    dns: None,
                    secondary_dns: None,
                },
            ),
        ))
        .unwrap();

    log::info!("Join `{AP_SSID}` (password `{AP_PASSWORD}`) with static IP 192.168.2.2, gateway 192.168.2.1, then open http://192.168.2.1/");

    let mut rx_buffer = [0u8; 1536];
    let mut tx_buffer = [0u8; 4096];
    let mut socket = wifi_stack.get_socket(&mut rx_buffer, &mut tx_buffer);
    socket.listen(80).unwrap();

    loop {
        socket.work();
        tick_servos(&mut state, &mut servo_x_pwm_channel, &mut servo_y_pwm_channel);

        if !socket.is_open() {
            socket.listen(80).unwrap();
        }

        if !socket.is_connected() {
            continue;
        }

        // Read the request, ticking servos while we wait
        let deadline = current_millis() + 1_000;
        let mut buffer = [0u8; 1024];
        let mut pos = 0;
        let mut complete = false;
        while current_millis() < deadline {
            socket.work();
            tick_servos(&mut state, &mut servo_x_pwm_channel, &mut servo_y_pwm_channel);
            if pos < buffer.len() {
                if let Ok(len) = socket.read(&mut buffer[pos..]) {
                    pos += len;
                }
            }
            if buffer[..pos].windows(4).any(|w| w == b"\r\n\r\n") {
                complete = true;
                break;
            }
        }

        if complete {
            let request = unsafe { core::str::from_utf8_unchecked(&buffer[..pos]) };
            let response: &[u8] = match parse_path(request) {
                Some("/") => PAGE.as_bytes(),
                Some(path) if path.starts_with("/d?dir=") => {
                    match &path[7..] {
                        "up" => state.command(Dir::Up),
                        "down" => state.command(Dir::Down),
                        "left" => state.command(Dir::Left),
                        "right" => state.command(Dir::Right),
                        "stop" => state.command(Dir::Stop),
                        other => log::warn!("Unknown dir: {other}"),
                    }
                    b"HTTP/1.0 204 No Content\r\nConnection: close\r\n\r\n"
                }
                _ => b"HTTP/1.0 404 Not Found\r\nConnection: close\r\n\r\n",
            };
            socket.write_all(response).unwrap();
            socket.flush().unwrap();
        }

        socket.close();
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

fn tick_servos<S, XO, YO>(
    state: &mut DpadState,
    x_channel: &mut ledc::channel::Channel<S, XO>,
    y_channel: &mut ledc::channel::Channel<S, YO>,
) where
    S: TimerSpeed,
    XO: OutputPin,
    YO: OutputPin,
{
    let now = current_millis();
    let dt_ms = now.saturating_sub(state.last_tick_ms);
    if dt_ms == 0 {
        return;
    }
    state.last_tick_ms = now;

    if state.dir != Dir::Stop && now.saturating_sub(state.last_cmd_ms) > DEADMAN_MS {
        log::warn!("Deadman timeout, stopping");
        state.dir = Dir::Stop;
    }

    let delta = RAMP_COUNTS_PER_SEC * dt_ms as f32 / 1000.0;
    let (prev_x, prev_y) = (state.x_duty as u16, state.y_duty as u16);
    match state.dir {
        Dir::Stop => return,
        Dir::Up => state.y_duty += delta,
        Dir::Down => state.y_duty -= delta,
        Dir::Right => state.x_duty += delta,
        Dir::Left => state.x_duty -= delta,
    }
    state.x_duty = state.x_duty.clamp(state.min_duty, state.max_duty);
    state.y_duty = state.y_duty.clamp(state.min_duty, state.max_duty);

    if state.x_duty as u16 != prev_x {
        x_channel.set_duty_cycle(state.x_duty as u16).unwrap();
    }
    if state.y_duty as u16 != prev_y {
        y_channel.set_duty_cycle(state.y_duty as u16).unwrap();
    }
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
