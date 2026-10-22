//! Manual WiFi D-pad mode.
//!
//! Open access point with a DHCP server, serving a web D-pad.
//! Hold a direction to glide the laser. Release to stop.
//! Join `laser-dpad`, then open http://192.168.2.1/

mod http;
mod net;

use core::cell::Cell;
use core::net::Ipv4Addr;

use embassy_executor::Spawner;
use embassy_net::{Ipv4Cidr, StackResources, StaticConfigV4};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_hal::pwm::SetDutyCycle;
use esp_hal::peripherals::WIFI;
use esp_hal::rng::Rng;
use esp_radio::wifi::ap::AccessPointConfig;
use esp_radio::wifi::{Config, ControllerConfig, Interface, WifiController};

use crate::mk_static;
use crate::servo::Servos;

const AP_SSID: &str = "laser-dpad";
const GATEWAY_IP: Ipv4Addr = Ipv4Addr::new(192, 168, 2, 1);

// Full-range sweep takes (max_duty - min_duty) / RAMP_COUNTS_PER_SEC seconds
const RAMP_COUNTS_PER_SEC: f32 = 450.0;
const SERVO_TICK_MS: u64 = 20;
// Stop if no keepalive arrives, guards against a lost release packet.
// Sized to tolerate one lost 120 ms keepalive plus jitter
const DEADMAN_MS: u64 = 350;

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

/// Brings up the access point, network stack, and servo task, then parks.
/// Deep sleep never runs in this mode. Pots and the preview button are
/// ignored after boot.
pub async fn run(spawner: Spawner, wifi: WIFI<'static>, servos: Servos) -> ! {
    let ap_config =
        Config::AccessPoint(AccessPointConfig::default().with_ssid(AP_SSID.try_into().unwrap()));
    let device = Interface::access_point();
    let controller = WifiController::new(
        wifi,
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
        // 3 http TCP sockets + DHCP UDP + headroom
        mk_static!(StackResources<8>, StackResources::<8>::new()),
        seed,
    );

    spawner.spawn(net::connection(controller).unwrap());
    spawner.spawn(net::net_task(runner).unwrap());
    spawner.spawn(net::run_dhcp(stack).unwrap());
    spawner.spawn(servo_task(servos).unwrap());

    stack.wait_config_up().await;
    log::info!("Join open network `{AP_SSID}`, then open http://{GATEWAY_IP}/");

    for _ in 0..http::HTTP_POOL_SIZE {
        spawner.spawn(http::http_task(stack).unwrap());
    }

    loop {
        Timer::after(Duration::from_secs(3600)).await;
    }
}

#[embassy_executor::task]
async fn servo_task(servos: Servos) {
    let Servos {
        mut x,
        mut y,
        bounds,
    } = servos;
    let mut x_duty = bounds.center();
    let mut y_duty = bounds.center();

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
        x_duty = x_duty.clamp(bounds.min, bounds.max);
        y_duty = y_duty.clamp(bounds.min, bounds.max);

        if x_duty as u16 != prev_x {
            x.set_duty_cycle(x_duty as u16).unwrap();
        }
        if y_duty as u16 != prev_y {
            y.set_duty_cycle(y_duty as u16).unwrap();
        }
    }
}
