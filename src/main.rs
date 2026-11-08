#![no_std]
#![no_main]
#![deny(clippy::mem_forget)]
#![deny(clippy::large_stack_frames)]

mod control;
mod http;
mod net;
mod servo;
mod store;

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::ram;
use esp_hal::rng::Rng;
use esp_hal::timer::timg::TimerGroup;
use random_laser_pointer::controller::Controller;
use random_laser_pointer::settings::Settings;

use random_laser_pointer::api::State;

esp_bootloader_esp_idf::esp_app_desc!();

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write($val);
        x
    }};
}
pub(crate) use mk_static;

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 36 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

    let laser = Output::new(peripherals.GPIO5, Level::Low, OutputConfig::default());
    let servos = servo::setup(peripherals.LEDC, peripherals.GPIO3, peripherals.GPIO2);
    let travel = servos.travel;
    let mut store = store::SettingsStore::new(peripherals.FLASH);
    if store.is_none() {
        log::warn!("Settings will not persist");
    }
    let settings = store
        .as_mut()
        .and_then(|s| s.load(travel))
        .unwrap_or_else(|| Settings::defaults(travel));

    let rng = Rng::new();
    let seed = || (rng.random() as u64) << 32 | rng.random() as u64;
    let controller = Controller::new(control::now_us(), travel, &settings, seed());
    control::STATE.lock(|s| {
        s.replace(Some(State {
            controller,
            settings,
            travel,
        }))
    });
    spawner.spawn(control::controller_task(servos, laser).unwrap());
    if let Some(store) = store {
        spawner.spawn(store::store_task(store).unwrap());
    }
    log::info!("Random mode");

    let stack = net::start(spawner, peripherals.WIFI, seed()).await;
    for _ in 0..http::HTTP_POOL_SIZE {
        spawner.spawn(http::http_task(stack).unwrap());
    }

    loop {
        Timer::after(Duration::from_secs(3600)).await;
    }
}
