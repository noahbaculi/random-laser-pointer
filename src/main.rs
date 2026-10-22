#![no_std]
#![no_main]
#![deny(clippy::mem_forget)]
#![deny(clippy::large_stack_frames)]

mod manual;
mod random;
mod servo;

use embassy_executor::Spawner;
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::ram;
use esp_hal::timer::timg::TimerGroup;

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

    let _laser_diode = Output::new(peripherals.GPIO5, Level::High, OutputConfig::default());
    log::info!("Laser Diode On");

    let servos = servo::setup(peripherals.LEDC, peripherals.GPIO3, peripherals.GPIO2);

    let preview_btn = Input::new(
        peripherals.GPIO6,
        InputConfig::default().with_pull(Pull::Up),
    );

    // Boot-time mode strap: latched (low) selects manual WiFi mode.
    // Unlatched keeps the button's preview role. Mode exit is a power cycle
    if preview_btn.is_low() {
        manual::run(spawner, peripherals.WIFI, servos).await
    } else {
        random::run(
            servos,
            peripherals.ADC1,
            peripherals.GPIO0,
            peripherals.GPIO1,
            peripherals.LPWR,
            preview_btn,
        )
        .await
    }
}
