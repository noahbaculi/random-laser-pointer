#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::ledc::{channel, timer, LSGlobalClkSource, Ledc, LowSpeed};
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::{Io, Level, Output},
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
};

const SERVO_MIN_DUTY: u8 = 2;
const SERVO_MAX_DUTY: u8 = 13;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::max(system.clock_control).freeze();
    let delay = Delay::new(&clocks);

    esp_println::logger::init_logger_from_env();

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut led = Output::new(io.pins.gpio13, Level::High);

    led.set_high();
    log::info!("LED On");
    delay.delay(500.millis());

    let pin_for_servo_1 = io.pins.gpio4;
    let pin_for_servo_2 = io.pins.gpio5;

    let mut ledc = Ledc::new(peripherals.LEDC, &clocks);

    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let mut lstimer0 = ledc.get_timer::<LowSpeed>(timer::Number::Timer0);

    lstimer0
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty14Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: 50.Hz(),
        })
        .unwrap();

    let mut servo_1_pwm_channel = ledc.get_channel(channel::Number::Channel0, pin_for_servo_1);
    servo_1_pwm_channel
        .configure(channel::config::Config {
            timer: &lstimer0,
            duty_pct: 0,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();
    let mut servo_2_pwm_channel = ledc.get_channel(channel::Number::Channel1, pin_for_servo_2);
    servo_2_pwm_channel
        .configure(channel::config::Config {
            timer: &lstimer0,
            duty_pct: 0,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();

    led.set_low();
    log::info!("LED Off");
    loop {
        log::info!("Looping...");

        for duty_num in SERVO_MIN_DUTY..=SERVO_MAX_DUTY {
            log::info!("Duty: {}", duty_num);
            servo_1_pwm_channel.set_duty(duty_num).unwrap();
            servo_2_pwm_channel.set_duty(15 - duty_num).unwrap();
            // servo_2_pwm_channel.set_duty(duty_num).unwrap();
            delay.delay(2000.millis());
        }
    }
}
