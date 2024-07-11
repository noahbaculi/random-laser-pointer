#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::ledc::{channel, timer, LSGlobalClkSource, Ledc, LowSpeed};
use esp_hal::mcpwm::timer::TimerClockConfig;
use esp_hal::mcpwm::{operator::PwmPinConfig, timer::PwmWorkingMode, McPwm, PeripheralClockConfig};
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::{Io, Level, Output},
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
};

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
    let mut servo_2_pwm_channel = ledc.get_channel(channel::Number::Channel0, pin_for_servo_2);
    servo_2_pwm_channel
        .configure(channel::config::Config {
            timer: &lstimer0,
            duty_pct: 0,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();

    loop {
        servo_1_pwm_channel.set_duty(20).unwrap();
        led.set_low();
        log::info!("LED Off");
        delay.delay(2000.millis());

        log::info!("Servo 1 sweep start");
        servo_1_pwm_channel.start_duty_fade(2, 13, 50_000).unwrap();
        log::info!("Servo 2 sweep start");
        servo_2_pwm_channel.start_duty_fade(2, 13, 50_000).unwrap();

        for duty_num in 2..=13 {
            // servo_1_pwm_channel.set_duty(duty_num).unwrap();
            // servo_1_pwm_channel.set_duty(duty_num).unwrap();
            log::info!("Duty: {}", duty_num);
            delay.delay(2000.millis());
        }
    }
}
