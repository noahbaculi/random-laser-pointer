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

    // initialize peripheral
    let clock_cfg = PeripheralClockConfig::with_frequency(&clocks, 40.MHz()).unwrap();
    let mut mcpwm = McPwm::new(peripherals.MCPWM0, clock_cfg);

    // connect operator0 to timer0
    mcpwm.operator0.set_timer(&mcpwm.timer0);
    // connect operator0 to pin
    let mut pwm_pin = mcpwm
        .operator0
        .with_pin_a(pin, PwmPinConfig::UP_ACTIVE_HIGH);

    // let max_duty = pwm_pin.get_max_duty();
    // log::info!("Max Duty: {}", max_duty);
    let period = pwm_pin.get_period();
    log::info!("Period: {}", period);
    let timestamp = pwm_pin.get_timestamp();
    log::info!("Timestamp: {}", timestamp);

    let timer_clock_cfg = clock_cfg
        .timer_clock_with_frequency(99, PwmWorkingMode::Increase, 20.Hz())
        .unwrap();
    mcpwm.timer0.start(timer_clock_cfg);

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
