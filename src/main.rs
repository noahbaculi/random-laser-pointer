#![no_std]
#![no_main]

use core::time::Duration;

use esp_backtrace as _;
use esp_hal::ledc::{channel, timer, LSGlobalClkSource, Ledc, LowSpeed};
use esp_hal::rtc_cntl::sleep::TimerWakeupSource;
use esp_hal::rtc_cntl::Rtc;
use esp_hal::time;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::{Io, Level, Output},
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
};
use rand::rngs::SmallRng;
use rand::{Rng, SeedableRng};

const ON_DURATION_MIN: u8 = 1; // Duration laser should be active every cycle
const SLEEP_DURATION_MIN: u8 = 2; // Duration laser should be inactive every cycle
const SERVO_MIN_SLEEP_MS: u32 = 900;
const SERVO_MAX_SLEEP_MS: u32 = 5 * 1_000;

const SERVO_MIN_DUTY: u8 = 3;
const SERVO_MAX_DUTY: u8 = 12;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::max(system.clock_control).freeze();
    let mut delay = Delay::new(&clocks);
    let mut rtc = Rtc::new(peripherals.LPWR, None);
    esp_println::logger::init_logger_from_env();

    // Instantiate LED
    let _power_led = Output::new(io.pins.gpio13, Level::High);
    log::info!("Power LED On");

    // Instantiate PWM pins
    let pin_for_servo_1 = io.pins.gpio4;
    let pin_for_servo_2 = io.pins.gpio5;

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

    // Instantiate PWM channels
    let mut servo_1_pwm_channel =
        ledc_pwm_controller.get_channel(channel::Number::Channel0, pin_for_servo_1);
    servo_1_pwm_channel
        .configure(channel::config::Config {
            timer: &pwm_timer,
            duty_pct: 0,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();
    let mut servo_2_pwm_channel =
        ledc_pwm_controller.get_channel(channel::Number::Channel1, pin_for_servo_2);
    servo_2_pwm_channel
        .configure(channel::config::Config {
            timer: &pwm_timer,
            duty_pct: 0,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();

    let mut small_rng = SmallRng::seed_from_u64(1); // seed doesn't matter
    let sleep_timer = TimerWakeupSource::new(Duration::from_secs(SLEEP_DURATION_MIN as u64 * 60));

    log::info!("Starting loop...");
    loop {
        let loop_instant = time::current_time();
        let uptime_min = loop_instant.duration_since_epoch().to_minutes();
        log::info!("Uptime: {} min", uptime_min);
        if uptime_min >= ON_DURATION_MIN.into() {
            log::info!("Entering deep sleep for {} min...", SLEEP_DURATION_MIN);
            delay.delay_millis(100);
            rtc.sleep_deep(&[&sleep_timer], &mut delay);
        }

        let servo_1_duty_percent = small_rng.gen_range(SERVO_MIN_DUTY..=SERVO_MAX_DUTY);
        log::info!("Servo 1 duty percent: {}", servo_1_duty_percent);
        servo_1_pwm_channel.set_duty(servo_1_duty_percent).unwrap();

        let servo_2_duty_percent = small_rng.gen_range(SERVO_MIN_DUTY..=SERVO_MAX_DUTY);
        log::info!("Servo 2 duty percent: {}", servo_2_duty_percent);
        servo_2_pwm_channel.set_duty(servo_2_duty_percent).unwrap();

        let sleep_duration = small_rng.gen_range(SERVO_MIN_SLEEP_MS..=SERVO_MAX_SLEEP_MS);
        log::info!("Sleeping for {} ms...", sleep_duration);
        delay.delay_millis(sleep_duration);

        log::info!("---");
    }
}
