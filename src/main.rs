#![no_std]
#![no_main]

use core::time::Duration;

use esp_backtrace as _;
use esp_hal::analog::adc::{Adc, AdcConfig, Attenuation};
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
    let mut small_rng = SmallRng::seed_from_u64(1); // seed irrelevant for random number generation
    esp_println::logger::init_logger_from_env();

    // Instantiate LED
    let _power_led = Output::new(io.pins.gpio13, Level::High);
    log::info!("Power LED On");

    // Instantiate pins
    let pin_for_servo_x = io.pins.gpio4;
    let pin_for_servo_y = io.pins.gpio5;
    let pin_for_pot_x = io.pins.gpio32; // ADC pin
    let pin_for_pot_y = io.pins.gpio33; // ADC pin

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

    // Instantiate PWM channels
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

    let mut adc_config = AdcConfig::new();
    let mut pot_x_pin = adc_config.enable_pin(pin_for_pot_x, Attenuation::Attenuation11dB);
    let mut pot_y_pin = adc_config.enable_pin(pin_for_pot_y, Attenuation::Attenuation11dB);
    let mut adc = Adc::new(peripherals.ADC1, adc_config);

    // Create sleep timer wakeup source
    let sleep_timer = TimerWakeupSource::new(Duration::from_secs(SLEEP_DURATION_MIN as u64 * 60));

    log::info!("Starting loop...");
    loop {
        // Check uptime and enter deep sleep if needed
        let uptime_min = time::current_time().duration_since_epoch().to_minutes();
        log::info!("Uptime = {} min", uptime_min);
        if uptime_min >= ON_DURATION_MIN.into() {
            log::info!("Entering deep sleep for {} min...", SLEEP_DURATION_MIN);
            delay.delay_millis(100);
            rtc.sleep_deep(&[&sleep_timer], &mut delay);
        }

        // Check potentiometer values
        let pot_x_value: u16 = nb::block!(adc.read_oneshot(&mut pot_x_pin)).unwrap();
        let pot_y_value: u16 = nb::block!(adc.read_oneshot(&mut pot_y_pin)).unwrap();
        log::info!(
            "Potentiometer X value = {:>4} | Potentiometer Y value = {:>4}",
            pot_x_value,
            pot_y_value
        );

        // Move servos to random positions
        let servo_x_duty_percent = small_rng.gen_range(SERVO_MIN_DUTY..=SERVO_MAX_DUTY);
        servo_x_pwm_channel.set_duty(servo_x_duty_percent).unwrap();
        let servo_y_duty_percent = small_rng.gen_range(SERVO_MIN_DUTY..=SERVO_MAX_DUTY);
        servo_y_pwm_channel.set_duty(servo_y_duty_percent).unwrap();
        log::info!(
            "Servo 1 duty = {:>2}% | Servo 2 duty = {:>2}%",
            servo_x_duty_percent,
            servo_y_duty_percent
        );

        // Sleep for random duration
        let sleep_duration = small_rng.gen_range(SERVO_MIN_SLEEP_MS..=SERVO_MAX_SLEEP_MS);
        log::info!("Sleeping for {} ms...", sleep_duration);
        delay.delay_millis(sleep_duration);

        log::info!("---");
    }
}
