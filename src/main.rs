#![no_std]
#![no_main]

use core::time::Duration;

use esp_backtrace as _;
use esp_hal::analog::adc::{Adc, AdcConfig, Attenuation};
use esp_hal::gpio::{Input, Pull};
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

const ON_DURATION_MIN: u8 = 10; // Duration laser should be active every cycle
const SLEEP_DURATION_MIN: u8 = 20; // Duration laser should be inactive every cycle
const SERVO_MIN_SLEEP_MS: u32 = 900;
const SERVO_MAX_SLEEP_MS: u32 = 5 * 1_000;

const SERVO_MIN_DUTY: u8 = 3;
const SERVO_MAX_DUTY: u8 = 13;
const SERVO_MIDDLE_DUTY: u8 = (SERVO_MAX_DUTY - SERVO_MIN_DUTY) / 2;
const POT_MIN_VALUE: u16 = 0;
const POT_MAX_VALUE: u16 = 4095;
const NUM_SERVO_LEVELS: u8 = (SERVO_MAX_DUTY - SERVO_MIN_DUTY) / 2;

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
    let pin_for_preview_btn = io.pins.gpio12; // ADC pin

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

    // Instantiate ADC
    let mut adc_config = AdcConfig::new();
    let mut pot_x_pin = adc_config.enable_pin(pin_for_pot_x, Attenuation::Attenuation11dB);
    let mut pot_y_pin = adc_config.enable_pin(pin_for_pot_y, Attenuation::Attenuation11dB);
    let mut adc = Adc::new(peripherals.ADC1, adc_config);

    let preview_btn = Input::new(pin_for_preview_btn, Pull::Up);

    // Create sleep timer wakeup source
    let sleep_timer = TimerWakeupSource::new(Duration::from_secs(SLEEP_DURATION_MIN as u64 * 60));

    log::info!("Starting loop... SERVO_LEVELS = {}", NUM_SERVO_LEVELS);
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
        let servo_x_duty_range = pot_to_servo_duty(pot_x_value);
        log::info!(
            "Potentiometer X value = {:>4} => {:?} | Potentiometer Y value = {:>4}",
            pot_x_value,
            servo_x_duty_range,
            pot_y_value
        );

        let servo_x_min_duty = SERVO_MIN_DUTY;
        let servo_x_max_duty = SERVO_MAX_DUTY;
        let servo_y_min_duty = SERVO_MIN_DUTY;
        let servo_y_max_duty = SERVO_MAX_DUTY;

        if preview_btn.is_low() {
            log::info!("Preview button pressed!");
            let preview_servo_delay_ms = 500;
            servo_y_pwm_channel.set_duty(servo_y_min_duty).unwrap();
            delay.delay_millis(preview_servo_delay_ms);
            servo_x_pwm_channel.set_duty(servo_x_min_duty).unwrap();
            delay.delay_millis(preview_servo_delay_ms);
            servo_y_pwm_channel.set_duty(servo_y_max_duty).unwrap();
            delay.delay_millis(preview_servo_delay_ms);
            servo_x_pwm_channel.set_duty(servo_x_max_duty).unwrap();
            delay.delay_millis(2 * preview_servo_delay_ms);
            continue;
        }

        // Move servos to random positions
        let servo_x_duty_percent = small_rng.gen_range(servo_x_min_duty..=servo_x_max_duty);
        servo_x_pwm_channel.set_duty(servo_x_duty_percent).unwrap();
        let servo_y_duty_percent = small_rng.gen_range(servo_y_min_duty..=servo_y_max_duty);
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

fn map_range<T>(in_value: T, in_min: T, in_max: T, out_min: T, out_max: T) -> T
where
    T: Copy
        + core::ops::Mul<Output = T>
        + core::ops::Add<Output = T>
        + core::ops::Div<Output = T>
        + core::ops::Sub<Output = T>,
{
    (in_value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
}

#[derive(Debug)]
struct ServoDutyPercentRange {
    min: u8,
    max: u8,
}

fn pot_to_servo_duty(pot_value: u16) -> ServoDutyPercentRange {
    let range_radius = map_range(
        pot_value,
        POT_MIN_VALUE,
        POT_MAX_VALUE,
        0,
        ((SERVO_MAX_DUTY - SERVO_MIN_DUTY) / 2) as u16,
    ) as u8;

    assert!(range_radius <= ((SERVO_MAX_DUTY - SERVO_MIN_DUTY) / 2));

    log::info!("Range radius = {}", range_radius);

    ServoDutyPercentRange {
        min: SERVO_MIDDLE_DUTY - range_radius,
        max: SERVO_MIDDLE_DUTY + range_radius,
    }
}
