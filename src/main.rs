#![no_std]
#![no_main]

use core::cmp::Ordering;
use core::ops::Range;
use core::time::Duration;

use embedded_hal::pwm::SetDutyCycle;
use esp_backtrace as _;
use esp_hal::analog::adc::{Adc, AdcConfig, Attenuation};
use esp_hal::gpio::{Input, OutputPin, Pull};
use esp_hal::ledc::timer::TimerSpeed;
use esp_hal::ledc::{self, channel, timer, LSGlobalClkSource, Ledc, LowSpeed};
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
const SERVO_MAX_SLEEP_MS: u32 = 2 * 1_000;
const SERVO_MIN_STEP_SLEEP_US: u32 = 1_000;
const SERVO_MAX_STEP_SLEEP_US: u32 = 15_000;

const SERVO_MIN_DUTY_PERCENTAGE: f32 = 0.03;
const SERVO_MAX_DUTY_PERCENTAGE: f32 = 0.12;
const POT_MIN_VALUE: u16 = 0;
const POT_MAX_VALUE: u16 = 4095;

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

    // Instantiate Laser Diode
    let _laser_diode = Output::new(io.pins.gpio5, Level::High);
    log::info!("Laser Diode On");

    // Instantiate pins
    let pin_for_servo_x = io.pins.gpio0;
    let pin_for_servo_y = io.pins.gpio1;
    let pin_for_pot_x = io.pins.gpio3; // ADC pin
    let pin_for_pot_y = io.pins.gpio2; // ADC pin
    let pin_for_preview_btn = io.pins.gpio6; // ADC pin

    // Instantiate ADC
    let mut adc_config = AdcConfig::new();
    let mut pot_x_pin = adc_config.enable_pin(pin_for_pot_x, Attenuation::Attenuation11dB);
    let mut pot_y_pin = adc_config.enable_pin(pin_for_pot_y, Attenuation::Attenuation11dB);
    let mut adc = Adc::new(peripherals.ADC1, adc_config);

    let preview_btn = Input::new(pin_for_preview_btn, Pull::Up);

    // Create sleep timer wakeup source
    let sleep_timer = TimerWakeupSource::new(Duration::from_secs(SLEEP_DURATION_MIN as u64 * 60));

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

    assert_eq!(
        servo_x_pwm_channel.max_duty_cycle(),
        servo_y_pwm_channel.max_duty_cycle(),
        "Servo PWM channels should have the same max duty cycle"
    );
    let pwm_channel_max_duty = servo_x_pwm_channel.max_duty_cycle();
    let servo_min_duty = SERVO_MIN_DUTY_PERCENTAGE * pwm_channel_max_duty as f32;
    let servo_max_duty = SERVO_MAX_DUTY_PERCENTAGE * pwm_channel_max_duty as f32;
    log::info!(
        "Servo min duty = {} | Servo max duty = {}",
        servo_min_duty,
        servo_max_duty
    );

    let mut last_servo_x_duty = (servo_min_duty + ((servo_max_duty - servo_min_duty) / 2.0)) as u16;
    let mut last_servo_y_duty = (servo_min_duty + ((servo_max_duty - servo_min_duty) / 2.0)) as u16;

    log::info!("Starting loop...");
    loop {
        // Check uptime and enter deep sleep if needed
        let uptime_min = time::current_time().duration_since_epoch().to_minutes();
        log::info!("--- Uptime = {} min", uptime_min);
        if uptime_min >= ON_DURATION_MIN.into() {
            log::info!("Entering deep sleep for {} min...", SLEEP_DURATION_MIN);
            delay.delay_millis(100);
            rtc.sleep_deep(&[&sleep_timer]);
        }

        // Check potentiometer values
        let pot_x_value: u16 = nb::block!(adc.read_oneshot(&mut pot_x_pin)).unwrap();
        // Invert potentiometer value to match wired direction
        let servo_x_duty_range =
            pot_to_servo_duty(POT_MAX_VALUE - pot_x_value, servo_min_duty, servo_max_duty);
        log::info!(
            "Potentiometer X = {:4} => Duty range = {:?}",
            pot_x_value,
            servo_x_duty_range,
        );
        let pot_y_value: u16 = nb::block!(adc.read_oneshot(&mut pot_y_pin)).unwrap();
        // Invert potentiometer value to match wired direction
        let servo_y_duty_range =
            pot_to_servo_duty(POT_MAX_VALUE - pot_y_value, servo_min_duty, servo_max_duty);
        log::info!(
            "Potentiometer Y = {:4} => Duty range = {:?}",
            pot_y_value,
            servo_y_duty_range
        );

        if preview_btn.is_low() {
            log::info!("Preview button pressed!");
            let preview_servo_delay_ms = 500;
            servo_y_pwm_channel
                .set_duty_cycle(servo_y_duty_range.start)
                .unwrap();
            delay.delay_millis(preview_servo_delay_ms);
            servo_x_pwm_channel
                .set_duty_cycle(servo_x_duty_range.start)
                .unwrap();
            delay.delay_millis(preview_servo_delay_ms);
            servo_y_pwm_channel
                .set_duty_cycle(servo_y_duty_range.end)
                .unwrap();
            delay.delay_millis(preview_servo_delay_ms);
            servo_x_pwm_channel
                .set_duty_cycle(servo_x_duty_range.end)
                .unwrap();
            delay.delay_millis(preview_servo_delay_ms);
            continue;
        }

        // Move servos to random positions
        let servo_x_duty = match servo_x_duty_range.is_empty() {
            true => servo_x_duty_range.start,
            false => small_rng.gen_range(servo_x_duty_range),
        };
        let servo_y_duty = match servo_y_duty_range.is_empty() {
            true => servo_y_duty_range.start,
            false => small_rng.gen_range(servo_y_duty_range),
        };
        log::info!(
            "Servo X duty = {:>2} | Servo Y duty = {:>2}",
            servo_x_duty,
            servo_y_duty
        );
        move_servo_gradually(
            &mut servo_x_pwm_channel,
            &mut delay,
            &mut small_rng,
            last_servo_x_duty,
            servo_x_duty,
        );
        last_servo_x_duty = servo_x_duty;

        move_servo_gradually(
            &mut servo_y_pwm_channel,
            &mut delay,
            &mut small_rng,
            last_servo_y_duty,
            servo_y_duty,
        );
        last_servo_y_duty = servo_y_duty;

        // Sleep for random duration
        let sleep_duration = small_rng.gen_range(SERVO_MIN_SLEEP_MS..=SERVO_MAX_SLEEP_MS);
        log::info!("Sleeping for {} ms...", sleep_duration);
        delay.delay_millis(sleep_duration);
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
    ((in_value - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min
}

fn pot_to_servo_duty(pot_value: u16, min_servo_duty: f32, max_servo_duty: f32) -> Range<u16> {
    let servo_middle_duty = min_servo_duty + ((max_servo_duty - min_servo_duty) / 2.0);
    let range_radius = map_range(
        pot_value as f32,
        POT_MIN_VALUE as f32,
        POT_MAX_VALUE as f32,
        0.0,
        (max_servo_duty - min_servo_duty) / 2.0,
    );

    assert!(range_radius <= ((max_servo_duty - min_servo_duty) / 2.0));

    Range {
        start: (servo_middle_duty - range_radius) as u16,
        end: (servo_middle_duty + range_radius) as u16,
    }
}

fn move_servo_gradually<S, O>(
    servo_pwm_channel: &mut ledc::channel::Channel<S, O>,
    delay: &mut Delay,
    rng: &mut SmallRng,
    start_duty: u16,
    end_duty: u16,
) where
    S: TimerSpeed,
    O: OutputPin,
{
    log::info!(
        "Moving servo gradually from {} to {}...",
        start_duty,
        end_duty
    );
    match start_duty.cmp(&end_duty) {
        Ordering::Equal => (), // Do nothing
        Ordering::Less => {
            for duty_value in start_duty..=end_duty {
                servo_pwm_channel.set_duty_cycle(duty_value).unwrap();
                delay
                    .delay_micros(rng.gen_range(SERVO_MIN_STEP_SLEEP_US..=SERVO_MAX_STEP_SLEEP_US));
            }
        }
        Ordering::Greater => {
            for duty_value in (end_duty..=start_duty).rev() {
                servo_pwm_channel.set_duty_cycle(duty_value).unwrap();
                delay
                    .delay_micros(rng.gen_range(SERVO_MIN_STEP_SLEEP_US..=SERVO_MAX_STEP_SLEEP_US));
            }
        }
    }
}
