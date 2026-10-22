//! Random-walk mode.
//!
//! The laser walks randomly inside a bounding box sized live by the two
//! potentiometers. Latching the preview button sweeps the box corners.
//! After 10 min of uptime the unit deep sleeps for 20 min, and the RTC
//! timer wakeup reboots it back through the boot-time mode strap.

use core::cmp::Ordering;
use core::ops::Range;

use embassy_time::{Instant, Timer};
use embedded_hal::pwm::SetDutyCycle;
use esp_hal::analog::adc::{Adc, AdcConfig, Attenuation};
use esp_hal::gpio::Input;
use esp_hal::ledc::{channel, LowSpeed};
use esp_hal::peripherals::{ADC1, GPIO0, GPIO1, LPWR};
use esp_hal::rng::Rng;
use esp_hal::rtc_cntl::sleep::{LowPower, RtcSleepConfig};
use esp_hal::time::{Duration, Instant as HalInstant};
use rand::rngs::SmallRng;
use rand::{RngExt as _, SeedableRng};

use crate::servo::Servos;

const SERVO_MIN_SLEEP_MS: u64 = 900;
const SERVO_MAX_SLEEP_MS: u64 = 2_000;
const SERVO_MIN_STEP_SLEEP_US: u64 = 1_000;
const SERVO_MAX_STEP_SLEEP_US: u64 = 15_000;
const POT_MIN_VALUE: u16 = 0;
const POT_MAX_VALUE: u16 = 4095;
const PREVIEW_SERVO_DELAY_MS: u64 = 500;
const ON_DURATION_MIN: u64 = 10; // Uptime before each deep sleep
const SLEEP_DURATION_MIN: u64 = 20; // Deep sleep duration per cycle

/// Runs the random walk forever. Never initializes the radio.
pub async fn run(
    servos: Servos,
    adc1: ADC1<'static>,
    pot_x_pin: GPIO0<'static>,
    pot_y_pin: GPIO1<'static>,
    lpwr: LPWR<'static>,
    preview_btn: Input<'static>,
) -> ! {
    let Servos {
        mut x,
        mut y,
        bounds,
    } = servos;

    let mut adc_config = AdcConfig::new();
    let mut pot_x = adc_config.enable_pin(pot_x_pin, Attenuation::_11dB);
    let mut pot_y = adc_config.enable_pin(pot_y_pin, Attenuation::_11dB);
    let mut adc = Adc::new(adc1, adc_config);

    let mut low_power = LowPower::new(lpwr);

    // Hardware-seeded so each boot walks a fresh sequence
    let hw_rng = Rng::new();
    let seed = (hw_rng.random() as u64) << 32 | hw_rng.random() as u64;
    let mut rng = SmallRng::seed_from_u64(seed);

    let mut last_x_duty = bounds.center() as u16;
    let mut last_y_duty = bounds.center() as u16;

    log::info!("Starting random walk...");
    loop {
        let uptime_min = Instant::now().as_secs() / 60;
        log::info!("--- Uptime = {} min", uptime_min);
        if uptime_min >= ON_DURATION_MIN {
            log::info!("Entering deep sleep for {} min...", SLEEP_DURATION_MIN);
            // Let the log line drain before the chip powers down
            Timer::after_millis(100).await;
            low_power.set_wakeup_deadline(
                HalInstant::now() + Duration::from_minutes(SLEEP_DURATION_MIN),
            );
            low_power.sleep_deep(RtcSleepConfig::deep());
        }

        let pot_x_value: u16 = nb::block!(adc.read_oneshot(&mut pot_x)).unwrap();
        // Invert potentiometer value to match wired direction
        let x_duty_range = pot_to_servo_duty(POT_MAX_VALUE - pot_x_value, bounds.min, bounds.max);
        log::info!(
            "Potentiometer X = {:4} => Duty range = {:?}",
            pot_x_value,
            x_duty_range,
        );
        let pot_y_value: u16 = nb::block!(adc.read_oneshot(&mut pot_y)).unwrap();
        // Invert potentiometer value to match wired direction
        let y_duty_range = pot_to_servo_duty(POT_MAX_VALUE - pot_y_value, bounds.min, bounds.max);
        log::info!(
            "Potentiometer Y = {:4} => Duty range = {:?}",
            pot_y_value,
            y_duty_range
        );

        if preview_btn.is_low() {
            log::info!("Preview button pressed!");
            y.set_duty_cycle(y_duty_range.start).unwrap();
            Timer::after_millis(PREVIEW_SERVO_DELAY_MS).await;
            x.set_duty_cycle(x_duty_range.start).unwrap();
            Timer::after_millis(PREVIEW_SERVO_DELAY_MS).await;
            y.set_duty_cycle(y_duty_range.end).unwrap();
            Timer::after_millis(PREVIEW_SERVO_DELAY_MS).await;
            x.set_duty_cycle(x_duty_range.end).unwrap();
            Timer::after_millis(PREVIEW_SERVO_DELAY_MS).await;
            continue;
        }

        let x_duty = match x_duty_range.is_empty() {
            true => x_duty_range.start,
            false => rng.random_range(x_duty_range),
        };
        let y_duty = match y_duty_range.is_empty() {
            true => y_duty_range.start,
            false => rng.random_range(y_duty_range),
        };
        log::info!(
            "Servo X duty = {:>2} | Servo Y duty = {:>2}",
            x_duty,
            y_duty
        );
        move_servo_gradually(&mut x, &mut rng, last_x_duty, x_duty).await;
        last_x_duty = x_duty;
        move_servo_gradually(&mut y, &mut rng, last_y_duty, y_duty).await;
        last_y_duty = y_duty;

        let sleep_duration = rng.random_range(SERVO_MIN_SLEEP_MS..=SERVO_MAX_SLEEP_MS);
        log::info!("Sleeping for {} ms...", sleep_duration);
        Timer::after_millis(sleep_duration).await;
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

/// Returns a duty window centered on servo middle, sized by the pot value.
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

/// Ramps one servo between duties with randomized 1-15 ms step delays.
async fn move_servo_gradually(
    servo_pwm_channel: &mut channel::Channel<'static, LowSpeed>,
    rng: &mut SmallRng,
    start_duty: u16,
    end_duty: u16,
) {
    log::info!(
        "Moving servo gradually from {} to {}...",
        start_duty,
        end_duty
    );
    match start_duty.cmp(&end_duty) {
        Ordering::Equal => (),
        Ordering::Less => {
            for duty_value in start_duty..=end_duty {
                servo_pwm_channel.set_duty_cycle(duty_value).unwrap();
                Timer::after_micros(
                    rng.random_range(SERVO_MIN_STEP_SLEEP_US..=SERVO_MAX_STEP_SLEEP_US),
                )
                .await;
            }
        }
        Ordering::Greater => {
            for duty_value in (end_duty..=start_duty).rev() {
                servo_pwm_channel.set_duty_cycle(duty_value).unwrap();
                Timer::after_micros(
                    rng.random_range(SERVO_MIN_STEP_SLEEP_US..=SERVO_MAX_STEP_SLEEP_US),
                )
                .await;
            }
        }
    }
}
