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
use rand::rngs::SmallRng;
use rand::{Rng, SeedableRng};

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
    let mut power_led = Output::new(io.pins.gpio13, Level::High);
    power_led.set_high();
    log::info!("Power LED On");

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

    loop {
        log::info!("Looping...");

        for duty_num in SERVO_MIN_DUTY..=SERVO_MAX_DUTY {
            let next_random = small_rng.gen_range(SERVO_MIN_DUTY..=SERVO_MAX_DUTY);
            log::info!("Next Random: {}", next_random);
            log::info!("Duty: {}", duty_num);
            servo_1_pwm_channel.set_duty(duty_num).unwrap();
            servo_2_pwm_channel.set_duty(15 - duty_num).unwrap();
            // servo_2_pwm_channel.set_duty(duty_num).unwrap();
            delay.delay(2000.millis());
        }
    }
}
