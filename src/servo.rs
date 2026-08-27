//! Servo PWM setup shared by both modes.

use embedded_hal::pwm::SetDutyCycle;
use esp_hal::gpio::DriveMode;
use esp_hal::ledc::channel::ChannelIFace;
use esp_hal::ledc::timer::TimerIFace;
use esp_hal::ledc::{channel, timer, LSGlobalClkSource, Ledc, LowSpeed};
use esp_hal::peripherals::{GPIO2, GPIO3, LEDC};
use esp_hal::time::Rate;

use crate::mk_static;

// 3-12% duty at 50 Hz spans full servo travel
pub const SERVO_MIN_DUTY_PERCENTAGE: f32 = 0.03;
pub const SERVO_MAX_DUTY_PERCENTAGE: f32 = 0.12;

/// Servo duty limits in PWM counts
#[derive(Clone, Copy)]
pub struct DutyBounds {
    pub min: f32,
    pub max: f32,
}

impl DutyBounds {
    pub fn center(&self) -> f32 {
        self.min + ((self.max - self.min) / 2.0)
    }
}

/// Both servo PWM channels and their shared duty bounds
pub struct Servos {
    pub x: channel::Channel<'static, LowSpeed>,
    pub y: channel::Channel<'static, LowSpeed>,
    pub bounds: DutyBounds,
}

/// Configures LEDC (low speed, 50 Hz, 14-bit) and centers both servos.
pub fn setup(ledc: LEDC<'static>, x_pin: GPIO3<'static>, y_pin: GPIO2<'static>) -> Servos {
    let ledc = mk_static!(Ledc<'static>, Ledc::new(ledc));
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    let pwm_timer = mk_static!(
        timer::Timer<'static, LowSpeed>,
        ledc.timer::<LowSpeed>(timer::Number::Timer0)
    );
    pwm_timer
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty14Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_hz(50),
        })
        .unwrap();
    let pwm_timer: &'static timer::Timer<'static, LowSpeed> = pwm_timer;

    let mut x = ledc.channel::<LowSpeed>(channel::Number::Channel0, x_pin);
    x.configure(channel::config::Config {
        timer: pwm_timer,
        duty_pct: 0,
        drive_mode: DriveMode::PushPull,
    })
    .unwrap();

    let mut y = ledc.channel::<LowSpeed>(channel::Number::Channel1, y_pin);
    y.configure(channel::config::Config {
        timer: pwm_timer,
        duty_pct: 0,
        drive_mode: DriveMode::PushPull,
    })
    .unwrap();

    let max_duty = x.max_duty_cycle() as f32;
    let bounds = DutyBounds {
        min: SERVO_MIN_DUTY_PERCENTAGE * max_duty,
        max: SERVO_MAX_DUTY_PERCENTAGE * max_duty,
    };
    x.set_duty_cycle(bounds.center() as u16).unwrap();
    y.set_duty_cycle(bounds.center() as u16).unwrap();
    log::info!("Servo duty range {} .. {}", bounds.min, bounds.max);

    Servos { x, y, bounds }
}
