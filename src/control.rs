//! Shared controller state and the task that owns the servos and laser pin.

use core::cell::RefCell;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::{with_deadline, Instant};
use embedded_hal::pwm::SetDutyCycle;
use esp_hal::gpio::{Level, Output};
use random_laser_pointer::api::State;

use crate::servo::Servos;

pub static STATE: Mutex<CriticalSectionRawMutex, RefCell<Option<State>>> =
    Mutex::new(RefCell::new(None));

/// Raised on any mode, D-pad, or settings change so the controller steps now
pub static WAKE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Raised by `/set` and `/points` to schedule a flash save
pub static SETTINGS_CHANGED: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Runs `f` on the shared state. `None` before `main` fills it
pub fn with_state<R>(f: impl FnOnce(&mut State) -> R) -> Option<R> {
    STATE.lock(|cell| cell.borrow_mut().as_mut().map(f))
}

pub fn now_us() -> u64 {
    Instant::now().as_micros()
}

#[embassy_executor::task]
pub async fn controller_task(servos: Servos, mut laser: Output<'static>) {
    let Servos { mut x, mut y, .. } = servos;
    loop {
        let out = with_state(|s| s.controller.step(now_us(), &s.settings));
        let Some(out) = out else {
            WAKE.wait().await;
            continue;
        };

        laser.set_level(Level::from(out.laser));
        if let Some(duty) = out.x {
            if let Err(e) = x.set_duty_cycle(duty) {
                log::warn!("x duty {duty}: {e:?}");
            }
        }
        if let Some(duty) = out.y {
            if let Err(e) = y.set_duty_cycle(duty) {
                log::warn!("y duty {duty}: {e:?}");
            }
        }

        match out.next_us {
            Some(at) => _ = with_deadline(Instant::from_micros(at), WAKE.wait()).await,
            None => WAKE.wait().await,
        }
    }
}
