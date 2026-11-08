//! Pure mode, phase, and motion state machine driven by a caller-supplied clock.

use rand::rngs::SmallRng;
use rand::{RngExt as _, SeedableRng};

use crate::settings::{Settings, Travel};

pub const MANUAL_TICK_US: u64 = 20_000;
// Tolerates one lost 120 ms keepalive plus jitter
pub const DEADMAN_US: u64 = 350_000;

const US_PER_MIN: u64 = 60_000_000;
// Keeps a zero dwell on a fixed point from spinning the controller task
const MIN_DWELL_US: u64 = 1_000;
// Manual position resolution, so slow speeds still move every tick
const MILLI: u32 = 1000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    Random,
    Manual,
    Off,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Dir {
    Stop,
    Up,
    Down,
    Left,
    Right,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    Active,
    Rest,
}

/// Pin writes for one step. `x` and `y` are `Some` only when the duty changed
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Output {
    pub laser: bool,
    pub x: Option<u16>,
    pub y: Option<u16>,
    /// Next step deadline. `None` waits for a wake signal only
    pub next_us: Option<u64>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Status {
    pub mode: Mode,
    /// `None` outside Random
    pub phase: Option<Phase>,
    pub phase_remaining_s: u32,
    pub laser: bool,
    pub x: u16,
    pub y: u16,
}

/// Random-walk progress. Both targets are picked when a move starts
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Walk {
    MovingX { x: u16, y: u16 },
    MovingY { y: u16 },
    Dwell { until_us: u64 },
}

pub struct Controller {
    travel: Travel,
    mode: Mode,
    phase: Phase,
    phase_end_us: u64,
    laser: bool,
    x: u16,
    y: u16,
    dir: Dir,
    dir_at_us: u64,
    next_tick_us: u64,
    // Manual position in thousandths of a count
    manual_x: u32,
    manual_y: u32,
    walk: Walk,
    walk_at_us: u64,
    rng: SmallRng,
}

impl Controller {
    /// Returns a controller in Random at the start of an active phase, servos centered
    pub fn new(now_us: u64, travel: Travel, settings: &Settings, seed: u64) -> Self {
        let mut c = Controller {
            travel,
            mode: Mode::Off,
            phase: Phase::Active,
            phase_end_us: 0,
            laser: false,
            x: travel.center(),
            y: travel.center(),
            dir: Dir::Stop,
            dir_at_us: 0,
            next_tick_us: 0,
            manual_x: 0,
            manual_y: 0,
            walk: Walk::Dwell { until_us: 0 },
            walk_at_us: 0,
            rng: SmallRng::seed_from_u64(seed),
        };
        c.set_mode(Mode::Random, now_us, settings);
        c
    }

    /// Switches mode. Entering Random, or picking it during rest, restarts the active phase
    pub fn set_mode(&mut self, mode: Mode, now_us: u64, settings: &Settings) {
        let resting = self.mode == Mode::Random && self.phase == Phase::Rest;
        if mode == self.mode && !(mode == Mode::Random && resting) {
            return;
        }
        self.mode = mode;
        self.dir = Dir::Stop;
        match mode {
            Mode::Random => self.enter_phase(Phase::Active, now_us, settings),
            Mode::Manual => self.enter_manual(),
            Mode::Off => self.laser = false,
        }
    }

    /// Applies a D-pad command. Any direction but `Stop` takes over into Manual
    pub fn manual(&mut self, dir: Dir, now_us: u64) {
        if dir != Dir::Stop && self.mode != Mode::Manual {
            self.mode = Mode::Manual;
            self.dir = Dir::Stop;
            self.enter_manual();
        }
        if self.mode != Mode::Manual {
            return;
        }
        // Keepalives must not reset the tick schedule
        if dir != Dir::Stop && !self.gliding(now_us) {
            self.next_tick_us = now_us + MANUAL_TICK_US;
        }
        self.dir = dir;
        self.dir_at_us = now_us;
    }

    /// Advances to `now_us` and returns the pin writes and next deadline
    pub fn step(&mut self, now_us: u64, settings: &Settings) -> Output {
        let (x, y, next_us) = match self.mode {
            Mode::Off => (None, None, None),
            Mode::Manual => self.step_manual(now_us, settings),
            Mode::Random => self.step_random(now_us, settings),
        };
        Output {
            laser: self.laser,
            x,
            y,
            next_us,
        }
    }

    pub fn status(&self, now_us: u64) -> Status {
        let random = self.mode == Mode::Random;
        let remaining_s = self.phase_end_us.saturating_sub(now_us).div_ceil(1_000_000);
        Status {
            mode: self.mode,
            phase: random.then_some(self.phase),
            phase_remaining_s: if random { remaining_s as u32 } else { 0 },
            laser: self.laser,
            x: self.x,
            y: self.y,
        }
    }

    fn enter_manual(&mut self) {
        self.laser = true;
        self.manual_x = self.x as u32 * MILLI;
        self.manual_y = self.y as u32 * MILLI;
    }

    fn enter_phase(&mut self, phase: Phase, start_us: u64, settings: &Settings) {
        let minutes = match phase {
            Phase::Active => settings.active_min,
            Phase::Rest => settings.rest_min,
        };
        self.phase = phase;
        self.phase_end_us = start_us + minutes as u64 * US_PER_MIN;
        self.laser = phase == Phase::Active;
        // Pick a fresh target from the held position
        self.walk = Walk::Dwell { until_us: start_us };
        self.walk_at_us = start_us;
    }

    fn step_random(
        &mut self,
        now_us: u64,
        settings: &Settings,
    ) -> (Option<u16>, Option<u16>, Option<u64>) {
        // Boundaries chain from the stored end so phases never drift
        while now_us >= self.phase_end_us {
            let next = match self.phase {
                Phase::Active if settings.rest_min > 0 => Phase::Rest,
                _ => Phase::Active,
            };
            self.enter_phase(next, self.phase_end_us, settings);
        }
        match self.phase {
            Phase::Rest => (None, None, Some(self.phase_end_us)),
            Phase::Active => {
                let (x, y) = if now_us >= self.walk_at_us {
                    self.walk(now_us, settings)
                } else {
                    (None, None)
                };
                (x, y, Some(self.walk_at_us.min(self.phase_end_us)))
            }
        }
    }

    /// Moves one count, or starts a dwell, and sets the next walk deadline
    fn walk(&mut self, now_us: u64, settings: &Settings) -> (Option<u16>, Option<u16>) {
        loop {
            match self.walk {
                Walk::Dwell { until_us } if now_us >= until_us => {
                    let a = settings.area();
                    self.walk = Walk::MovingX {
                        x: self.rng.random_range(a.x_min..=a.x_max),
                        y: self.rng.random_range(a.y_min..=a.y_max),
                    };
                }
                Walk::Dwell { until_us } => {
                    self.walk_at_us = until_us;
                    return (None, None);
                }
                Walk::MovingX { x, y } if self.x == x => self.walk = Walk::MovingY { y },
                Walk::MovingX { x, .. } => {
                    let next = toward(self.x, x);
                    self.walk_at_us = now_us + self.glide_us(settings);
                    return (self.move_x(next), None);
                }
                Walk::MovingY { y } if self.y == y => {
                    let dwell_ms = self
                        .rng
                        .random_range(settings.dwell_min_ms..=settings.dwell_max_ms);
                    let until_us = now_us + (dwell_ms as u64 * 1000).max(MIN_DWELL_US);
                    self.walk = Walk::Dwell { until_us };
                    self.walk_at_us = until_us;
                    return (None, None);
                }
                Walk::MovingY { y } => {
                    let next = toward(self.y, y);
                    self.walk_at_us = now_us + self.glide_us(settings);
                    return (None, self.move_y(next));
                }
            }
        }
    }

    fn glide_us(&mut self, settings: &Settings) -> u64 {
        self.rng
            .random_range(settings.glide_min_us..=settings.glide_max_us) as u64
    }

    fn gliding(&self, now_us: u64) -> bool {
        self.dir != Dir::Stop && now_us.saturating_sub(self.dir_at_us) <= DEADMAN_US
    }

    fn step_manual(
        &mut self,
        now_us: u64,
        settings: &Settings,
    ) -> (Option<u16>, Option<u16>, Option<u64>) {
        if !self.gliding(now_us) {
            return (None, None, None);
        }
        if now_us < self.next_tick_us {
            return (None, None, Some(self.next_tick_us));
        }
        self.next_tick_us += MANUAL_TICK_US;
        if self.next_tick_us <= now_us {
            self.next_tick_us = now_us + MANUAL_TICK_US;
        }

        let delta = settings.manual_speed as u32 * (MANUAL_TICK_US / 1000) as u32;
        let (lo, hi) = (
            self.travel.min as u32 * MILLI,
            self.travel.max as u32 * MILLI,
        );
        let nudge = |v: u32, up: bool| {
            if up {
                v + delta
            } else {
                v.saturating_sub(delta)
            }
            .clamp(lo, hi)
        };
        // Inverted to match the servo mounting orientation
        match self.dir {
            Dir::Stop => {}
            Dir::Up => self.manual_y = nudge(self.manual_y, false),
            Dir::Down => self.manual_y = nudge(self.manual_y, true),
            Dir::Right => self.manual_x = nudge(self.manual_x, false),
            Dir::Left => self.manual_x = nudge(self.manual_x, true),
        }
        let x = self.move_x((self.manual_x / MILLI) as u16);
        let y = self.move_y((self.manual_y / MILLI) as u16);
        (x, y, Some(self.next_tick_us))
    }

    fn move_x(&mut self, x: u16) -> Option<u16> {
        (x != self.x).then(|| {
            self.x = x;
            x
        })
    }

    fn move_y(&mut self, y: u16) -> Option<u16> {
        (y != self.y).then(|| {
            self.y = y;
            y
        })
    }
}

/// Returns `from` moved one count toward `to`
fn toward(from: u16, to: u16) -> u16 {
    if to > from {
        from + 1
    } else {
        from - 1
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const T: Travel = Travel {
        min: 491,
        max: 1966,
    };
    const S: u64 = 1_000_000;
    const MS: u64 = 1_000;

    fn settings() -> Settings {
        Settings::defaults(T)
    }

    fn boot(s: &Settings) -> Controller {
        Controller::new(0, T, s, 1)
    }

    #[test]
    fn boots_random_active_laser_on() {
        let s = settings();
        let st = boot(&s).status(0);
        assert_eq!(st.mode, Mode::Random);
        assert_eq!(st.phase, Some(Phase::Active));
        assert!(st.laser);
        assert_eq!(st.phase_remaining_s, 600);
        assert_eq!((st.x, st.y), (T.center(), T.center()));
    }

    #[test]
    fn active_then_rest_then_active() {
        let s = settings();
        let mut c = boot(&s);
        let out = c.step(600 * S, &s);
        assert!(!out.laser);
        assert_eq!(c.status(600 * S).phase, Some(Phase::Rest));
        let out = c.step(1800 * S, &s);
        assert!(out.laser);
        assert_eq!(c.status(1800 * S).phase, Some(Phase::Active));
    }

    #[test]
    fn rest_zero_never_rests() {
        let mut s = settings();
        s.rest_min = 0;
        let mut c = boot(&s);
        for t in [600 * S, 1200 * S] {
            assert!(c.step(t, &s).laser);
            assert_eq!(c.status(t).phase, Some(Phase::Active));
        }
    }

    #[test]
    fn rest_holds_position() {
        let s = settings();
        let mut c = boot(&s);
        c.step(600 * S, &s);
        let out = c.step(700 * S, &s);
        assert_eq!((out.x, out.y), (None, None));
        assert!(!out.laser);
        assert_eq!(out.next_us, Some(1800 * S));
    }

    #[test]
    fn phase_length_change_applies_at_next_boundary() {
        let mut s = settings();
        let mut c = boot(&s);
        c.step(300 * S, &s);
        s.active_min = 1;
        c.step(599 * S, &s);
        assert_eq!(c.status(599 * S).phase, Some(Phase::Active));
        c.step(600 * S, &s);
        assert_eq!(c.status(600 * S).phase, Some(Phase::Rest));
        // Rest started with rest_min 20, so it keeps its 1800 s end
        s.rest_min = 1;
        c.step(700 * S, &s);
        c.step(1799 * S, &s);
        assert_eq!(c.status(1799 * S).phase, Some(Phase::Rest));
        c.step(1800 * S, &s);
        assert_eq!(c.status(1800 * S).phase, Some(Phase::Active));
        c.step(1860 * S, &s);
        assert_eq!(c.status(1860 * S).phase, Some(Phase::Rest));
        c.step(1920 * S, &s);
        assert_eq!(c.status(1920 * S).phase, Some(Phase::Active));
    }

    #[test]
    fn entering_random_restarts_active() {
        let s = settings();
        let mut c = boot(&s);
        c.set_mode(Mode::Off, 100 * S, &s);
        c.set_mode(Mode::Random, 200 * S, &s);
        let st = c.status(200 * S);
        assert_eq!(st.phase, Some(Phase::Active));
        assert_eq!(st.phase_remaining_s, 600);
    }

    #[test]
    fn off_laser_low_no_motion() {
        let s = settings();
        let mut c = boot(&s);
        c.set_mode(Mode::Off, 10 * S, &s);
        let out = c.step(11 * S, &s);
        assert_eq!(
            out,
            Output {
                laser: false,
                x: None,
                y: None,
                next_us: None
            }
        );
        assert_eq!(c.status(11 * S).phase, None);
    }

    #[test]
    fn dpad_switches_to_manual() {
        let s = settings();
        let mut from_random = boot(&s);
        let mut from_off = boot(&s);
        from_off.set_mode(Mode::Off, S, &s);
        let mut from_rest = boot(&s);
        from_rest.step(600 * S, &s);
        for (c, t) in [
            (&mut from_random, 5 * S),
            (&mut from_off, 5 * S),
            (&mut from_rest, 700 * S),
        ] {
            c.manual(Dir::Up, t);
            let st = c.status(t);
            assert_eq!(st.mode, Mode::Manual);
            assert!(st.laser);
            assert!(c.step(t, &s).laser);
        }
    }

    #[test]
    fn dpad_stop_does_not_switch() {
        let s = settings();
        let mut c = boot(&s);
        c.manual(Dir::Stop, S);
        assert_eq!(c.status(S).mode, Mode::Random);
    }

    #[test]
    fn manual_up_moves_y_down_9_per_tick() {
        let s = settings();
        let mut c = boot(&s);
        let t = 5 * S;
        c.manual(Dir::Up, t);
        let out = c.step(t + 20 * MS, &s);
        assert_eq!(out.y, Some(T.center() - 9));
        assert_eq!(out.x, None);
        assert_eq!(out.next_us, Some(t + 40 * MS));
    }

    #[test]
    fn manual_mapping_is_inverted() {
        let s = settings();
        let t = 5 * S;
        let c0 = T.center();
        for (dir, x, y) in [
            (Dir::Down, None, Some(c0 + 9)),
            (Dir::Right, Some(c0 - 9), None),
            (Dir::Left, Some(c0 + 9), None),
        ] {
            let mut c = boot(&s);
            c.manual(dir, t);
            let out = c.step(t + 20 * MS, &s);
            assert_eq!((out.x, out.y), (x, y), "{dir:?}");
        }
    }

    #[test]
    fn manual_clamps_to_travel_not_area() {
        let s = settings();
        let mut c = boot(&s);
        let mut t = S;
        // Full travel at 450 counts/s takes about 3.3 s
        while t < 6 * S {
            if t.is_multiple_of(100 * MS) {
                c.manual(Dir::Left, t);
            }
            c.step(t, &s);
            t += 20 * MS;
        }
        assert_eq!(c.status(t).x, 1966);
        assert!(c.status(t).x > s.area().x_max);
    }

    #[test]
    fn deadman_stops_after_350ms() {
        let s = settings();
        let mut c = boot(&s);
        let t = 5 * S;
        c.manual(Dir::Up, t);
        let mut ys = Vec::new();
        for i in 1..=20 {
            let now = t + i * 20 * MS;
            c.step(now, &s);
            ys.push((now - t, c.status(now).y));
        }
        let at = |dt: u64| ys.iter().find(|(d, _)| *d == dt).unwrap().1;
        assert!(at(340 * MS) < at(320 * MS));
        assert_eq!(at(360 * MS), at(340 * MS));
        assert_eq!(at(400 * MS), at(340 * MS));
        assert_eq!(c.step(t + 420 * MS, &s).next_us, None);
    }

    #[test]
    fn manual_stays_until_mode_change() {
        let s = settings();
        let mut c = boot(&s);
        c.manual(Dir::Up, S);
        c.step(S + 20 * MS, &s);
        let later = S + 600 * S;
        c.step(later, &s);
        assert_eq!(c.status(later).mode, Mode::Manual);
        assert!(c.status(later).laser);
    }

    // Follows `next_us` from `start` and records every step's time and output
    fn drive(c: &mut Controller, s: &Settings, start: u64, steps: usize) -> Vec<(u64, Output)> {
        let mut now = start;
        let mut log = Vec::new();
        for _ in 0..steps {
            let out = c.step(now, s);
            log.push((now, out));
            now = out.next_us.expect("random active always has a deadline");
        }
        log
    }

    fn short_dwell() -> Settings {
        let mut s = settings();
        s.dwell_min_ms = 0;
        s.dwell_max_ms = 5;
        s
    }

    #[test]
    fn targets_stay_inside_area() {
        let s = short_dwell();
        let a = s.area();
        let mut c = boot(&s);
        let log = drive(&mut c, &s, 0, 2000);
        let xs: Vec<u16> = log.iter().filter_map(|(_, o)| o.x).collect();
        let ys: Vec<u16> = log.iter().filter_map(|(_, o)| o.y).collect();
        assert!(xs.len() > 100 && ys.len() > 100);
        for x in xs.iter().skip_while(|x| !(a.x_min..=a.x_max).contains(x)) {
            assert!((a.x_min..=a.x_max).contains(x), "x {x}");
        }
        for y in ys.iter().skip_while(|y| !(a.y_min..=a.y_max).contains(y)) {
            assert!((a.y_min..=a.y_max).contains(y), "y {y}");
        }
    }

    #[test]
    fn moves_x_fully_then_y() {
        let s = short_dwell();
        let mut c = boot(&s);
        let mut now = 0;
        for _ in 0..2000 {
            let out = c.step(now, &s);
            if let Walk::MovingX { x, .. } = c.walk {
                assert_eq!(out.y, None, "y moved before x reached {x}");
            }
            if out.y.is_some() {
                assert!(matches!(c.walk, Walk::MovingY { .. }));
            }
            now = out.next_us.unwrap();
        }
    }

    #[test]
    fn one_count_per_step() {
        let s = short_dwell();
        let mut c = boot(&s);
        let log = drive(&mut c, &s, 0, 2000);
        let mut last = (T.center(), T.center());
        for (_, o) in log {
            if let Some(x) = o.x {
                assert_eq!(x.abs_diff(last.0), 1);
                last.0 = x;
            }
            if let Some(y) = o.y {
                assert_eq!(y.abs_diff(last.1), 1);
                last.1 = y;
            }
            assert!(o.x.is_none() || o.y.is_none());
        }
    }

    #[test]
    fn glide_delay_within_range() {
        let s = short_dwell();
        let mut c = boot(&s);
        let log = drive(&mut c, &s, 0, 2000);
        let mut moves = 0;
        for (now, o) in log {
            if o.x.is_some() || o.y.is_some() {
                let d = o.next_us.unwrap() - now;
                assert!((1000..=15000).contains(&d), "glide {d}");
                moves += 1;
            }
        }
        assert!(moves > 1000);
    }

    #[test]
    fn dwell_within_range() {
        let s = settings();
        let mut c = boot(&s);
        let mut now = 0;
        let mut dwells = 0;
        for _ in 0..5000 {
            let out = c.step(now, &s);
            let next = out.next_us.unwrap();
            if out.x.is_none() && out.y.is_none() && matches!(c.walk, Walk::Dwell { .. }) {
                let d = next - now;
                assert!((900_000..=2_000_000).contains(&d), "dwell {d}");
                dwells += 1;
            }
            now = next;
        }
        assert!(dwells > 3);
    }

    #[test]
    fn next_target_uses_new_area() {
        let mut s = short_dwell();
        let new = crate::settings::Area {
            x_min: 500,
            x_max: 600,
            y_min: 500,
            y_max: 600,
        };
        s.presets[1].area = new;
        let mut c = boot(&s);
        let mut now = 0;
        // Run until mid X move
        let old_x = loop {
            let out = c.step(now, &s);
            now = out.next_us.unwrap();
            if let Walk::MovingX { x, .. } = c.walk {
                if c.x.abs_diff(x) > 2 {
                    break x;
                }
            }
        };
        s.preset = 1;
        // Current axis keeps its old target
        loop {
            let out = c.step(now, &s);
            now = out.next_us.unwrap();
            match c.walk {
                Walk::MovingX { x, .. } => assert_eq!(x, old_x),
                _ => break,
            }
        }
        assert_eq!(c.x, old_x);
        // Next picked target lands in the new area
        let mut dwelled = false;
        loop {
            let out = c.step(now, &s);
            now = out.next_us.unwrap();
            match c.walk {
                Walk::Dwell { .. } => dwelled = true,
                Walk::MovingX { x, y } if dwelled => {
                    assert!((new.x_min..=new.x_max).contains(&x));
                    assert!((new.y_min..=new.y_max).contains(&y));
                    break;
                }
                Walk::MovingY { y } if dwelled => {
                    assert!((new.y_min..=new.y_max).contains(&y));
                    break;
                }
                _ => {}
            }
        }
    }

    #[test]
    fn degenerate_area_is_fixed_point() {
        let mut s = short_dwell();
        s.presets[0].area.x_min = 1000;
        s.presets[0].area.x_max = 1000;
        let mut c = boot(&s);
        drive(&mut c, &s, 0, 3000);
        assert_eq!(c.x, 1000);
    }

    #[test]
    fn rest_interrupts_move() {
        let mut s = settings();
        s.active_min = 1;
        s.rest_min = 1;
        s.glide_min_us = 50_000;
        s.glide_max_us = 50_000;
        let mut c = boot(&s);
        let mut now = 0;
        while now < 60 * S {
            let out = c.step(now, &s);
            assert!(out.next_us.unwrap() <= 60 * S);
            now = out.next_us.unwrap();
        }
        let held = (c.x, c.y);
        let out = c.step(60 * S, &s);
        assert!(!out.laser);
        assert_eq!(out.next_us, Some(120 * S));
        assert_eq!((out.x, out.y), (None, None));
        let out = c.step(90 * S, &s);
        assert_eq!((out.x, out.y), (None, None));
        assert_eq!((c.x, c.y), held);
        let out = c.step(120 * S, &s);
        assert!(out.laser);
        let first = out
            .x
            .map(|x| x.abs_diff(held.0))
            .or(out.y.map(|y| y.abs_diff(held.1)));
        assert!(matches!(first, None | Some(1)));
    }

    #[test]
    fn fixed_point_zero_dwell_never_spins() {
        let mut s = settings();
        s.presets[0].area = crate::settings::Area {
            x_min: 1000,
            x_max: 1000,
            y_min: 1000,
            y_max: 1000,
        };
        s.dwell_min_ms = 0;
        s.dwell_max_ms = 0;
        let mut c = boot(&s);
        let mut now = 0;
        for _ in 0..3000 {
            let next = c.step(now, &s).next_us.unwrap();
            assert!(next > now, "deadline {next} not after {now}");
            now = next;
        }
    }

    #[test]
    fn random_during_rest_restarts_active() {
        let s = settings();
        let mut c = boot(&s);
        c.step(600 * S, &s);
        c.set_mode(Mode::Random, 700 * S, &s);
        let st = c.status(700 * S);
        assert_eq!(st.phase, Some(Phase::Active));
        assert!(st.laser);
        assert_eq!(st.phase_remaining_s, 600);
    }

    #[test]
    fn random_during_active_keeps_timer() {
        let s = settings();
        let mut c = boot(&s);
        c.set_mode(Mode::Random, 100 * S, &s);
        assert_eq!(c.status(100 * S).phase_remaining_s, 500);
    }
}
