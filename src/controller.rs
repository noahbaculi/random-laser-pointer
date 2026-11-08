//! Pure mode, phase, and motion state machine driven by a caller-supplied clock.

use rand::rngs::SmallRng;
use rand::{RngExt as _, SeedableRng};

use crate::geometry::{bbox, contains, segment_inside, Point};
use crate::settings::{Settings, Travel};

pub const MANUAL_TICK_US: u64 = 20_000;
// Tolerates one lost 120 ms keepalive plus jitter
pub const DEADMAN_US: u64 = 350_000;
/// Random target picks per move before dwelling instead
pub const TARGET_TRIES: u32 = 32;

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
    UpLeft,
    UpRight,
    DownLeft,
    DownRight,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    Active,
    Rest,
}

/// Reason Random holds with the laser off
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Idle {
    /// Active room has fewer than 3 points
    NoArea,
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
    pub idle: Option<Idle>,
    pub laser: bool,
    pub x: u16,
    pub y: u16,
}

/// Straight move from `from` to `to`, one count per step on the longer axis
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Line {
    from: Point,
    to: Point,
    step: u32,
    steps: u32,
}

impl Line {
    fn new(from: Point, to: Point) -> Line {
        let steps = from.x.abs_diff(to.x).max(from.y.abs_diff(to.y)) as u32;
        Line {
            from,
            to,
            step: 0,
            steps,
        }
    }

    fn done(&self) -> bool {
        self.step >= self.steps
    }

    /// Moves `n` steps along, stopping at `to`, and returns the new point
    fn advance(&mut self, n: u32) -> Point {
        if self.steps == 0 {
            return self.to;
        }
        self.step = (self.step + n).min(self.steps);
        let lerp = |a: u16, b: u16| {
            let (a, d, k, n) = (
                a as i64,
                b as i64 - a as i64,
                self.step as i64,
                self.steps as i64,
            );
            // Rounds half up so the minor axis moves at most one count per step
            (a + (2 * d * k + n).div_euclid(2 * n)) as u16
        };
        Point {
            x: lerp(self.from.x, self.to.x),
            y: lerp(self.from.y, self.to.y),
        }
    }
}

/// Random-walk progress
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Walk {
    Moving(Line),
    Dwell { until_us: u64 },
}

/// What Manual is doing
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Drive {
    Dpad,
    Goto(Line),
    /// `leg` indexes the point being approached, with the point count meaning the first again
    Preview {
        line: Line,
        leg: usize,
    },
}

pub struct Controller {
    travel: Travel,
    mode: Mode,
    phase: Phase,
    phase_end_us: u64,
    idle: bool,
    laser: bool,
    x: u16,
    y: u16,
    dir: Dir,
    dir_at_us: u64,
    next_tick_us: u64,
    // Manual position in thousandths of a count
    manual_x: u32,
    manual_y: u32,
    drive: Drive,
    // Goto and preview progress in thousandths of a count
    glide_milli: u32,
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
            idle: false,
            laser: false,
            x: travel.center(),
            y: travel.center(),
            dir: Dir::Stop,
            dir_at_us: 0,
            next_tick_us: 0,
            manual_x: 0,
            manual_y: 0,
            drive: Drive::Dpad,
            glide_milli: 0,
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

    /// Applies a D-pad command. Any direction but `Stop` takes over into Manual and cancels a glide
    pub fn manual(&mut self, dir: Dir, now_us: u64) {
        if dir != Dir::Stop && self.mode != Mode::Manual {
            self.mode = Mode::Manual;
            self.dir = Dir::Stop;
            self.enter_manual();
        }
        if self.mode != Mode::Manual {
            return;
        }
        if dir != Dir::Stop && self.drive != Drive::Dpad {
            self.enter_manual();
        }
        // Keepalives must not reset the tick schedule
        if dir != Dir::Stop && !self.gliding(now_us) {
            self.next_tick_us = now_us + MANUAL_TICK_US;
        }
        self.dir = dir;
        self.dir_at_us = now_us;
    }

    /// Switches to Manual and glides straight to `to`, clamped to travel
    pub fn goto(&mut self, to: Point, now_us: u64) {
        let to = Point {
            x: to.x.clamp(self.travel.min, self.travel.max),
            y: to.y.clamp(self.travel.min, self.travel.max),
        };
        self.start_glide(Drive::Goto(Line::new(self.here(), to)), now_us);
    }

    /// Switches to Manual and traces the active outline, then holds at its first point.
    /// Does nothing below 3 points
    pub fn preview(&mut self, now_us: u64, settings: &Settings) {
        let Some(&first) = settings.outline().points().first() else {
            return;
        };
        if settings.outline().points().len() < 3 {
            return;
        }
        let line = Line::new(self.here(), first);
        self.start_glide(Drive::Preview { line, leg: 0 }, now_us);
    }

    /// Drops Random's current move so the next target comes from the new outline
    pub fn outline_changed(&mut self, now_us: u64) {
        if let Walk::Moving(_) = self.walk {
            self.walk = Walk::Dwell { until_us: now_us };
            self.walk_at_us = now_us;
        }
    }

    /// Returns the current Random, goto, or preview destination
    pub fn target(&self) -> Option<Point> {
        match (self.mode, self.drive, self.walk) {
            (Mode::Random, _, Walk::Moving(line)) => Some(line.to),
            (Mode::Manual, Drive::Goto(line) | Drive::Preview { line, .. }, _) => Some(line.to),
            _ => None,
        }
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
            idle: (random && self.idle).then_some(Idle::NoArea),
            laser: self.laser,
            x: self.x,
            y: self.y,
        }
    }

    fn here(&self) -> Point {
        Point {
            x: self.x,
            y: self.y,
        }
    }

    fn enter_manual(&mut self) {
        self.laser = true;
        self.drive = Drive::Dpad;
        self.manual_x = self.x as u32 * MILLI;
        self.manual_y = self.y as u32 * MILLI;
    }

    fn start_glide(&mut self, drive: Drive, now_us: u64) {
        if self.mode != Mode::Manual {
            self.mode = Mode::Manual;
            self.enter_manual();
        }
        self.dir = Dir::Stop;
        self.drive = drive;
        self.glide_milli = 0;
        self.next_tick_us = now_us;
    }

    fn enter_phase(&mut self, phase: Phase, start_us: u64, settings: &Settings) {
        let minutes = match phase {
            Phase::Active => settings.active_min,
            Phase::Rest => settings.rest_min,
        };
        self.phase = phase;
        self.phase_end_us = start_us + minutes as u64 * US_PER_MIN;
        self.idle = settings.outline().points().len() < 3;
        self.laser = phase == Phase::Active && !self.idle;
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
        let idle = settings.outline().points().len() < 3;
        if idle != self.idle {
            self.idle = idle;
            // Restart the walk so recovery picks a target right away
            self.walk = Walk::Dwell { until_us: now_us };
            self.walk_at_us = now_us;
        }
        self.laser = self.phase == Phase::Active && !idle;
        if !self.laser {
            return (None, None, Some(self.phase_end_us));
        }
        let (x, y) = if now_us >= self.walk_at_us {
            self.walk(now_us, settings)
        } else {
            (None, None)
        };
        (x, y, Some(self.walk_at_us.min(self.phase_end_us)))
    }

    /// Moves one count, or starts a dwell, and sets the next walk deadline
    fn walk(&mut self, now_us: u64, settings: &Settings) -> (Option<u16>, Option<u16>) {
        match self.walk {
            Walk::Dwell { until_us } if now_us < until_us => {
                self.walk_at_us = until_us;
                (None, None)
            }
            Walk::Dwell { .. } => match self.pick_target(settings) {
                Some(to) if to != self.here() => {
                    self.walk = Walk::Moving(Line::new(self.here(), to));
                    self.walk(now_us, settings)
                }
                _ => {
                    self.dwell(now_us, settings);
                    (None, None)
                }
            },
            Walk::Moving(mut line) => {
                let p = line.advance(1);
                self.walk = Walk::Moving(line);
                let moved = (self.move_x(p.x), self.move_y(p.y));
                if line.done() {
                    self.dwell(now_us, settings);
                } else {
                    self.walk_at_us = now_us + self.glide_us(settings);
                }
                moved
            }
        }
    }

    fn dwell(&mut self, now_us: u64, settings: &Settings) {
        let dwell_ms = self
            .rng
            .random_range(settings.dwell_min_ms..=settings.dwell_max_ms);
        let until_us = now_us + (dwell_ms as u64 * 1000).max(MIN_DWELL_US);
        self.walk = Walk::Dwell { until_us };
        self.walk_at_us = until_us;
    }

    /// Returns a random point inside the active outline, by rejection within its
    /// bounding box. From inside, the straight path to it must stay inside too
    fn pick_target(&mut self, settings: &Settings) -> Option<Point> {
        let outline = settings.outline().points();
        let (lo, hi) = bbox(outline)?;
        let here = self.here();
        let inside = contains(outline, here);
        (0..TARGET_TRIES).find_map(|_| {
            let p = Point {
                x: self.rng.random_range(lo.x..=hi.x),
                y: self.rng.random_range(lo.y..=hi.y),
            };
            let ok = contains(outline, p) && (!inside || segment_inside(outline, here, p));
            ok.then_some(p)
        })
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
        let moving = match self.drive {
            Drive::Dpad => self.gliding(now_us),
            Drive::Goto(_) | Drive::Preview { .. } => true,
        };
        if !moving {
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
        let p = match self.drive {
            Drive::Dpad => self.nudge(delta),
            Drive::Goto(mut line) => {
                let p = line.advance(self.glide_counts(delta));
                self.drive = if line.done() {
                    Drive::Dpad
                } else {
                    Drive::Goto(line)
                };
                p
            }
            Drive::Preview { mut line, leg } => {
                let p = line.advance(self.glide_counts(delta));
                self.drive = Drive::Preview { line, leg };
                if line.done() {
                    self.next_leg(leg, p, settings);
                }
                p
            }
        };
        if self.drive == Drive::Dpad {
            self.manual_x = p.x as u32 * MILLI;
            self.manual_y = p.y as u32 * MILLI;
        }
        let (x, y) = (self.move_x(p.x), self.move_y(p.y));
        let next = (self.drive != Drive::Dpad || self.gliding(now_us)).then_some(self.next_tick_us);
        (x, y, next)
    }

    /// Returns whole counts to glide this tick from `delta` thousandths
    fn glide_counts(&mut self, delta: u32) -> u32 {
        self.glide_milli += delta;
        let n = self.glide_milli / MILLI;
        self.glide_milli %= MILLI;
        n
    }

    /// Heads to the point after `leg`, or holds once back at the first
    fn next_leg(&mut self, leg: usize, here: Point, settings: &Settings) {
        let points = settings.outline().points();
        let next = leg + 1;
        self.drive = match points.get(next % points.len().max(1)) {
            Some(&to) if leg < points.len() && points.len() >= 3 => Drive::Preview {
                line: Line::new(here, to),
                leg: next,
            },
            _ => Drive::Dpad,
        };
    }

    /// Returns the D-pad position after one tick of `delta` thousandths per axis
    fn nudge(&mut self, delta: u32) -> Point {
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
        let (dx, dy) = match self.dir {
            Dir::Stop => (None, None),
            Dir::Up => (None, Some(false)),
            Dir::Down => (None, Some(true)),
            Dir::Right => (Some(false), None),
            Dir::Left => (Some(true), None),
            Dir::UpLeft => (Some(true), Some(false)),
            Dir::UpRight => (Some(false), Some(false)),
            Dir::DownLeft => (Some(true), Some(true)),
            Dir::DownRight => (Some(false), Some(true)),
        };
        if let Some(up) = dx {
            self.manual_x = nudge(self.manual_x, up);
        }
        if let Some(up) = dy {
            self.manual_y = nudge(self.manual_y, up);
        }
        Point {
            x: (self.manual_x / MILLI) as u16,
            y: (self.manual_y / MILLI) as u16,
        }
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::settings::Outline;

    const T: Travel = Travel {
        min: 491,
        max: 1966,
    };
    const S: u64 = 1_000_000;
    const MS: u64 = 1_000;

    fn p(x: u16, y: u16) -> Point {
        Point { x, y }
    }

    fn square(lo: u16, hi: u16) -> Outline {
        Outline::new(&[p(lo, lo), p(hi, lo), p(hi, hi), p(lo, hi)]).unwrap()
    }

    // Defaults with room 0 set to the middle half of travel
    fn settings() -> Settings {
        let mut s = Settings::defaults(T);
        s.presets[0].outline = square(859, 1598);
        s
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
        assert!(c.status(t).x > bbox(s.outline().points()).unwrap().1.x);
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
        let mut c = boot(&s);
        let log = drive(&mut c, &s, 0, 2000);
        let xs: Vec<u16> = log.iter().filter_map(|(_, o)| o.x).collect();
        let ys: Vec<u16> = log.iter().filter_map(|(_, o)| o.y).collect();
        assert!(xs.len() > 100 && ys.len() > 100);
        for v in xs.iter().chain(&ys) {
            assert!((859..=1598).contains(v), "{v}");
        }
    }

    #[test]
    fn walk_steps_one_count_on_major_axis() {
        let s = short_dwell();
        let mut c = boot(&s);
        let mut last = (c.x, c.y);
        let mut moves = 0;
        for (_, o) in drive(&mut c, &s, 0, 3000) {
            let now = (o.x.unwrap_or(last.0), o.y.unwrap_or(last.1));
            if now != last {
                let (dx, dy) = (now.0.abs_diff(last.0), now.1.abs_diff(last.1));
                assert_eq!(dx.max(dy), 1, "{last:?} -> {now:?}");
                moves += 1;
            }
            last = now;
        }
        assert!(moves > 1000);
    }

    const L: [Point; 6] = [
        Point { x: 600, y: 600 },
        Point { x: 1800, y: 600 },
        Point { x: 1800, y: 1800 },
        Point { x: 1200, y: 1800 },
        Point { x: 1200, y: 1200 },
        Point { x: 600, y: 1200 },
    ];

    fn with_outline(points: &[Point]) -> Settings {
        let mut s = short_dwell();
        s.presets[0].outline = Outline::new(points).unwrap();
        s
    }

    #[test]
    fn l_shape_walk_never_leaves() {
        let s = with_outline(&L);
        for seed in 1..=5 {
            let mut c = Controller::new(0, T, &s, seed);
            let mut now = 0;
            for _ in 0..5000 {
                now = c.step(now, &s).next_us.unwrap();
                assert!(contains(&L, p(c.x, c.y)), "seed {seed} at {:?}", (c.x, c.y));
            }
        }
    }

    // Steps every manual tick until the glide stops, returning visited points
    fn glide(c: &mut Controller, s: &Settings, start: u64) -> (u64, Vec<Point>) {
        let mut now = start;
        let mut seen = vec![p(c.x, c.y)];
        loop {
            let next = c.step(now, s).next_us;
            seen.push(p(c.x, c.y));
            assert!(seen.len() < 10_000, "glide never stopped");
            match next {
                Some(next) => now = next,
                None => return (now, seen),
            }
        }
    }

    #[test]
    fn reentry_from_outside_then_stays_inside() {
        let s = with_outline(&L);
        let mut c = boot(&s);
        c.goto(p(700, 1700), S);
        let (now, _) = glide(&mut c, &s, S);
        assert!(!contains(&L, p(c.x, c.y)));
        c.set_mode(Mode::Random, now, &s);
        let mut t = now;
        let mut entered = false;
        for _ in 0..5000 {
            t = c.step(t, &s).next_us.unwrap();
            let inside = contains(&L, p(c.x, c.y));
            if entered {
                assert!(inside, "left at {:?}", (c.x, c.y));
            } else if matches!(c.walk, Walk::Dwell { .. }) {
                assert!(inside, "first move ended outside at {:?}", (c.x, c.y));
                entered = true;
            }
        }
        assert!(entered);
    }

    #[test]
    fn no_area_with_fewer_than_three() {
        for n in 0..3 {
            let mut s = settings();
            s.presets[0].outline = Outline::new(&L[..n]).unwrap();
            let mut c = boot(&s);
            let out = c.step(0, &s);
            assert!(!out.laser, "{n}");
            assert_eq!((out.x, out.y), (None, None));
            let st = c.status(0);
            assert!(!st.laser);
            assert_eq!(st.idle, Some(Idle::NoArea));
            let out = c.step(10 * S, &s);
            assert_eq!((out.x, out.y), (None, None));
        }
    }

    #[test]
    fn drop_below_three_goes_idle_and_recovers() {
        let tri = [p(800, 800), p(1600, 800), p(1200, 1600)];
        let mut s = with_outline(&tri);
        let mut c = boot(&s);
        let mut now = 0;
        for _ in 0..200 {
            now = c.step(now, &s).next_us.unwrap();
        }
        assert_eq!(c.status(now).idle, None);
        s.presets[0].outline = Outline::new(&tri[..2]).unwrap();
        let out = c.step(now, &s);
        assert!(!out.laser);
        assert_eq!((out.x, out.y), (None, None));
        assert_eq!(c.status(now).idle, Some(Idle::NoArea));
        s.presets[0].outline = Outline::new(&tri).unwrap();
        let out = c.step(now + S, &s);
        assert!(out.laser);
        assert_eq!(c.status(now + S).idle, None);
        let moved = drive(&mut c, &s, out.next_us.unwrap(), 500)
            .iter()
            .any(|(_, o)| o.x.is_some() || o.y.is_some());
        assert!(moved);
    }

    #[test]
    fn no_target_dwells_not_spins() {
        // Sliver whose only integer points lie on its long edge
        let mut s = with_outline(&[p(500, 500), p(1900, 1901), p(501, 500)]);
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
    fn diagonals_move_both_axes() {
        let s = settings();
        let t = 5 * S;
        let c0 = T.center();
        for (dir, x, y) in [
            (Dir::UpLeft, c0 + 9, c0 - 9),
            (Dir::UpRight, c0 - 9, c0 - 9),
            (Dir::DownLeft, c0 + 9, c0 + 9),
            (Dir::DownRight, c0 - 9, c0 + 9),
        ] {
            let mut c = boot(&s);
            c.manual(dir, t);
            let out = c.step(t + 20 * MS, &s);
            assert_eq!((out.x, out.y), (Some(x), Some(y)), "{dir:?}");
        }
    }

    #[test]
    fn goto_glides_straight_to_target() {
        let s = settings();
        let mut c = boot(&s);
        let (a, b) = (p(T.center(), T.center()), p(1500, 1000));
        c.goto(b, S);
        assert_eq!(c.status(S).mode, Mode::Manual);
        assert_eq!(c.target(), Some(b));
        let (_, seen) = glide(&mut c, &s, S);
        let (dx, dy) = (b.x as f64 - a.x as f64, b.y as f64 - a.y as f64);
        let len = (dx * dx + dy * dy).sqrt();
        for q in &seen {
            let off = ((q.x as f64 - a.x as f64) * dy - (q.y as f64 - a.y as f64) * dx).abs() / len;
            assert!(off <= 1.0, "{q:?} off line by {off}");
        }
        // Start, 272 counts at 9 per tick
        assert_eq!(seen.len(), 1 + 272usize.div_ceil(9));
        assert_eq!(*seen.last().unwrap(), b);
        assert_eq!(c.status(S).mode, Mode::Manual);
        assert!(c.status(S).laser);
        assert_eq!(c.target(), None);
    }

    #[test]
    fn goto_clamps_to_travel() {
        let s = settings();
        let mut c = boot(&s);
        c.goto(p(0, 5000), S);
        glide(&mut c, &s, S);
        assert_eq!((c.x, c.y), (491, 1966));
    }

    #[test]
    fn dpad_cancels_goto() {
        let s = settings();
        let mut c = boot(&s);
        c.goto(p(1500, 1500), S);
        c.step(S + 20 * MS, &s);
        let y = c.y;
        c.manual(Dir::Up, S + 30 * MS);
        assert_eq!(c.target(), None);
        c.step(S + 40 * MS, &s);
        c.step(S + 60 * MS, &s);
        assert!(c.y < y);
    }

    #[test]
    fn preview_visits_points_in_order() {
        let tri = [p(800, 800), p(1600, 800), p(1200, 1600)];
        let s = with_outline(&tri);
        let mut c = boot(&s);
        c.preview(S, &s);
        assert_eq!(c.status(S).mode, Mode::Manual);
        let (_, seen) = glide(&mut c, &s, S);
        let mut want = [tri[0], tri[1], tri[2], tri[0]].into_iter().peekable();
        for q in &seen {
            if want.peek() == Some(q) {
                want.next();
            }
        }
        assert_eq!(want.next(), None, "missed a point");
        assert_eq!(p(c.x, c.y), tri[0]);
        assert!(c.status(S).laser);
        assert_eq!(c.target(), None);
    }

    #[test]
    fn preview_below_three_is_noop() {
        let s = with_outline(&L[..2]);
        let mut c = boot(&s);
        c.manual(Dir::Up, S);
        c.manual(Dir::Stop, S);
        let before = (c.x, c.y);
        c.preview(2 * S, &s);
        assert_eq!(c.target(), None);
        assert_eq!(c.step(2 * S, &s).next_us, None);
        assert_eq!((c.x, c.y), before);
    }

    #[test]
    fn random_target_reported() {
        let s = short_dwell();
        let mut c = boot(&s);
        let mut now = 0;
        let to = loop {
            now = c.step(now, &s).next_us.unwrap();
            if let Some(to) = c.target() {
                break to;
            }
        };
        assert!(contains(s.outline().points(), to));
    }

    #[test]
    fn glide_delay_within_range() {
        let s = short_dwell();
        let mut c = boot(&s);
        let mut now = 0;
        let mut moves = 0;
        for _ in 0..2000 {
            let next = c.step(now, &s).next_us.unwrap();
            // Arrival steps start a dwell instead
            if matches!(c.walk, Walk::Moving(_)) {
                let d = next - now;
                assert!((1000..=15000).contains(&d), "glide {d}");
                moves += 1;
            }
            now = next;
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
            if matches!(c.walk, Walk::Dwell { .. }) {
                let d = next - now;
                assert!((900_000..=2_000_000).contains(&d), "dwell {d}");
                dwells += 1;
            }
            now = next;
        }
        assert!(dwells > 3);
    }

    #[test]
    fn outline_change_restarts_move() {
        let mut s = short_dwell();
        s.presets[1].outline = square(500, 600);
        let mut c = boot(&s);
        let mut now = 0;
        // Run until mid move
        loop {
            now = c.step(now, &s).next_us.unwrap();
            if let Some(to) = c.target() {
                if c.x.abs_diff(to.x).max(c.y.abs_diff(to.y)) > 2 {
                    break;
                }
            }
        }
        s.preset = 1;
        c.outline_changed(now);
        // One straight move into the new outline, then it never leaves
        let mut entered = false;
        for _ in 0..3000 {
            now = c.step(now, &s).next_us.unwrap();
            if let Some(to) = c.target() {
                assert!(
                    (500..=600).contains(&to.x) && (500..=600).contains(&to.y),
                    "{to:?}"
                );
            }
            let inside = (500..=600).contains(&c.x) && (500..=600).contains(&c.y);
            if entered {
                assert!(inside, "left at {:?}", (c.x, c.y));
            }
            entered |= inside && matches!(c.walk, Walk::Dwell { .. });
        }
        assert!(entered);
    }

    #[test]
    fn goto_current_position_holds() {
        let s = settings();
        let mut c = boot(&s);
        let here = p(c.x, c.y);
        c.goto(here, S);
        let (_, seen) = glide(&mut c, &s, S);
        assert!(seen.iter().all(|q| *q == here));
        assert_eq!(c.target(), None);
    }

    #[test]
    fn preview_again_from_first_point() {
        let tri = [p(800, 800), p(1600, 800), p(1200, 1600)];
        let s = with_outline(&tri);
        let mut c = boot(&s);
        c.preview(S, &s);
        let (now, _) = glide(&mut c, &s, S);
        assert_eq!(p(c.x, c.y), tri[0]);
        c.preview(now, &s);
        let (_, seen) = glide(&mut c, &s, now);
        assert!(seen.contains(&tri[1]) && seen.contains(&tri[2]));
        assert_eq!(p(c.x, c.y), tri[0]);
    }

    #[test]
    fn tiny_outline_stays_put() {
        let mut s = short_dwell();
        s.presets[0].outline =
            Outline::new(&[p(1000, 1000), p(1001, 1000), p(1000, 1001)]).unwrap();
        let mut c = boot(&s);
        let log = drive(&mut c, &s, 0, 3000);
        assert!(log.iter().all(|(_, o)| o.next_us.is_some()));
        assert!((1000..=1001).contains(&c.x) && (1000..=1001).contains(&c.y));
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
        let dx = out.x.map_or(0, |x| x.abs_diff(held.0));
        let dy = out.y.map_or(0, |y| y.abs_diff(held.1));
        assert!(dx.max(dy) <= 1);
    }

    #[test]
    fn fixed_point_zero_dwell_never_spins() {
        let mut s = settings();
        s.presets[0].outline =
            Outline::new(&[p(1000, 1000), p(1001, 1000), p(1000, 1001)]).unwrap();
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
