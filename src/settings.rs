//! Runtime settings, room presets, and their valid ranges.

use crate::geometry::Point;

/// Servo duty limits in PWM counts
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Travel {
    pub min: u16,
    pub max: u16,
}

impl Travel {
    pub fn center(&self) -> u16 {
        self.min + (self.max - self.min) / 2
    }
}

pub const MAX_POINTS: usize = 16;

/// Room outline of up to `MAX_POINTS` duty-count points
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Outline {
    points: [Point; MAX_POINTS],
    len: u8,
}

impl Outline {
    pub const EMPTY: Outline = Outline {
        points: [Point { x: 0, y: 0 }; MAX_POINTS],
        len: 0,
    };

    /// Returns `None` above `MAX_POINTS` points
    pub fn new(points: &[Point]) -> Option<Outline> {
        let mut o = Outline::EMPTY;
        o.points.get_mut(..points.len())?.copy_from_slice(points);
        o.len = points.len() as u8;
        Some(o)
    }

    pub fn points(&self) -> &[Point] {
        &self.points[..self.len as usize]
    }
}

pub const NAME_MAX_LEN: usize = 16;

/// Preset name, printable ASCII without `"` or `\` so it embeds in JSON unescaped
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Name {
    bytes: [u8; NAME_MAX_LEN],
    len: u8,
}

impl Name {
    /// Returns `None` unless `s` is 1 to 16 printable ASCII chars without `"` or `\`
    pub fn new(s: &str) -> Option<Name> {
        let src = s.as_bytes();
        let valid = |b: &u8| (0x20..=0x7E).contains(b) && *b != b'"' && *b != b'\\';
        if src.is_empty() || src.len() > NAME_MAX_LEN || !src.iter().all(valid) {
            return None;
        }
        let mut bytes = [0; NAME_MAX_LEN];
        bytes[..src.len()].copy_from_slice(src);
        Some(Name {
            bytes,
            len: src.len() as u8,
        })
    }

    pub fn as_str(&self) -> &str {
        core::str::from_utf8(&self.bytes[..self.len as usize]).unwrap_or("")
    }
}

/// Named room with its own outline
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Preset {
    pub name: Name,
    pub outline: Outline,
}

pub const PRESET_COUNT: usize = 4;

const DEFAULT_NAMES: [&str; PRESET_COUNT] = ["Room 1", "Room 2", "Room 3", "Room 4"];

/// Every runtime-adjustable value
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Settings {
    pub presets: [Preset; PRESET_COUNT],
    pub preset: u8,
    pub glide_min_us: u32,
    pub glide_max_us: u32,
    pub dwell_min_ms: u32,
    pub dwell_max_ms: u32,
    /// Counts per second
    pub manual_speed: u16,
    pub active_min: u16,
    /// 0 disables the rest phase
    pub rest_min: u16,
}

impl Settings {
    /// Returns defaults with every room's outline empty
    pub fn defaults() -> Self {
        Settings {
            presets: DEFAULT_NAMES.map(|n| Preset {
                name: Name::new(n).unwrap(),
                outline: Outline::EMPTY,
            }),
            preset: 0,
            glide_min_us: 1000,
            glide_max_us: 15000,
            dwell_min_ms: 900,
            dwell_max_ms: 2000,
            manual_speed: 450,
            active_min: 10,
            rest_min: 20,
        }
    }

    /// Clamps every field into its valid range and swaps reversed min/max pairs
    pub fn clamp(&mut self, travel: Travel) {
        self.preset = self.preset.min(PRESET_COUNT as u8 - 1);
        for p in &mut self.presets {
            let len = p.outline.len as usize;
            for pt in &mut p.outline.points[..len] {
                pt.x = pt.x.clamp(travel.min, travel.max);
                pt.y = pt.y.clamp(travel.min, travel.max);
            }
        }
        (self.glide_min_us, self.glide_max_us) =
            ordered(self.glide_min_us, self.glide_max_us, 100, 50_000);
        (self.dwell_min_ms, self.dwell_max_ms) =
            ordered(self.dwell_min_ms, self.dwell_max_ms, 0, 60_000);
        self.manual_speed = self.manual_speed.clamp(50, 2000);
        self.active_min = self.active_min.clamp(1, 240);
        self.rest_min = self.rest_min.min(240);
    }

    /// Returns the active room's outline
    pub fn outline(&self) -> &Outline {
        &self.presets[self.preset as usize].outline
    }
}

/// Clamps both ends into `lo..=hi` and returns them in ascending order
fn ordered<T: Ord + Copy>(a: T, b: T, lo: T, hi: T) -> (T, T) {
    let (a, b) = (a.clamp(lo, hi), b.clamp(lo, hi));
    (a.min(b), a.max(b))
}

#[cfg(test)]
mod tests {
    use super::*;

    const T: Travel = Travel {
        min: 491,
        max: 1966,
    };

    fn p(x: u16, y: u16) -> Point {
        Point { x, y }
    }

    #[test]
    fn defaults_match_spec() {
        let s = Settings::defaults();
        assert_eq!(s.preset, 0);
        for (i, p) in s.presets.iter().enumerate() {
            assert_eq!(p.name.as_str(), format!("Room {}", i + 1));
            assert_eq!(p.outline, Outline::EMPTY);
        }
        assert_eq!((s.glide_min_us, s.glide_max_us), (1000, 15000));
        assert_eq!((s.dwell_min_ms, s.dwell_max_ms), (900, 2000));
        assert_eq!(s.manual_speed, 450);
        assert_eq!(s.active_min, 10);
        assert_eq!(s.rest_min, 20);
    }

    #[test]
    fn clamp_limits_ranges() {
        let mut s = Settings::defaults();
        s.glide_min_us = 10;
        s.glide_max_us = 99_999;
        s.manual_speed = 0;
        s.active_min = 0;
        s.rest_min = 999;
        s.dwell_max_ms = 70_000;
        s.preset = 9;
        s.clamp(T);
        assert_eq!(s.glide_min_us, 100);
        assert_eq!(s.glide_max_us, 50_000);
        assert_eq!(s.manual_speed, 50);
        assert_eq!(s.active_min, 1);
        assert_eq!(s.rest_min, 240);
        assert_eq!(s.dwell_max_ms, 60_000);
        assert_eq!(s.preset, 3);
    }

    #[test]
    fn outline_rejects_more_than_16() {
        let pts = [p(500, 500); 17];
        assert!(Outline::new(&pts).is_none());
        assert_eq!(Outline::new(&pts[..16]).unwrap().points().len(), 16);
    }

    #[test]
    fn clamp_moves_points_into_travel() {
        let mut s = Settings::defaults();
        s.presets[0].outline = Outline::new(&[p(0, 4000), p(1000, 1000)]).unwrap();
        s.clamp(T);
        assert_eq!(
            s.presets[0].outline.points(),
            &[p(491, 1966), p(1000, 1000)]
        );
    }

    #[test]
    fn clamp_swaps_reversed_ranges() {
        let mut s = Settings::defaults();
        s.glide_min_us = 9000;
        s.glide_max_us = 2000;
        s.dwell_min_ms = 9000;
        s.dwell_max_ms = 2000;
        s.clamp(T);
        assert_eq!((s.glide_min_us, s.glide_max_us), (2000, 9000));
        assert_eq!((s.dwell_min_ms, s.dwell_max_ms), (2000, 9000));
    }

    #[test]
    fn rest_zero_is_kept() {
        let mut s = Settings::defaults();
        s.rest_min = 0;
        s.clamp(T);
        assert_eq!(s.rest_min, 0);
    }

    #[test]
    fn name_rules() {
        assert_eq!(Name::new("Bedroom").unwrap().as_str(), "Bedroom");
        assert!(Name::new("").is_none());
        assert!(Name::new("abcdefghijklmnopq").is_none());
        assert!(Name::new("a\"b").is_none());
        assert!(Name::new("a\\b").is_none());
        assert!(Name::new("caf\u{e9}").is_none());
        assert!(Name::new("a\tb").is_none());
        assert_eq!(
            Name::new("abcdefghijklmnop").unwrap().as_str(),
            "abcdefghijklmnop"
        );
    }

    #[test]
    fn outline_follows_active_preset() {
        let mut s = Settings::defaults();
        let o = Outline::new(&[p(600, 800), p(700, 800), p(700, 900)]).unwrap();
        s.presets[2].outline = o;
        s.preset = 2;
        assert_eq!(*s.outline(), o);
    }
}
