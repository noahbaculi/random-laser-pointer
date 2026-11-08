//! Runtime settings, room presets, and their valid ranges.

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

/// Random-walk bounding box in duty counts
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Area {
    pub x_min: u16,
    pub x_max: u16,
    pub y_min: u16,
    pub y_max: u16,
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

/// Named room with its own bounding box
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Preset {
    pub name: Name,
    pub area: Area,
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
    /// Returns defaults with every preset covering the middle half of `travel`
    pub fn defaults(travel: Travel) -> Self {
        let quarter = (travel.max - travel.min) / 4;
        let area = Area {
            x_min: travel.min + quarter,
            x_max: travel.max - quarter,
            y_min: travel.min + quarter,
            y_max: travel.max - quarter,
        };
        Settings {
            presets: DEFAULT_NAMES.map(|n| Preset {
                name: Name::new(n).unwrap(),
                area,
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
            let a = &mut p.area;
            (a.x_min, a.x_max) = ordered(a.x_min, a.x_max, travel.min, travel.max);
            (a.y_min, a.y_max) = ordered(a.y_min, a.y_max, travel.min, travel.max);
        }
        (self.glide_min_us, self.glide_max_us) =
            ordered(self.glide_min_us, self.glide_max_us, 100, 50_000);
        (self.dwell_min_ms, self.dwell_max_ms) =
            ordered(self.dwell_min_ms, self.dwell_max_ms, 0, 60_000);
        self.manual_speed = self.manual_speed.clamp(50, 2000);
        self.active_min = self.active_min.clamp(1, 240);
        self.rest_min = self.rest_min.min(240);
    }

    /// Returns the active preset's area
    pub fn area(&self) -> Area {
        self.presets[self.preset as usize].area
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
    const DEFAULT_AREA: Area = Area {
        x_min: 859,
        x_max: 1598,
        y_min: 859,
        y_max: 1598,
    };

    #[test]
    fn defaults_match_spec() {
        let s = Settings::defaults(T);
        assert_eq!(s.preset, 0);
        for (i, p) in s.presets.iter().enumerate() {
            assert_eq!(p.name.as_str(), format!("Room {}", i + 1));
            assert_eq!(p.area, DEFAULT_AREA);
        }
        assert_eq!((s.glide_min_us, s.glide_max_us), (1000, 15000));
        assert_eq!((s.dwell_min_ms, s.dwell_max_ms), (900, 2000));
        assert_eq!(s.manual_speed, 450);
        assert_eq!(s.active_min, 10);
        assert_eq!(s.rest_min, 20);
    }

    #[test]
    fn clamp_limits_ranges() {
        let mut s = Settings::defaults(T);
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
    fn clamp_area_to_travel() {
        let mut s = Settings::defaults(T);
        s.presets[0].area.x_min = 0;
        s.presets[0].area.y_max = 4000;
        s.clamp(T);
        assert_eq!(s.presets[0].area.x_min, 491);
        assert_eq!(s.presets[0].area.y_max, 1966);
    }

    #[test]
    fn clamp_swaps_reversed_area() {
        let mut s = Settings::defaults(T);
        s.presets[1].area.x_min = 1500;
        s.presets[1].area.x_max = 900;
        s.clamp(T);
        assert_eq!(
            (s.presets[1].area.x_min, s.presets[1].area.x_max),
            (900, 1500)
        );
    }

    #[test]
    fn clamp_swaps_reversed_ranges() {
        let mut s = Settings::defaults(T);
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
        let mut s = Settings::defaults(T);
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
    fn area_follows_active_preset() {
        let mut s = Settings::defaults(T);
        let a = Area {
            x_min: 600,
            x_max: 700,
            y_min: 800,
            y_max: 900,
        };
        s.presets[2].area = a;
        s.preset = 2;
        assert_eq!(s.area(), a);
    }
}
