//! Fixed-layout flash record for `Settings`, little-endian with a CRC-32 trailer.

use crate::geometry::{self_crossing, Point};
use crate::settings::{Name, Outline, Settings, Travel, MAX_POINTS, NAME_MAX_LEN, PRESET_COUNT};

const MAGIC: u16 = 0x1A5E;
const VERSION: u8 = 2;
// Name length, name, point count, points
const PRESET_LEN: usize = 1 + NAME_MAX_LEN + 1 + MAX_POINTS * 4;
// Header, presets, globals, 2 pad bytes
const BODY_LEN: usize = 4 + PRESET_COUNT * PRESET_LEN + 4 * 4 + 3 * 2 + 2;
/// Whole words, since `esp-storage` writes 4-byte units
pub const RECORD_LEN: usize = BODY_LEN + 4;

/// Little-endian byte cursor over a record
struct Cursor<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl Cursor<'_> {
    fn put(&mut self, bytes: &[u8]) {
        self.buf[self.pos..self.pos + bytes.len()].copy_from_slice(bytes);
        self.pos += bytes.len();
    }

    fn take<const N: usize>(&mut self) -> [u8; N] {
        let mut out = [0; N];
        out.copy_from_slice(&self.buf[self.pos..self.pos + N]);
        self.pos += N;
        out
    }

    fn u8(&mut self) -> u8 {
        self.take::<1>()[0]
    }

    fn u16(&mut self) -> u16 {
        u16::from_le_bytes(self.take())
    }

    fn u32(&mut self) -> u32 {
        u32::from_le_bytes(self.take())
    }
}

pub fn encode(settings: &Settings) -> [u8; RECORD_LEN] {
    let mut buf = [0; RECORD_LEN];
    let mut c = Cursor {
        buf: &mut buf,
        pos: 0,
    };
    c.put(&MAGIC.to_le_bytes());
    c.put(&[VERSION, settings.preset]);
    for p in &settings.presets {
        let name = p.name.as_str().as_bytes();
        let mut padded = [0; NAME_MAX_LEN];
        padded[..name.len()].copy_from_slice(name);
        c.put(&[name.len() as u8]);
        c.put(&padded);
        let points = p.outline.points();
        c.put(&[points.len() as u8]);
        for i in 0..MAX_POINTS {
            let pt = points.get(i).copied().unwrap_or(Point { x: 0, y: 0 });
            c.put(&pt.x.to_le_bytes());
            c.put(&pt.y.to_le_bytes());
        }
    }
    for v in [
        settings.glide_min_us,
        settings.glide_max_us,
        settings.dwell_min_ms,
        settings.dwell_max_ms,
    ] {
        c.put(&v.to_le_bytes());
    }
    for v in [
        settings.manual_speed,
        settings.active_min,
        settings.rest_min,
    ] {
        c.put(&v.to_le_bytes());
    }
    seal(&mut buf);
    buf
}

/// Returns clamped settings, or `None` on bad magic, version, checksum, name, or point count.
/// A self-crossing outline loads empty
pub fn decode(buf: &[u8; RECORD_LEN], travel: Travel) -> Option<Settings> {
    let (body, sum) = buf.split_at(BODY_LEN);
    if crc32(body).to_le_bytes() != sum {
        return None;
    }
    let mut copy = *buf;
    let mut c = Cursor {
        buf: &mut copy,
        pos: 0,
    };
    if c.u16() != MAGIC || c.u8() != VERSION {
        return None;
    }
    let mut s = Settings::defaults();
    s.preset = c.u8();
    for p in &mut s.presets {
        let len = c.u8() as usize;
        let bytes = c.take::<NAME_MAX_LEN>();
        p.name = core::str::from_utf8(bytes.get(..len)?)
            .ok()
            .and_then(Name::new)?;
        let count = c.u8() as usize;
        let points: [Point; MAX_POINTS] = core::array::from_fn(|_| Point {
            x: c.u16(),
            y: c.u16(),
        });
        p.outline = Outline::new(points.get(..count)?)?;
    }
    s.glide_min_us = c.u32();
    s.glide_max_us = c.u32();
    s.dwell_min_ms = c.u32();
    s.dwell_max_ms = c.u32();
    s.manual_speed = c.u16();
    s.active_min = c.u16();
    s.rest_min = c.u16();
    s.clamp(travel);
    for p in &mut s.presets {
        if self_crossing(p.outline.points()) {
            p.outline = Outline::EMPTY;
        }
    }
    Some(s)
}

/// Writes the checksum trailer over the body
fn seal(buf: &mut [u8; RECORD_LEN]) {
    let sum = crc32(&buf[..BODY_LEN]);
    buf[BODY_LEN..].copy_from_slice(&sum.to_le_bytes());
}

/// Returns the CRC-32 (IEEE) of `data`
fn crc32(data: &[u8]) -> u32 {
    let mut crc = !0u32;
    for &b in data {
        crc ^= b as u32;
        for _ in 0..8 {
            crc = (crc >> 1) ^ (0xEDB8_8320 & (crc & 1).wrapping_neg());
        }
    }
    !crc
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Point;
    use crate::settings::{Name, Outline, MAX_POINTS};

    fn p(x: u16, y: u16) -> Point {
        Point { x, y }
    }

    // Offset of preset `i`'s point count
    fn count_at(i: usize) -> usize {
        4 + i * PRESET_LEN + 1 + NAME_MAX_LEN
    }

    const T: Travel = Travel {
        min: 491,
        max: 1966,
    };

    fn edited() -> Settings {
        let mut s = Settings::defaults();
        s.presets[0].name = Name::new("Kitchen").unwrap();
        s.presets[3].name = Name::new("abcdefghijklmnop").unwrap();
        s.presets[3].outline = Outline::new(&[p(600, 491), p(1700, 491), p(1700, 1966)]).unwrap();
        s.preset = 3;
        s.glide_min_us = 250;
        s.glide_max_us = 40_000;
        s.dwell_min_ms = 0;
        s.dwell_max_ms = 60_000;
        s.manual_speed = 1234;
        s.active_min = 240;
        s.rest_min = 0;
        s
    }

    #[test]
    fn record_len_is_word_multiple() {
        assert_eq!(RECORD_LEN % 4, 0);
    }

    #[test]
    fn round_trip_defaults() {
        let s = Settings::defaults();
        assert_eq!(decode(&encode(&s), T), Some(s));
    }

    #[test]
    fn round_trip_edited() {
        let s = edited();
        assert_eq!(decode(&encode(&s), T), Some(s));
    }

    #[test]
    fn decode_erased_sector_is_none() {
        assert_eq!(decode(&[0xFF; RECORD_LEN], T), None);
    }

    #[test]
    fn decode_zeroed_is_none() {
        assert_eq!(decode(&[0; RECORD_LEN], T), None);
    }

    #[test]
    fn decode_wrong_magic_is_none() {
        let mut buf = encode(&edited());
        buf[0] ^= 1;
        seal(&mut buf);
        assert_eq!(decode(&buf, T), None);
    }

    #[test]
    fn decode_version_1_is_none() {
        let mut buf = encode(&edited());
        buf[2] = 1;
        seal(&mut buf);
        assert_eq!(decode(&buf, T), None);
    }

    #[test]
    fn round_trip_with_outlines() {
        let mut s = edited();
        s.presets[1].outline = Outline::new(&[p(900, 900), p(1500, 900), p(1500, 1500)]).unwrap();
        let star: Vec<Point> = (0..MAX_POINTS)
            .map(|i| {
                let a = i as f64 * core::f64::consts::PI / 8.0;
                let r = if i % 2 == 0 { 400.0 } else { 160.0 };
                p((1228.0 + r * a.cos()) as u16, (1228.0 + r * a.sin()) as u16)
            })
            .collect();
        s.presets[2].outline = Outline::new(&star).unwrap();
        assert_eq!(s.presets[0].outline.points().len(), 0);
        assert_eq!(decode(&encode(&s), T), Some(s));
    }

    #[test]
    fn decode_count_over_16_is_none() {
        let mut buf = encode(&edited());
        buf[count_at(0)] = 17;
        seal(&mut buf);
        assert_eq!(decode(&buf, T), None);
    }

    #[test]
    fn decode_resets_only_crossing_room() {
        let mut s = edited();
        // Bow-tie, written raw since encode trusts its input
        s.presets[0].outline =
            Outline::new(&[p(600, 600), p(1000, 1000), p(1000, 600), p(600, 1000)]).unwrap();
        let buf = encode(&s);
        let mut want = s;
        want.presets[0].outline = Outline::EMPTY;
        assert_eq!(decode(&buf, T), Some(want));
    }

    #[test]
    fn decode_bad_checksum_is_none() {
        let mut buf = encode(&edited());
        buf[40] ^= 0x10;
        assert_eq!(decode(&buf, T), None);
    }

    #[test]
    fn decode_torn_record_is_none() {
        let mut buf = encode(&edited());
        buf[RECORD_LEN / 2..].fill(0xFF);
        assert_eq!(decode(&buf, T), None);
    }

    #[test]
    fn decode_clamps_out_of_range() {
        let mut s = edited();
        s.manual_speed = 9000;
        let decoded = decode(&encode(&s), T).unwrap();
        assert_eq!(decoded.manual_speed, 2000);
    }

    #[test]
    fn decode_rejects_bad_name() {
        let mut buf = encode(&edited());
        // First preset's name length byte
        buf[4] = 20;
        seal(&mut buf);
        assert_eq!(decode(&buf, T), None);
        buf[4] = 0;
        seal(&mut buf);
        assert_eq!(decode(&buf, T), None);
    }
}
