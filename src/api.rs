//! HTTP request routing, `/set` query handling, and status JSON.

use core::fmt::Write;

use crate::controller::{Dir, Mode, Phase, Status};
use crate::settings::{Name, Settings, Travel, NAME_MAX_LEN, PRESET_COUNT};

// Fits four 16-char names with every field at its type max
pub const STATUS_BUF_LEN: usize = 1024;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Route<'a> {
    Page,
    Dpad(Dir),
    Mode(Mode),
    /// Raw query string, still percent-encoded
    Set(&'a str),
    Status,
    NotFound,
    BadRequest,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SetError {
    UnknownKey,
    BadValue,
    BadName,
}

/// Maps a raw request head to a route. Anything but `GET` is `NotFound`
pub fn route(request: &str) -> Route<'_> {
    let mut parts = request.lines().next().unwrap_or("").split(' ');
    let (Some("GET"), Some(target)) = (parts.next(), parts.next()) else {
        return Route::NotFound;
    };
    let (path, query) = target.split_once('?').unwrap_or((target, ""));
    match path {
        "/" => Route::Page,
        "/d" => match query {
            "dir=up" => Route::Dpad(Dir::Up),
            "dir=down" => Route::Dpad(Dir::Down),
            "dir=left" => Route::Dpad(Dir::Left),
            "dir=right" => Route::Dpad(Dir::Right),
            "dir=stop" => Route::Dpad(Dir::Stop),
            _ => Route::BadRequest,
        },
        "/mode" => match query {
            "m=random" => Route::Mode(Mode::Random),
            "m=manual" => Route::Mode(Mode::Manual),
            "m=off" => Route::Mode(Mode::Off),
            _ => Route::BadRequest,
        },
        "/set" => Route::Set(query),
        "/status" => Route::Status,
        _ => Route::NotFound,
    }
}

/// Applies `key=value` pairs in order, then clamps. Changes nothing on error
pub fn apply_set(query: &str, settings: &mut Settings, travel: Travel) -> Result<(), SetError> {
    let mut s = *settings;
    for pair in query.split('&').filter(|p| !p.is_empty()) {
        let (key, value) = pair.split_once('=').ok_or(SetError::BadValue)?;
        if key == "name" {
            let mut buf = [0u8; NAME_MAX_LEN];
            let name = decode(value, &mut buf).and_then(Name::new);
            s.presets[s.preset as usize].name = name.ok_or(SetError::BadName)?;
            continue;
        }
        let v: u32 = value.parse().map_err(|_| SetError::BadValue)?;
        // Out-of-range values saturate here and clamp below
        let v16 = v.min(u16::MAX as u32) as u16;
        match key {
            "preset" => s.preset = v.min(PRESET_COUNT as u32 - 1) as u8,
            "glide_min_us" => s.glide_min_us = v,
            "glide_max_us" => s.glide_max_us = v,
            "dwell_min_ms" => s.dwell_min_ms = v,
            "dwell_max_ms" => s.dwell_max_ms = v,
            "manual_speed" => s.manual_speed = v16,
            "active_min" => s.active_min = v16,
            "rest_min" => s.rest_min = v16,
            _ => return Err(SetError::UnknownKey),
        }
    }
    s.clamp(travel);
    *settings = s;
    Ok(())
}

/// Percent-decodes `src` into `buf` with `+` as space. `None` on a bad escape,
/// overflow, or invalid UTF-8
pub fn decode<'b>(src: &str, buf: &'b mut [u8]) -> Option<&'b str> {
    let mut bytes = src.bytes();
    let mut len = 0;
    while let Some(b) = bytes.next() {
        let out = match b {
            b'+' => b' ',
            b'%' => {
                let hi = (bytes.next()? as char).to_digit(16)?;
                let lo = (bytes.next()? as char).to_digit(16)?;
                (hi * 16 + lo) as u8
            }
            b => b,
        };
        *buf.get_mut(len)? = out;
        len += 1;
    }
    core::str::from_utf8(&buf[..len]).ok()
}

/// Writes the status JSON. Names need no escaping since `Name` excludes `"` and `\`
pub fn write_status(
    w: &mut impl Write,
    status: &Status,
    settings: &Settings,
    travel: Travel,
) -> core::fmt::Result {
    let mode = match status.mode {
        Mode::Random => "random",
        Mode::Manual => "manual",
        Mode::Off => "off",
    };
    let phase = match status.phase {
        Some(Phase::Active) => "\"active\"",
        Some(Phase::Rest) => "\"rest\"",
        None => "null",
    };
    write!(
        w,
        r#"{{"mode":"{mode}","phase":{phase},"phase_remaining_s":{},"laser":{},"x":{},"y":{},"travel":[{},{}],"preset":{},"names":["#,
        status.phase_remaining_s,
        status.laser,
        status.x,
        status.y,
        travel.min,
        travel.max,
        settings.preset,
    )?;
    for (i, p) in settings.presets.iter().enumerate() {
        let sep = if i == 0 { "" } else { "," };
        write!(w, r#"{sep}"{}""#, p.name.as_str())?;
    }
    w.write_str(r#"],"points":["#)?;
    for (i, p) in settings.outline().points().iter().enumerate() {
        let sep = if i == 0 { "" } else { "," };
        write!(w, "{sep}[{},{}]", p.x, p.y)?;
    }
    write!(
        w,
        r#"],"glide_min_us":{},"glide_max_us":{},"dwell_min_ms":{},"dwell_max_ms":{},"manual_speed":{},"active_min":{},"rest_min":{}}}"#,
        settings.glide_min_us,
        settings.glide_max_us,
        settings.dwell_min_ms,
        settings.dwell_max_ms,
        settings.manual_speed,
        settings.active_min,
        settings.rest_min,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::controller::Controller;
    use crate::geometry::Point;
    use crate::settings::{Outline, MAX_POINTS};

    const T: Travel = Travel {
        min: 491,
        max: 1966,
    };

    fn req(target: &str) -> String {
        format!("GET {target} HTTP/1.1\r\nHost: 192.168.2.1\r\n\r\n")
    }

    #[test]
    fn routes() {
        assert_eq!(route(&req("/")), Route::Page);
        assert_eq!(route(&req("/d?dir=up")), Route::Dpad(Dir::Up));
        assert_eq!(route(&req("/d?dir=stop")), Route::Dpad(Dir::Stop));
        assert_eq!(route(&req("/d?dir=sideways")), Route::BadRequest);
        assert_eq!(route(&req("/mode?m=off")), Route::Mode(Mode::Off));
        assert_eq!(route(&req("/mode?m=random")), Route::Mode(Mode::Random));
        assert_eq!(route(&req("/mode?m=x")), Route::BadRequest);
        assert_eq!(route(&req("/set?a=1")), Route::Set("a=1"));
        assert_eq!(route(&req("/status")), Route::Status);
        assert_eq!(route(&req("/nope")), Route::NotFound);
        assert_eq!(route("POST / HTTP/1.1\r\n\r\n"), Route::NotFound);
        assert_eq!(route(""), Route::NotFound);
    }

    fn apply(query: &str) -> (Result<(), SetError>, Settings) {
        let mut s = Settings::defaults(T);
        let r = apply_set(query, &mut s, T);
        (r, s)
    }

    #[test]
    fn set_applies_and_clamps() {
        let (r, s) = apply("manual_speed=9000&rest_min=0");
        assert_eq!(r, Ok(()));
        assert_eq!((s.manual_speed, s.rest_min), (2000, 0));
    }

    #[test]
    fn set_preset_then_edit() {
        let (r, s) = apply("preset=2&name=Bed%20room");
        assert_eq!(r, Ok(()));
        assert_eq!(s.preset, 2);
        assert_eq!(s.presets[2].name.as_str(), "Bed room");
        assert_eq!(s.presets[0], Settings::defaults(T).presets[0]);
    }

    #[test]
    fn set_rejects_rectangle_keys() {
        for q in ["x_min=900", "x_max=900", "y_min=900", "y_max=900"] {
            let (r, s) = apply(q);
            assert_eq!(r, Err(SetError::UnknownKey), "{q}");
            assert_eq!(s, Settings::defaults(T));
        }
    }

    #[test]
    fn set_plus_is_space() {
        let (r, s) = apply("name=Living+room");
        assert_eq!(r, Ok(()));
        assert_eq!(s.presets[0].name.as_str(), "Living room");
    }

    #[test]
    fn set_unknown_key_applies_nothing() {
        let (r, s) = apply("manual_speed=600&bogus=1");
        assert_eq!(r, Err(SetError::UnknownKey));
        assert_eq!(s, Settings::defaults(T));
    }

    #[test]
    fn set_overflow_is_bad_value() {
        for q in [
            "glide_max_us=99999999999",
            "active_min=-1",
            "rest_min=",
            "rest_min",
        ] {
            let (r, s) = apply(q);
            assert_eq!(r, Err(SetError::BadValue), "{q}");
            assert_eq!(s, Settings::defaults(T));
        }
    }

    #[test]
    fn set_bad_name() {
        for q in [
            "name=%22q%22",
            "name=abcdefghijklmnopq",
            "name=",
            "name=%zz",
        ] {
            let (r, s) = apply(q);
            assert_eq!(r, Err(SetError::BadName), "{q}");
            assert_eq!(s, Settings::defaults(T));
        }
    }

    #[test]
    fn decode_rejects_bad_escape() {
        let mut buf = [0u8; 16];
        assert_eq!(decode("%zz", &mut buf), None);
        assert_eq!(decode("%4", &mut buf), None);
        assert_eq!(decode("%FF", &mut buf), None);
        assert_eq!(decode("abcdefghijklmnopqrst", &mut buf), None);
        assert_eq!(decode("a%20b", &mut buf), Some("a b"));
    }

    fn status_string(status: &Status, s: &Settings) -> String {
        let mut out = String::new();
        write_status(&mut out, status, s, T).unwrap();
        out
    }

    fn pt(x: u16, y: u16) -> Point {
        Point { x, y }
    }

    #[test]
    fn status_json_exact() {
        let mut s = Settings::defaults(T);
        s.presets[0].outline =
            Outline::new(&[pt(900, 900), pt(1500, 900), pt(1500, 1500)]).unwrap();
        let status = Controller::new(0, T, &s, 1).status(0);
        let expected = r#"{"mode":"random","phase":"active","phase_remaining_s":600,"laser":true,"x":1228,"y":1228,"travel":[491,1966],"preset":0,"names":["Room 1","Room 2","Room 3","Room 4"],"points":[[900,900],[1500,900],[1500,1500]],"glide_min_us":1000,"glide_max_us":15000,"dwell_min_ms":900,"dwell_max_ms":2000,"manual_speed":450,"active_min":10,"rest_min":20}"#;
        assert_eq!(status_string(&status, &s), expected);
    }

    #[test]
    fn status_phase_null_outside_random() {
        let s = Settings::defaults(T);
        let mut c = Controller::new(0, T, &s, 1);
        c.set_mode(Mode::Off, 0, &s);
        let json = status_string(&c.status(0), &s);
        assert!(json.starts_with(r#"{"mode":"off","phase":null,"#), "{json}");
    }

    #[test]
    fn status_fits_buffer() {
        let mut s = Settings::defaults(T);
        for p in &mut s.presets {
            p.name = Name::new("MMMMMMMMMMMMMMMM").unwrap();
            p.outline = Outline::new(&[pt(u16::MAX, u16::MAX); MAX_POINTS]).unwrap();
        }
        s.glide_min_us = u32::MAX;
        s.glide_max_us = u32::MAX;
        s.dwell_min_ms = u32::MAX;
        s.dwell_max_ms = u32::MAX;
        s.manual_speed = u16::MAX;
        s.active_min = u16::MAX;
        s.rest_min = u16::MAX;
        s.preset = 3;
        let status = Status {
            mode: Mode::Manual,
            phase: Some(Phase::Active),
            phase_remaining_s: u32::MAX,
            laser: false,
            x: u16::MAX,
            y: u16::MAX,
        };
        let travel = Travel {
            min: u16::MAX,
            max: u16::MAX,
        };
        let mut out = String::new();
        write_status(&mut out, &status, &s, travel).unwrap();
        assert!(out.len() <= STATUS_BUF_LEN, "{}", out.len());
    }

    #[test]
    fn status_fits_buffer_at_travel_max() {
        let mut s = Settings::defaults(T);
        for p in &mut s.presets {
            p.name = Name::new("MMMMMMMMMMMMMMMM").unwrap();
            p.outline = Outline::new(&[pt(1966, 1966); MAX_POINTS]).unwrap();
        }
        let status = Controller::new(0, T, &s, 1).status(0);
        assert!(status_string(&status, &s).len() < STATUS_BUF_LEN);
    }
}
