//! HTTP request routing, query handling, shared dispatch, and status JSON.

use core::fmt::Write;

use crate::controller::{Controller, Dir, Idle, Mode, Phase, Status};
use crate::geometry::{self_crossing, Point};
use crate::settings::{Name, Outline, Settings, Travel, MAX_POINTS, NAME_MAX_LEN, PRESET_COUNT};

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
    Goto(Point),
    Preview,
    /// Raw query string, still percent-encoded
    Points(&'a str),
    NotFound,
    BadRequest,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SetError {
    UnknownKey,
    BadValue,
    BadName,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PointsError {
    BadValue,
    TooMany,
    Crossing,
    /// `r` is not the active room
    WrongRoom,
}

/// Controller, settings, and travel shared by the HTTP shell and the controller loop
pub struct State {
    pub controller: Controller,
    pub settings: Settings,
    pub travel: Travel,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reply {
    Page,
    NoContent,
    Status,
    BadRequest,
    Conflict,
    NotFound,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Outcome {
    pub reply: Reply,
    /// Step the controller now
    pub wake: bool,
    /// Schedule a flash save
    pub settings_changed: bool,
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
            "dir=upleft" => Route::Dpad(Dir::UpLeft),
            "dir=upright" => Route::Dpad(Dir::UpRight),
            "dir=downleft" => Route::Dpad(Dir::DownLeft),
            "dir=downright" => Route::Dpad(Dir::DownRight),
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
        "/goto" => parse_goto(query).map_or(Route::BadRequest, Route::Goto),
        "/preview" => Route::Preview,
        "/points" => Route::Points(query),
        _ => Route::NotFound,
    }
}

/// Parses `x=N&y=N` in either order
fn parse_goto(query: &str) -> Option<Point> {
    let (mut x, mut y) = (None, None);
    for pair in query.split('&') {
        let (key, value) = pair.split_once('=')?;
        let v = value.parse().ok()?;
        match key {
            "x" => x = Some(v),
            "y" => y = Some(v),
            _ => return None,
        }
    }
    Some(Point { x: x?, y: y? })
}

/// Applies the route to `state` and returns the reply and side effects for the shell
pub fn dispatch(route: Route<'_>, state: &mut State, now_us: u64) -> Outcome {
    let done = |reply| Outcome {
        reply,
        wake: false,
        settings_changed: false,
    };
    let act = Outcome {
        reply: Reply::NoContent,
        wake: true,
        settings_changed: false,
    };
    let saved = Outcome {
        reply: Reply::Status,
        wake: true,
        settings_changed: true,
    };
    let State {
        controller: c,
        settings,
        travel,
    } = state;
    let before = *settings.outline();
    let outcome = match route {
        Route::Page => done(Reply::Page),
        Route::Status => done(Reply::Status),
        Route::Dpad(dir) => {
            c.manual(dir, now_us);
            act
        }
        Route::Mode(mode) => {
            c.set_mode(mode, now_us, settings);
            act
        }
        Route::Goto(p) => {
            c.goto(p, now_us);
            act
        }
        Route::Preview => {
            c.preview(now_us, settings);
            act
        }
        Route::Set(query) => match apply_set(query, settings, *travel) {
            Ok(()) => saved,
            Err(_) => done(Reply::BadRequest),
        },
        Route::Points(query) => match apply_points(query, settings, *travel) {
            Ok(()) => saved,
            Err(PointsError::WrongRoom) => done(Reply::Conflict),
            Err(_) => done(Reply::BadRequest),
        },
        Route::BadRequest => done(Reply::BadRequest),
        Route::NotFound => done(Reply::NotFound),
    };
    if *settings.outline() != before {
        c.outline_changed(now_us);
    }
    outcome
}

/// Replaces the active room's outline from `r=N&p=x,y;x,y;...`, clamped to travel.
/// An empty `p` clears the room. Changes nothing on error
pub fn apply_points(
    query: &str,
    settings: &mut Settings,
    travel: Travel,
) -> Result<(), PointsError> {
    let (mut room, mut list) = (None, None);
    for pair in query.split('&') {
        match pair.split_once('=') {
            Some(("r", v)) => room = Some(v.parse::<u8>().map_err(|_| PointsError::BadValue)?),
            Some(("p", v)) => list = Some(v),
            _ => return Err(PointsError::BadValue),
        }
    }
    let (Some(room), Some(list)) = (room, list) else {
        return Err(PointsError::BadValue);
    };
    // 16 points of "65535,65535;"
    let mut buf = [0u8; 192];
    let list = decode(list, &mut buf).ok_or(PointsError::BadValue)?;
    let mut points = [Point { x: 0, y: 0 }; MAX_POINTS];
    let mut n = 0;
    for xy in list.split(';').filter(|s| !s.is_empty()) {
        let slot = points.get_mut(n).ok_or(PointsError::TooMany)?;
        let (x, y) = xy.split_once(',').ok_or(PointsError::BadValue)?;
        let parse = |v: &str| v.parse::<u16>().map_err(|_| PointsError::BadValue);
        *slot = Point {
            x: parse(x)?.clamp(travel.min, travel.max),
            y: parse(y)?.clamp(travel.min, travel.max),
        };
        n += 1;
    }
    if room != settings.preset {
        return Err(PointsError::WrongRoom);
    }
    if self_crossing(&points[..n]) {
        return Err(PointsError::Crossing);
    }
    settings.presets[room as usize].outline =
        Outline::new(&points[..n]).ok_or(PointsError::TooMany)?;
    Ok(())
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
    let idle = match status.idle {
        Some(Idle::NoArea) => "\"no_area\"",
        None => "null",
    };
    write!(
        w,
        r#"{{"mode":"{mode}","phase":{phase},"phase_remaining_s":{},"idle":{idle},"laser":{},"x":{},"y":{},"travel":[{},{}],"preset":{},"names":["#,
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
        let expected = r#"{"mode":"random","phase":"active","phase_remaining_s":600,"idle":null,"laser":true,"x":1228,"y":1228,"travel":[491,1966],"preset":0,"names":["Room 1","Room 2","Room 3","Room 4"],"points":[[900,900],[1500,900],[1500,1500]],"glide_min_us":1000,"glide_max_us":15000,"dwell_min_ms":900,"dwell_max_ms":2000,"manual_speed":450,"active_min":10,"rest_min":20}"#;
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
            idle: Some(crate::controller::Idle::NoArea),
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

    #[test]
    fn routes_new() {
        assert_eq!(route(&req("/d?dir=upleft")), Route::Dpad(Dir::UpLeft));
        assert_eq!(route(&req("/d?dir=upright")), Route::Dpad(Dir::UpRight));
        assert_eq!(route(&req("/d?dir=downleft")), Route::Dpad(Dir::DownLeft));
        assert_eq!(route(&req("/d?dir=downright")), Route::Dpad(Dir::DownRight));
        assert_eq!(
            route(&req("/goto?x=900&y=1000")),
            Route::Goto(pt(900, 1000))
        );
        assert_eq!(
            route(&req("/goto?y=1000&x=900")),
            Route::Goto(pt(900, 1000))
        );
        assert_eq!(route(&req("/goto?x=a")), Route::BadRequest);
        assert_eq!(route(&req("/goto?x=900")), Route::BadRequest);
        assert_eq!(route(&req("/goto?x=900&y=99999")), Route::BadRequest);
        assert_eq!(route(&req("/preview")), Route::Preview);
        assert_eq!(route(&req("/points?r=0&p=1,2")), Route::Points("r=0&p=1,2"));
    }

    fn points(query: &str, preset: u8) -> (Result<(), PointsError>, Settings) {
        let mut s = Settings::defaults(T);
        s.preset = preset;
        let r = apply_points(query, &mut s, T);
        (r, s)
    }

    #[test]
    fn points_replace_active_room() {
        let (r, s) = points("r=0&p=900,900;1500,900;1500,1500", 0);
        assert_eq!(r, Ok(()));
        assert_eq!(
            s.presets[0].outline.points(),
            &[pt(900, 900), pt(1500, 900), pt(1500, 1500)]
        );
        assert_eq!(s.presets[1..], Settings::defaults(T).presets[1..]);
    }

    #[test]
    fn points_clamp_and_decode() {
        let (r, s) = points("r=0&p=0%2C900%3B1500,900;1500,4000", 0);
        assert_eq!(r, Ok(()));
        assert_eq!(
            s.presets[0].outline.points(),
            &[pt(491, 900), pt(1500, 900), pt(1500, 1966)]
        );
    }

    #[test]
    fn points_wrong_room_conflict() {
        let (r, s) = points("r=0&p=900,900;1500,900;1500,1500", 1);
        assert_eq!(r, Err(PointsError::WrongRoom));
        let mut want = Settings::defaults(T);
        want.preset = 1;
        assert_eq!(s, want);
        let mut state = state();
        state.settings.preset = 1;
        let out = dispatch(
            Route::Points("r=0&p=900,900;1500,900;1500,1500"),
            &mut state,
            0,
        );
        assert_eq!(out.reply, Reply::Conflict);
        assert!(!out.settings_changed);
        assert_eq!(state.settings, want);
    }

    #[test]
    fn points_too_many() {
        let p: Vec<String> = (0..17).map(|i| format!("{},900", 600 + i)).collect();
        let (r, s) = points(&format!("r=0&p={}", p.join(";")), 0);
        assert_eq!(r, Err(PointsError::TooMany));
        assert_eq!(s, Settings::defaults(T));
    }

    #[test]
    fn points_crossing_refused() {
        let (r, s) = points("r=0&p=600,600;1000,1000;1000,600;600,1000", 0);
        assert_eq!(r, Err(PointsError::Crossing));
        assert_eq!(s, Settings::defaults(T));
    }

    #[test]
    fn points_bad_value() {
        for q in [
            "r=0&p=1,x",
            "r=0&p=1",
            "r=0&p=99999,1",
            "p=1,1",
            "r=0",
            "r=x&p=",
            "r=0&p=1,1&q=2",
        ] {
            let (r, s) = points(q, 0);
            assert_eq!(r, Err(PointsError::BadValue), "{q}");
            assert_eq!(s, Settings::defaults(T));
        }
    }

    #[test]
    fn points_empty_clears() {
        let mut s = Settings::defaults(T);
        s.presets[0].outline =
            Outline::new(&[pt(900, 900), pt(1500, 900), pt(1500, 1500)]).unwrap();
        assert_eq!(apply_points("r=0&p=", &mut s, T), Ok(()));
        assert_eq!(s.presets[0].outline.points().len(), 0);
    }

    fn state() -> State {
        let settings = Settings::defaults(T);
        State {
            controller: Controller::new(0, T, &settings, 1),
            settings,
            travel: T,
        }
    }

    #[test]
    fn dispatch_flags() {
        let flags = |route: Route<'_>| {
            let o = dispatch(route, &mut state(), 0);
            (o.reply, o.wake, o.settings_changed)
        };
        assert_eq!(flags(Route::Page), (Reply::Page, false, false));
        assert_eq!(flags(Route::Status), (Reply::Status, false, false));
        assert_eq!(flags(Route::Dpad(Dir::Up)), (Reply::NoContent, true, false));
        assert_eq!(
            flags(Route::Set("manual_speed=600")),
            (Reply::Status, true, true)
        );
        assert_eq!(
            flags(Route::Set("bogus=1")),
            (Reply::BadRequest, false, false)
        );
        assert_eq!(
            flags(Route::Points("r=0&p=900,900;1500,900;1500,1500")),
            (Reply::Status, true, true)
        );
        assert_eq!(
            flags(Route::Points("r=0&p=1")),
            (Reply::BadRequest, false, false)
        );
        assert_eq!(
            flags(Route::Goto(pt(900, 900))),
            (Reply::NoContent, true, false)
        );
        assert_eq!(flags(Route::Preview), (Reply::NoContent, true, false));
        assert_eq!(
            flags(Route::Mode(Mode::Off)),
            (Reply::NoContent, true, false)
        );
        assert_eq!(flags(Route::BadRequest), (Reply::BadRequest, false, false));
        assert_eq!(flags(Route::NotFound), (Reply::NotFound, false, false));
    }

    #[test]
    fn dispatch_applies_actions() {
        let mut st = state();
        dispatch(Route::Goto(pt(900, 1000)), &mut st, 0);
        assert_eq!(st.controller.target(), Some(pt(900, 1000)));
        dispatch(Route::Set("manual_speed=600"), &mut st, 0);
        assert_eq!(st.settings.manual_speed, 600);
    }

    #[test]
    fn status_idle_field() {
        let s = Settings::defaults(T);
        let json = status_string(&Controller::new(0, T, &s, 1).status(0), &s);
        assert!(
            json.contains(r#""phase_remaining_s":600,"idle":"no_area","#),
            "{json}"
        );
    }

    #[test]
    fn dispatch_room_switch_drops_old_target() {
        let mut st = state();
        st.settings.presets[0].outline =
            Outline::new(&[pt(900, 900), pt(1500, 900), pt(1500, 1500)]).unwrap();
        st.settings.presets[1].outline =
            Outline::new(&[pt(500, 500), pt(600, 500), pt(600, 600)]).unwrap();
        st.settings.dwell_min_ms = 0;
        st.settings.dwell_max_ms = 0;
        let mut now = 0;
        while st.controller.target().is_none() {
            now = st.controller.step(now, &st.settings).next_us.unwrap();
        }
        dispatch(Route::Set("preset=1"), &mut st, now);
        assert_eq!(st.controller.target(), None);
        // Unrelated settings keep the move going
        while st.controller.target().is_none() {
            now = st.controller.step(now, &st.settings).next_us.unwrap();
        }
        dispatch(Route::Set("manual_speed=600"), &mut st, now);
        assert!(st.controller.target().is_some());
    }
}
