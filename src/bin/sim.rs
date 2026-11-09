//! Host simulator: the real controller and API over `std` sockets, plus a room view.
//!
//! Run with `cargo run --target aarch64-apple-darwin --no-default-features --features sim --bin sim`

// Empty on the firmware target, where `--all-features` clippy still checks this bin
#![cfg_attr(target_os = "none", no_std, no_main)]

#[cfg(target_os = "none")]
use esp_backtrace as _;

#[cfg(not(target_os = "none"))]
fn main() {
    host::run();
}

#[cfg(not(target_os = "none"))]
mod host {
    use std::fs;
    use std::io::{Read, Write};
    use std::net::{TcpListener, TcpStream, UdpSocket};
    use std::sync::{Arc, Condvar, Mutex, MutexGuard};
    use std::thread;
    use std::time::{Duration, Instant, SystemTime};

    use random_laser_pointer::api::{self, State};
    use random_laser_pointer::controller::Controller;
    use random_laser_pointer::record::{self, RECORD_LEN};
    use random_laser_pointer::settings::{Settings, Travel};

    const TRAVEL: Travel = Travel {
        min: 491,
        max: 1966,
    };
    const PORT: u16 = 8080;
    const SETTINGS_PATH: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/target/sim-settings.bin");
    const SIM_PAGE: &str = include_str!("../sim.html");

    /// Shared state plus the controller wake flag, mirroring the firmware's `STATE` and `WAKE`
    struct Sim {
        state: State,
        woken: bool,
    }

    struct Shared {
        sim: Mutex<Sim>,
        wake: Condvar,
        start: Instant,
    }

    impl Shared {
        fn now_us(&self) -> u64 {
            self.start.elapsed().as_micros() as u64
        }

        fn lock(&self) -> MutexGuard<'_, Sim> {
            self.sim.lock().unwrap_or_else(|e| e.into_inner())
        }
    }

    pub fn run() {
        let settings = load();
        let seed = SystemTime::now()
            .duration_since(SystemTime::UNIX_EPOCH)
            .map_or(1, |d| d.as_nanos() as u64);
        let shared = Arc::new(Shared {
            sim: Mutex::new(Sim {
                state: State {
                    controller: Controller::new(0, TRAVEL, &settings, seed),
                    settings,
                    travel: TRAVEL,
                },
                woken: false,
            }),
            wake: Condvar::new(),
            start: Instant::now(),
        });

        let listener = match TcpListener::bind(("0.0.0.0", PORT)) {
            Ok(l) => l,
            Err(e) => {
                eprintln!("bind port {PORT}: {e}");
                std::process::exit(1);
            }
        };
        match lan_ip() {
            Some(ip) => println!("phone: http://{ip}:{PORT}/"),
            None => println!("phone: no LAN address found"),
        }
        println!("room view: http://localhost:{PORT}/sim");

        let s = Arc::clone(&shared);
        thread::spawn(move || controller_loop(&s));
        for stream in listener.incoming().flatten() {
            let s = Arc::clone(&shared);
            thread::spawn(move || serve(stream, &s));
        }
    }

    /// Returns saved settings, or defaults when the file is missing or invalid
    fn load() -> Settings {
        fs::read(SETTINGS_PATH)
            .ok()
            .and_then(|b| <[u8; RECORD_LEN]>::try_from(b).ok())
            .and_then(|b| record::decode(&b, TRAVEL))
            .unwrap_or_else(Settings::defaults)
    }

    fn save(settings: &Settings) {
        if let Err(e) = fs::write(SETTINGS_PATH, record::encode(settings)) {
            eprintln!("save {SETTINGS_PATH}: {e}");
        }
    }

    /// Returns the address the default route uses. Connecting UDP sends no packets
    fn lan_ip() -> Option<std::net::IpAddr> {
        let socket = UdpSocket::bind("0.0.0.0:0").ok()?;
        socket.connect("8.8.8.8:80").ok()?;
        Some(socket.local_addr().ok()?.ip())
    }

    fn controller_loop(shared: &Shared) {
        let mut sim = shared.lock();
        loop {
            let now = shared.now_us();
            let State {
                controller,
                settings,
                ..
            } = &mut sim.state;
            let out = controller.step(now, settings);
            sim.woken = false;
            sim = match out.next_us {
                Some(at) => {
                    let wait = Duration::from_micros(at.saturating_sub(shared.now_us()));
                    let woken = |s: &mut Sim| !s.woken;
                    match shared.wake.wait_timeout_while(sim, wait, woken) {
                        Ok((g, _)) => g,
                        Err(e) => e.into_inner().0,
                    }
                }
                None => match shared.wake.wait_while(sim, |s| !s.woken) {
                    Ok(g) => g,
                    Err(e) => e.into_inner(),
                },
            };
        }
    }

    /// Serves one keep-alive connection until the client closes it
    fn serve(mut stream: TcpStream, shared: &Shared) {
        let _ = stream.set_read_timeout(Some(Duration::from_secs(10)));
        let mut buffer = [0u8; 1024];
        loop {
            let mut pos = 0;
            loop {
                match stream.read(&mut buffer[pos..]) {
                    Ok(0) | Err(_) => return,
                    Ok(len) => {
                        pos += len;
                        if buffer[..pos].windows(4).any(|w| w == b"\r\n\r\n") {
                            break;
                        }
                        if pos == buffer.len() {
                            return;
                        }
                    }
                }
            }
            let ok = respond(&mut stream, &buffer[..pos], shared);
            if !ok {
                return;
            }
        }
    }

    /// Writes the reply and returns whether to keep the connection open
    fn respond(stream: &mut TcpStream, request: &[u8], shared: &Shared) -> bool {
        if request.starts_with(b"GET /sim ") {
            return send(stream, "200 OK", "text/html", SIM_PAGE);
        }
        if request.starts_with(b"GET /sim/status ") {
            return sim_status(stream, shared);
        }
        let (response, saved) = {
            let mut sim = shared.lock();
            let response = api::handle(request, &mut sim.state, shared.now_us());
            if response.wake {
                sim.woken = true;
                shared.wake.notify_one();
            }
            let saved = response.save.then_some(sim.state.settings);
            (response, saved)
        };
        if let Some(settings) = saved {
            save(&settings);
        }
        if let Some((route, reply)) = response.log {
            println!("{route:?} -> {reply:?}");
        }
        let (mut head, mut buf) = (String::new(), String::new());
        let Ok(body) = response.render(&mut head, &mut buf) else {
            return false;
        };
        stream.write_all(head.as_bytes()).is_ok()
            && stream.write_all(body.as_bytes()).is_ok()
            && !response.close
    }

    /// Writes the status JSON with the current target appended
    fn sim_status(stream: &mut TcpStream, shared: &Shared) -> bool {
        let (response, target) = {
            let mut sim = shared.lock();
            let response = api::handle(
                b"GET /status HTTP/1.1\r\n\r\n",
                &mut sim.state,
                shared.now_us(),
            );
            (response, sim.state.controller.target())
        };
        let (mut head, mut body) = (String::new(), String::new());
        if response.render(&mut head, &mut body).is_err() {
            return false;
        }
        let target = match target {
            Some(p) => format!("[{},{}]", p.x, p.y),
            None => "null".into(),
        };
        body.pop();
        body.push_str(&format!(r#","target":{target}}}"#));
        send(stream, "200 OK", "application/json", &body)
    }

    fn send(stream: &mut TcpStream, code: &str, content_type: &str, body: &str) -> bool {
        let head = format!(
            "HTTP/1.1 {code}\r\nContent-Length: {}\r\nContent-Type: {content_type}\r\nCache-Control: no-store\r\n\r\n",
            body.len()
        );
        stream.write_all(head.as_bytes()).is_ok() && stream.write_all(body.as_bytes()).is_ok()
    }
}
