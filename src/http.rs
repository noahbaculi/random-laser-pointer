//! Keep-alive HTTP server pool, API dispatch, and the control page.

use core::fmt::Write as _;

use embassy_net::tcp::TcpSocket;
use embassy_net::{IpListenEndpoint, Stack};
use embassy_time::Duration;
use embedded_io_async::Write;
use random_laser_pointer::api::{self, Reply, Route, STATUS_BUF_LEN};

use crate::control::{now_us, with_state, SETTINGS_CHANGED, WAKE};

pub const HTTP_POOL_SIZE: usize = 3;

const NO_CONTENT: &[u8] = b"HTTP/1.1 204 No Content\r\nContent-Length: 0\r\n\r\n";
const BAD_REQUEST: &[u8] = b"HTTP/1.1 400 Bad Request\r\nContent-Length: 0\r\n\r\n";
const CONFLICT: &[u8] = b"HTTP/1.1 409 Conflict\r\nContent-Length: 0\r\n\r\n";
const NOT_FOUND: &[u8] = b"HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\n\r\n";

// Keep-alive HTTP server: each pooled task holds one persistent
// connection, so commands avoid a TCP handshake per request and a
// stop never races a closing socket
#[embassy_executor::task(pool_size = HTTP_POOL_SIZE)]
pub async fn http_task(stack: Stack<'static>) {
    let mut rx_buffer = [0; 1536];
    let mut tx_buffer = [0; 4096];
    let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
    socket.set_timeout(Some(Duration::from_secs(10)));

    loop {
        if let Err(e) = socket
            .accept(IpListenEndpoint {
                addr: None,
                port: 80,
            })
            .await
        {
            log::warn!("accept error: {:?}", e);
            socket.abort();
            _ = socket.flush().await;
            continue;
        }

        'connection: loop {
            let mut buffer = [0u8; 1024];
            let mut pos = 0;
            loop {
                match socket.read(&mut buffer[pos..]).await {
                    Ok(0) => break 'connection,
                    Ok(len) => {
                        pos += len;
                        if buffer[..pos].windows(4).any(|w| w == b"\r\n\r\n") {
                            break;
                        }
                        if pos == buffer.len() {
                            break 'connection;
                        }
                    }
                    Err(_) => break 'connection,
                }
            }

            let ok = match core::str::from_utf8(&buffer[..pos]) {
                Ok(request) => respond(&mut socket, api::route(request)).await,
                Err(_) => socket.write_all(BAD_REQUEST).await.is_ok(),
            };
            if !ok || socket.flush().await.is_err() {
                break;
            }
        }

        socket.close();
        _ = socket.flush().await;
        socket.abort();
    }
}

async fn respond(socket: &mut TcpSocket<'_>, route: Route<'_>) -> bool {
    let Some(out) = with_state(|s| api::dispatch(route, s, now_us())) else {
        return socket.write_all(BAD_REQUEST).await.is_ok();
    };
    if !matches!(route, Route::Status | Route::Page | Route::Dpad(_)) {
        log::info!("{route:?} -> {:?}", out.reply);
    }
    if out.wake {
        WAKE.signal(());
    }
    if out.settings_changed {
        SETTINGS_CHANGED.signal(());
    }
    let reply: &[u8] = match out.reply {
        Reply::Page => return send(socket, "text/html", PAGE_BODY.as_bytes()).await,
        Reply::Status => return send_status(socket).await,
        Reply::NoContent => NO_CONTENT,
        Reply::BadRequest => BAD_REQUEST,
        Reply::Conflict => CONFLICT,
        Reply::NotFound => NOT_FOUND,
    };
    socket.write_all(reply).await.is_ok()
}

async fn send_status(socket: &mut TcpSocket<'_>) -> bool {
    // Copy out so formatting runs outside the critical section
    let Some((status, settings, travel)) =
        with_state(|s| (s.controller.status(now_us()), s.settings, s.travel))
    else {
        return socket.write_all(BAD_REQUEST).await.is_ok();
    };
    let mut body = Buf::<STATUS_BUF_LEN>::new();
    if api::write_status(&mut body, &status, &settings, travel).is_err() {
        return false;
    }
    send(socket, "application/json", body.as_bytes()).await
}

async fn send(socket: &mut TcpSocket<'_>, content_type: &str, body: &[u8]) -> bool {
    let mut header = Buf::<128>::new();
    if write!(
        header,
        "HTTP/1.1 200 OK\r\nContent-Type: {content_type}\r\nContent-Length: {}\r\n\r\n",
        body.len()
    )
    .is_err()
    {
        return false;
    }
    socket.write_all(header.as_bytes()).await.is_ok() && socket.write_all(body).await.is_ok()
}

/// Fixed-capacity `core::fmt::Write` sink
struct Buf<const N: usize> {
    buf: [u8; N],
    pos: usize,
}

impl<const N: usize> Buf<N> {
    fn new() -> Self {
        Buf {
            buf: [0; N],
            pos: 0,
        }
    }

    fn as_bytes(&self) -> &[u8] {
        &self.buf[..self.pos]
    }
}

impl<const N: usize> core::fmt::Write for Buf<N> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        let end = self.pos + bytes.len();
        if end > N {
            return Err(core::fmt::Error);
        }
        self.buf[self.pos..end].copy_from_slice(bytes);
        self.pos = end;
        Ok(())
    }
}

const PAGE_BODY: &str = include_str!("page.html");
