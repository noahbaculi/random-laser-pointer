//! Keep-alive HTTP server pool over the shared request handler.

use alloc::string::String;

use embassy_net::tcp::TcpSocket;
use embassy_net::{IpListenEndpoint, Stack};
use embassy_time::Duration;
use embedded_io_async::Write;
use random_laser_pointer::api;

use crate::control::{now_us, with_state, SETTINGS_CHANGED, WAKE};

pub const HTTP_POOL_SIZE: usize = 3;

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

            let ok = respond(&mut socket, &buffer[..pos]).await;
            if !ok || socket.flush().await.is_err() {
                break;
            }
        }

        socket.close();
        _ = socket.flush().await;
        socket.abort();
    }
}

/// Writes the reply and returns whether to keep the connection open
async fn respond(socket: &mut TcpSocket<'_>, request: &[u8]) -> bool {
    let Some(response) = with_state(|s| api::handle(request, s, now_us())) else {
        return false;
    };
    if let Some((route, reply)) = response.log {
        log::info!("{route:?} -> {reply:?}");
    }
    if response.wake {
        WAKE.signal(());
    }
    if response.save {
        SETTINGS_CHANGED.signal(());
    }
    let (mut head, mut buf) = (String::new(), String::new());
    let Ok(body) = response.render(&mut head, &mut buf) else {
        return false;
    };
    socket.write_all(head.as_bytes()).await.is_ok()
        && socket.write_all(body.as_bytes()).await.is_ok()
        && !response.close
}
