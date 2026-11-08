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

const PAGE_BODY: &str = r##"<!DOCTYPE html><html><head>
<meta name=viewport content="width=device-width,initial-scale=1,user-scalable=no">
<title>Laser</title><style>
body{margin:0;font-family:system-ui;background:#111;color:#eee;display:flex;flex-direction:column;align-items:center;justify-content:center;min-height:100vh;touch-action:none;-webkit-user-select:none;user-select:none}
#pad{display:grid;grid-template-columns:repeat(3,88px);grid-template-rows:repeat(3,88px);gap:8px}
button{font-size:34px;border:0;border-radius:14px;background:#333;color:#eee;touch-action:none}
button:active{background:#0a84ff}
#rtt{display:flex;align-items:center;justify-content:center;font-size:14px;color:#888;white-space:pre;text-align:center}
#st{color:#888}
#modes{display:flex;gap:8px;margin-bottom:16px}
#modes button{font-size:18px;padding:12px 16px}
#modes button.on{background:#0a84ff}
</style></head><body>
<h3>Laser</h3>
<div id=modes><button data-m=random>Random</button><button data-m=manual>Manual</button><button data-m=off>Off</button></div>
<div id=pad>
<i></i><button data-d=up>&#9650;</button><i></i>
<button data-d=left>&#9664;</button><div id=rtt>--</div><button data-d=right>&#9654;</button>
<i></i><button data-d=down>&#9660;</button><i></i>
</div>
<p id=st>released</p>
<script>
let held=null,timer=null,busy=false,pend=null,rtts=[];
function show(r){rtts.push(r);if(rtts.length>30)rtts.shift();
let a=rtts.reduce((x,y)=>x+y,0)/rtts.length;
document.getElementById('rtt').textContent=r.toFixed(0)+' ms\navg '+a.toFixed(0);}
function send(d){if(busy){pend=d;return}busy=true;let t=performance.now();
fetch('/d?dir='+d).then(()=>show(performance.now()-t)).catch(()=>{if(d==='stop'&&!held)setTimeout(()=>send('stop'),100)}).finally(()=>{busy=false;if(pend!==null){let p=pend;pend=null;send(p)}});}
function dn(e,d){e.preventDefault();held=d;document.getElementById('st').textContent=d;send(d);
if(timer)clearInterval(timer);timer=setInterval(()=>{if(held)send(held)},120);}
function up(e){e.preventDefault();if(!held)return;held=null;if(timer)clearInterval(timer);timer=null;
document.getElementById('st').textContent='released';pend=null;send('stop');}
function mark(m){for(const b of document.querySelectorAll('#modes button'))b.classList.toggle('on',b.dataset.m===m)}
function poll(){fetch('/status').then(r=>r.json()).then(s=>mark(s.mode)).catch(()=>{})}
for(const b of document.querySelectorAll('#modes button'))b.addEventListener('click',()=>{mark(b.dataset.m);fetch('/mode?m='+b.dataset.m).finally(poll)});
poll();setInterval(poll,2000);
for(const b of document.querySelectorAll('#pad button')){
b.addEventListener('pointerdown',e=>dn(e,b.dataset.d));
b.addEventListener('pointerup',up);b.addEventListener('pointercancel',up);b.addEventListener('pointerleave',up);}
</script></body></html>"##;
