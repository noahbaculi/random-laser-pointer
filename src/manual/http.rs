//! Keep-alive HTTP server pool and the D-pad page.

use embassy_net::tcp::TcpSocket;
use embassy_net::{IpListenEndpoint, Stack};
use embassy_time::Duration;
use embedded_io_async::Write;

use super::{set_command, Dir};

pub(super) const HTTP_POOL_SIZE: usize = 3;

// Keep-alive HTTP server: each pooled task holds one persistent
// connection, so commands avoid a TCP handshake per request and a
// stop never races a closing socket
#[embassy_executor::task(pool_size = HTTP_POOL_SIZE)]
pub(super) async fn http_task(stack: Stack<'static>) {
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

            let request = unsafe { core::str::from_utf8_unchecked(&buffer[..pos]) };
            let ok = match parse_path(request) {
                Some("/") => send_page(&mut socket).await,
                Some(path) if path.starts_with("/d?dir=") => {
                    match &path[7..] {
                        "up" => set_command(Dir::Up),
                        "down" => set_command(Dir::Down),
                        "left" => set_command(Dir::Left),
                        "right" => set_command(Dir::Right),
                        "stop" => set_command(Dir::Stop),
                        other => log::warn!("Unknown dir: {other}"),
                    }
                    socket
                        .write_all(b"HTTP/1.1 204 No Content\r\nContent-Length: 0\r\n\r\n")
                        .await
                        .is_ok()
                }
                _ => socket
                    .write_all(b"HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\n\r\n")
                    .await
                    .is_ok(),
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

async fn send_page(socket: &mut TcpSocket<'_>) -> bool {
    use core::fmt::Write as _;
    let mut header = HeaderBuf {
        buf: [0u8; 128],
        pos: 0,
    };
    if write!(
        header,
        "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length: {}\r\n\r\n",
        PAGE_BODY.len()
    )
    .is_err()
    {
        return false;
    }
    socket.write_all(&header.buf[..header.pos]).await.is_ok()
        && socket.write_all(PAGE_BODY.as_bytes()).await.is_ok()
}

struct HeaderBuf {
    buf: [u8; 128],
    pos: usize,
}

impl core::fmt::Write for HeaderBuf {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        if self.pos + bytes.len() > self.buf.len() {
            return Err(core::fmt::Error);
        }
        self.buf[self.pos..self.pos + bytes.len()].copy_from_slice(bytes);
        self.pos += bytes.len();
        Ok(())
    }
}

fn parse_path(request: &str) -> Option<&str> {
    let first_line = request.lines().next()?;
    let mut parts = first_line.split(' ');
    match parts.next()? {
        "GET" => parts.next(),
        _ => None,
    }
}

const PAGE_BODY: &str = r##"<!DOCTYPE html><html><head>
<meta name=viewport content="width=device-width,initial-scale=1,user-scalable=no">
<title>Laser D-pad</title><style>
body{margin:0;font-family:system-ui;background:#111;color:#eee;display:flex;flex-direction:column;align-items:center;justify-content:center;min-height:100vh;touch-action:none;-webkit-user-select:none;user-select:none}
#pad{display:grid;grid-template-columns:repeat(3,88px);grid-template-rows:repeat(3,88px);gap:8px}
button{font-size:34px;border:0;border-radius:14px;background:#333;color:#eee;touch-action:none}
button:active{background:#0a84ff}
#rtt{display:flex;align-items:center;justify-content:center;font-size:14px;color:#888;white-space:pre;text-align:center}
#st{color:#888}
</style></head><body>
<h3>Laser D-pad</h3>
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
for(const b of document.querySelectorAll('button')){
b.addEventListener('pointerdown',e=>dn(e,b.dataset.d));
b.addEventListener('pointerup',up);b.addEventListener('pointercancel',up);b.addEventListener('pointerleave',up);}
</script></body></html>"##;
