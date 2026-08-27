//! WiFi AP lifecycle, network stack runner, and DHCP server.

use core::net::Ipv4Addr;

use embassy_net::{Runner, Stack};
use embassy_time::{Duration, Timer};
use esp_radio::wifi::{Interface, WifiController};

use super::GATEWAY_IP;

#[embassy_executor::task]
pub(super) async fn net_task(mut runner: Runner<'static, Interface>) {
    runner.run().await
}

#[embassy_executor::task]
pub(super) async fn run_dhcp(stack: Stack<'static>) {
    use core::net::SocketAddrV4;

    use edge_dhcp::io::{self, DEFAULT_SERVER_PORT};
    use edge_dhcp::server::{Server, ServerOptions};
    use edge_nal::UdpBind;
    use edge_nal_embassy::{Udp, UdpBuffers};

    let mut buf = [0u8; 1500];
    let mut gw_buf = [Ipv4Addr::UNSPECIFIED];

    let buffers = UdpBuffers::<3, 1024, 1024, 10>::new();
    let unbound_socket = Udp::new(stack, &buffers);
    let mut bound_socket = unbound_socket
        .bind(core::net::SocketAddr::V4(SocketAddrV4::new(
            Ipv4Addr::UNSPECIFIED,
            DEFAULT_SERVER_PORT,
        )))
        .await
        .unwrap();

    loop {
        _ = io::server::run(
            &mut Server::<_, 64>::new_with_et(GATEWAY_IP),
            &ServerOptions::new(GATEWAY_IP, Some(&mut gw_buf)),
            &mut bound_socket,
            &mut buf,
        )
        .await
        .inspect_err(|e| log::warn!("DHCP server error: {e:?}"));
        Timer::after(Duration::from_millis(500)).await;
    }
}

#[embassy_executor::task]
pub(super) async fn connection(controller: WifiController<'static>) {
    loop {
        let ev = controller
            .wait_for_access_point_connected_event_async()
            .await;
        match ev {
            Ok(esp_radio::wifi::ap::EventInfo::Connected(info)) => {
                log::info!("Station connected: {:?}", info);
            }
            Ok(esp_radio::wifi::ap::EventInfo::Disconnected(info)) => {
                log::info!("Station disconnected: {:?}", info);
            }
            _ => (),
        }
        Timer::after(Duration::from_millis(1000)).await
    }
}
