//! Open WiFi access point, network stack runner, and DHCP server.

use core::net::Ipv4Addr;

use embassy_executor::Spawner;
use embassy_net::{Ipv4Cidr, Runner, Stack, StackResources, StaticConfigV4};
use embassy_time::{Duration, Timer};
use esp_hal::peripherals::WIFI;
use esp_radio::wifi::ap::AccessPointConfig;
use esp_radio::wifi::{Config, ControllerConfig, Interface, WifiController};

use crate::mk_static;

pub const AP_SSID: &str = "laser-dpad";
pub const GATEWAY_IP: Ipv4Addr = Ipv4Addr::new(192, 168, 2, 1);

/// Brings up the access point, network stack, and DHCP server, then waits for the link
pub async fn start(spawner: Spawner, wifi: WIFI<'static>, seed: u64) -> Stack<'static> {
    let ap_config =
        Config::AccessPoint(AccessPointConfig::default().with_ssid(AP_SSID.try_into().unwrap()));
    let device = Interface::access_point();
    let controller = WifiController::new(
        wifi,
        ControllerConfig::default().with_initial_config(ap_config),
    )
    .unwrap();
    log::info!("WiFi AP starting");

    let net_config = embassy_net::Config::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(GATEWAY_IP, 24),
        gateway: Some(GATEWAY_IP),
        dns_servers: Default::default(),
    });

    let (stack, runner) = embassy_net::new(
        device,
        net_config,
        // 3 http TCP sockets + DHCP UDP + headroom
        mk_static!(StackResources<8>, StackResources::<8>::new()),
        seed,
    );

    spawner.spawn(connection(controller).unwrap());
    spawner.spawn(net_task(runner).unwrap());
    spawner.spawn(run_dhcp(stack).unwrap());

    stack.wait_config_up().await;
    log::info!("Join open network `{AP_SSID}`, then open http://{GATEWAY_IP}/");
    stack
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, Interface>) {
    runner.run().await
}

#[embassy_executor::task]
async fn run_dhcp(stack: Stack<'static>) {
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
async fn connection(controller: WifiController<'static>) {
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
