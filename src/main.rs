#![no_std]
#![no_main]
#![allow(async_fn_in_trait)]

mod setup_pio_1;
mod tasks;

// use core::str::from_utf8;
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::peripherals::{PIO0, PIO1};
use embassy_rp::pio::Pio;
use {defmt_rtt as _, panic_probe as _};

use crate::setup_pio_1::setup_pio_task_sm0;
use crate::tasks::{cyw43_task, net_task, pio_task_sm0, uart_tx_task};
use cyw43::JoinOptions;
use cyw43_pio::{DEFAULT_CLOCK_DIVIDER, PioSpi};
use embassy_net::StackResources;
use embassy_net::tcp::TcpSocket;
use embassy_rp::clocks::RoscRng;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::uart::{Async, UartTx};
use embassy_rp::{bind_interrupts, uart};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer, with_timeout};
use embedded_io_async::Write;
use static_cell::StaticCell;

#[defmt::panic_handler]
fn panic() -> ! {
    panic_probe::hard_fault();
}

static SIGNAL: Signal<ThreadModeRawMutex, ()> = Signal::new();
static UART_READY: Signal<ThreadModeRawMutex, ()> = Signal::new();
static INPUT: Signal<ThreadModeRawMutex, u8> = Signal::new();
static ECHO: Signal<ThreadModeRawMutex, u8> = Signal::new();

static START_SEQ: [u8; 4] = [0xA1, 0x00, 0xA2, 0x00];
static STOP_SEQ: [u8; 4] = [0xA3, 0x00, 0xA0, 0x00];

bind_interrupts!(struct Irqs {
    PIO1_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO1>;
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("spawner started");
    let p = embassy_rp::init(embassy_rp::config::Config::default());

    // Create UART writer
    let mut uart_config = uart::Config::default();
    uart_config.baudrate = 4800;
    let uart_tx: UartTx<'_, Async> = UartTx::new(p.UART1, p.PIN_4, p.DMA_CH1, uart_config);

    // PIO machinery
    let Pio {
        mut common,
        mut sm0,
        ..
    } = Pio::new(p.PIO1, Irqs);

    setup_pio_task_sm0(&mut common, &mut sm0);

    // WiFi

    let fw = include_bytes!("../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../cyw43-firmware/43439A0_clm.bin");
    // let nvram = include_bytes!("../cyw43-firmware/nvram_rp2040.bin");

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        DEFAULT_CLOCK_DIVIDER,
        pio.irq0,
        cs,
        p.PIN_24,
        p.PIN_29,
        p.DMA_CH0,
    );

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;

    // let _ = spawner.spawn(unwrap!(cyw43_task(runner)));

    let _ = spawner.spawn(cyw43_task(runner));

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    let config = embassy_net::Config::dhcpv4(Default::default());

    // Generate random seed
    let mut rng = RoscRng;
    let seed = rng.next_u64();

    // Init network stack
    static RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(
        net_device,
        config,
        RESOURCES.init(StackResources::new()),
        seed,
    );

    let _ = spawner.spawn(net_task(runner));

    let wifi_net: &str = str::from_utf8(include_bytes!("../wifi_net")).unwrap();
    let wifi_pass = include_bytes!("../wifi_pass");

    while let Err(_err) = control.join(wifi_net, JoinOptions::new(wifi_pass)).await {
        info!("join wifi network failed");
    }

    info!("waiting for link...");
    stack.wait_link_up().await;

    info!("waiting for DHCP...");
    stack.wait_config_up().await;

    // And now we can use it!
    info!("Stack is up!");

    // Start PIO
    let _ = spawner.spawn(pio_task_sm0(sm0));

    // Start UART
    let _ = spawner.spawn(uart_tx_task(uart_tx));

    let mut rx_buffer = [0; 1];
    let mut tx_buffer = [0; 1];
    let mut buf = [0; 1];

    let _ = UART_READY.wait().await;
    info!("UART is ready...");

    Timer::after(Duration::from_secs(5)).await;
    transmit_bytes(&START_SEQ).await;

    Timer::after(Duration::from_secs(5)).await;
    transmit_bytes(&STOP_SEQ).await;

    Timer::after(Duration::from_secs(2)).await;

    info!("SECOND ROUND...");

    Timer::after(Duration::from_secs(5)).await;
    transmit_bytes(&START_SEQ).await;

    Timer::after(Duration::from_secs(5)).await;
    transmit_bytes(&STOP_SEQ).await;

    Timer::after(Duration::from_secs(2)).await;

    loop {
        // reset signals
        // SIGNAL.reset();
        // INPUT.reset();
        // ECHO.reset();

        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(120)));

        control.gpio_set(0, false).await;
        info!("Listening on TCP:1234...");
        if let Err(e) = socket.accept(1234).await {
            warn!("accept error: {:?}", e);
            continue;
        }

        info!("Received connection from {:?}", socket.remote_endpoint());
        control.gpio_set(0, true).await;

        loop {
            let _n = match socket.read(&mut buf).await {
                Ok(0) => {
                    warn!("read EOF");
                    break;
                }
                Ok(n) => n,
                Err(e) => {
                    warn!("read error: {:?}", e);
                    break;
                }
            };

            info!("recv: {:02x}", buf[0]);

            // push the received byte it into INPUT
            INPUT.signal(buf[0]);

            // wait for ECHO upt to 2 sec
            if let Ok(echo) = with_timeout(Duration::from_secs(2), ECHO.wait()).await {
                info!("echo: {:02x}", echo);
                match socket.write_all(&[echo]).await {
                    Ok(()) => {
                        // can accept a new byte from input
                    }
                    Err(e) => {
                        warn!("write error: {:?}", e);
                        break;
                    }
                };
            } else {
                warn!("echo has not arrived, reconnecting...");
                socket.abort();
                let _ = socket.flush().await;
                break;
            };
        }
    }
}

async fn transmit_bytes(seq: &[u8]) {
    for (i, b) in seq.iter().enumerate() {
        info!("sending byte {:02x} number {}", b, i);
        INPUT.signal(*b);
        let echo = ECHO.wait().await;
        info!("echo {:02x} received", echo);
        let delay = if (i + 1).is_multiple_of(2) { 50 } else { 20 };
        info!("delay {}", delay);
        Timer::after(Duration::from_millis(delay)).await;
    }
}
