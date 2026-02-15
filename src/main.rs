#![no_std]
#![no_main]
#![allow(async_fn_in_trait)]

// use core::str::from_utf8;
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::peripherals::{DMA_CH0, PIO0, PIO1};
use embassy_rp::pio::program::pio_asm;
use embassy_rp::pio::{Common, Config as PioConf, Pio, ShiftConfig, ShiftDirection, StateMachine};
use {defmt_rtt as _, panic_probe as _};

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
use embassy_time::Duration;
use embedded_io_async::Write;
use fixed::types::extra::U8;
use static_cell::StaticCell;

#[defmt::panic_handler]
fn panic() -> ! {
    panic_probe::hard_fault();
}

static SIGNAL: Signal<ThreadModeRawMutex, ()> = Signal::new();
static INPUT: Signal<ThreadModeRawMutex, u8> = Signal::new();
static ECHO: Signal<ThreadModeRawMutex, u8> = Signal::new();

bind_interrupts!(struct Irqs {
    PIO1_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO1>;
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
});

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    runner.run().await
}

fn setup_pio_task_sm0<'d>(pio: &mut Common<'d, PIO1>, sm: &mut StateMachine<'d, PIO1, 0>) {
    let prg = pio_asm!(
        ".wrap_target"
        "  wait 0 pin 0"
        "  wait 1 pin 0"
        "  wait 0 pin 3"
        "  wait 1 pin 3"
        "  wait 0 pin 3"
        "  in null, 1",
        ".wrap"
    );
    let mut cfg = PioConf::default();
    cfg.use_program(&pio.load_program(&prg.program), &[]);

    cfg.shift_in = ShiftConfig {
        auto_fill: true,
        direction: ShiftDirection::Left,
        threshold: 1,
    };

    cfg.clock_divider = fixed::FixedU32::<U8>::from_num(10);
    sm.set_config(&cfg);
    sm.set_enable(true);
}

// signals when the "confirmation pulse" from typewriter is received
#[embassy_executor::task]
async fn pio_task_sm0(mut sm: StateMachine<'static, PIO1, 0>) -> ! {
    loop {
        let _ = sm.rx().wait_pull().await;
        SIGNAL.signal(());
    }
}

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("spawner started");
    let p = embassy_rp::init(embassy_rp::config::Config::default());

    // Create UART writer
    let mut uart_config = uart::Config::default();
    uart_config.baudrate = 4800;
    let uart_tx: UartTx<'_, Async> = UartTx::new(p.UART0, p.PIN_0, p.DMA_CH1, uart_config);

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

    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(5)));

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

            // info!("Received: {}", from_utf8(&buf[..n]).unwrap());

            // push the received byte it into INPUT
            INPUT.signal(buf[0]);

            // wait for ECHO
            let echo = ECHO.wait().await;

            match socket.write_all(&[echo]).await {
                Ok(()) => {
                    // can accept a new byte from input
                }
                Err(e) => {
                    warn!("write error: {:?}", e);
                    break;
                }
            };
        }
    }
}

#[embassy_executor::task]
async fn uart_tx_task(tx: UartTx<'static, Async>) -> ! {
    let uart_future = uart_writer_task(tx);
    uart_future.await;
}

// these steps are done:
// - receive a byte from INPUT
// - transmit the byte to UART
// - wait for SIGNAL
// - produce ECHO
async fn uart_writer_task<'d>(mut uart_tx: UartTx<'d, Async>) -> ! {
    loop {
        let byte = INPUT.wait().await;
        let _ = uart_tx.write(&[byte]).await;
        let _ = SIGNAL.wait().await;
        ECHO.signal(byte);
        info!("byte is forwarded");
    }
}
