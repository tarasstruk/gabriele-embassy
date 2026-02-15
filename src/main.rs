#![no_std]
#![no_main]
#![allow(async_fn_in_trait)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::join::{join, join4};
use embassy_rp::peripherals::{DMA_CH0, PIO0, PIO1, USB};
use embassy_rp::pio::program::pio_asm;
use embassy_rp::pio::{Common, Config as PioConf, Pio, ShiftConfig, ShiftDirection, StateMachine};
use {defmt_rtt as _, panic_probe as _};

use cyw43::JoinOptions;
use cyw43_pio::{DEFAULT_CLOCK_DIVIDER, PioSpi};
use embassy_net::StackResources;
use embassy_rp::clocks::RoscRng;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::uart::{Async, UartTx};
use embassy_rp::usb::{Driver, Instance};
use embassy_rp::{bind_interrupts, pio, uart};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::signal::Signal;
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, Sender, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, Config};
use fixed::types::extra::U8;
use static_cell::StaticCell;

// #[unsafe(link_section = ".boot_loader")]
// #[used]
// pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

// use panic_halt as _;

#[defmt::panic_handler]
fn panic() -> ! {
    panic_probe::hard_fault();
}

static SIGNAL: Signal<ThreadModeRawMutex, ()> = Signal::new();
static INPUT: Signal<ThreadModeRawMutex, u8> = Signal::new();
static ECHO: Signal<ThreadModeRawMutex, u8> = Signal::new();

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<USB>;
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

    // Create the driver, from the HAL.
    let driver = Driver::new(p.USB, Irqs);

    // Create embassy-usb Config
    let mut config = Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("PIO UART example");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );

    // Create classes on the builder.
    let class = CdcAcmClass::new(&mut builder, &mut state, 64);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_runner = usb.run();

    // Create UART writer
    let mut uart_config = uart::Config::default();
    uart_config.baudrate = 4800;
    let mut uart_tx: UartTx<'_, Async> = UartTx::new(p.UART0, p.PIN_0, p.DMA_CH1, uart_config);

    // PIO machinery
    let pio::Pio {
        mut common,
        mut sm0,
        ..
    } = pio::Pio::new(p.PIO1, Irqs);

    setup_pio_task_sm0(&mut common, &mut sm0);

    let (mut usb_tx, mut usb_rx) = class.split();

    // Read + write from USB
    let usb_future = async {
        loop {
            usb_rx.wait_connection().await;
            let _ = join(usb_read(&mut usb_rx), usb_write(&mut usb_tx)).await;
        }
    };

    // UART writer worker
    let uart_future = uart_writer_task(&mut uart_tx);

    let pio_task_future = pio_task_sm0(sm0);

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

    // Run everything concurrently.
    join4(usb_runner, usb_future, uart_future, pio_task_future).await;
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic(),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

async fn usb_read<'d, T: Instance + 'd>(
    usb_rx: &mut Receiver<'d, Driver<'d, T>>,
) -> Result<(), Disconnected> {
    let mut buf = [0; 1];
    loop {
        info!("byte read");
        let _n = usb_rx.read_packet(&mut buf).await?;
        let byte = buf[0];
        INPUT.signal(byte);
    }
}

// wait for a feedback from typewriter from ECHO
// and write the received byte to USB port
async fn usb_write<'d, T: Instance + 'd>(
    usb_tx: &mut Sender<'d, Driver<'d, T>>,
) -> Result<(), Disconnected> {
    loop {
        let byte = ECHO.wait().await;
        usb_tx.write_packet(&[byte]).await?;
    }
}

// async fn uart_write<PIO: pio::Instance, const SM: usize>(
//     uart_tx: &mut PioUartTx<'_, PIO, SM>,
// ) -> ! {
//     loop {
//         let byte = INPUT.wait().await;
//         let _ = uart_tx.write_u8(byte).await;
//         let _ = SIGNAL.wait().await;
//         ECHO.signal(byte);
//         info!("byte forwarded");
//     }
// }

async fn uart_writer_task<'d>(uart_tx: &mut UartTx<'d, Async>) -> ! {
    loop {
        let byte = INPUT.wait().await;
        let _ = uart_tx.write(&[byte]).await;
        let _ = SIGNAL.wait().await;
        ECHO.signal(byte);
        info!("byte is forwarded");
    }
}
