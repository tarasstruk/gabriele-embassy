use crate::{ECHO, INPUT, SIGNAL, UART_READY};
use cyw43_pio::PioSpi;
use defmt::info;
use embassy_rp::gpio::Output;
use embassy_rp::peripherals::{DMA_CH0, PIO0, PIO1};
use embassy_rp::pio::StateMachine;
use embassy_rp::uart::{Async, UartTx};

#[embassy_executor::task]
pub async fn net_task(mut runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    runner.run().await
}

// signals when the "confirmation pulse" from typewriter is received
#[embassy_executor::task]
pub async fn pio_task_sm0(mut sm: StateMachine<'static, PIO1, 0>) -> ! {
    loop {
        let _ = sm.rx().wait_pull().await;
        SIGNAL.signal(());
    }
}

#[embassy_executor::task]
pub async fn cyw43_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

// these steps are done:
// - receive a byte from INPUT
// - transmit the byte to UART
// - wait for SIGNAL
// - produce ECHO
#[embassy_executor::task]
pub async fn uart_tx_task(mut tx: UartTx<'static, Async>) -> ! {
    // let uart_future = uart_writer(tx);
    // uart_future.await;
    loop {
        UART_READY.signal(());
        let byte = INPUT.wait().await;
        info!("INPUT: {:02x}", byte);
        let res = tx.write(&[byte]).await;
        info!("WRITTEN: {:02x} WITH RESULT {:?}", byte, res);
        let _ = SIGNAL.wait().await;
        info!("SIGNAL RECV FOR: {:02x}", byte);
        ECHO.signal(byte);
        info!("byte is forwarded");
    }
}
