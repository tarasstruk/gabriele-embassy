use defmt::error;
use embassy_rp::uart::{Async, UartRx};

const START_ACK: [u8; 2] = [0xA1, 0xA2];
const STOP_ACK: [u8; 2] = [0xA3, 0xA0];

pub(crate) enum MachineFeedback {
    Started,
    Stopped,
    #[allow(unused)]
    Unknown([u8; 2]),
}

impl From<[u8; 2]> for MachineFeedback {
    fn from(value: [u8; 2]) -> Self {
        match value {
            START_ACK => MachineFeedback::Started,
            STOP_ACK => MachineFeedback::Stopped,
            other => MachineFeedback::Unknown(other),
        }
    }
}

pub(crate) async fn check_feedback(rx: &mut UartRx<'static, Async>) -> Result<MachineFeedback, ()> {
    let mut buf = [0; 2];
    match rx.read(&mut buf).await {
        Ok(_) => Ok(buf.into()),
        Err(e) => {
            error!("UART read failure: {:?}", e);
            Err(())
        }
    }
}
