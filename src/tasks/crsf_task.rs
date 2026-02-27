use embassy_executor::task;
use embassy_stm32::peripherals::{DMA1_CH2, UART4};
use embassy_stm32::usart::UartRx;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Sender;

use crate::drivers::crsf::CrsfParser;
use crate::state::RcData;

/// CRSF/ELRS task — reads UART4 RX continuously and sends RcData on each parsed frame.
#[task]
pub async fn crsf_task(
    mut crsf_rx: UartRx<'static, UART4, DMA1_CH2>,
    crsf_tx: Sender<'static, CriticalSectionRawMutex, RcData, 1>,
) {
    let mut parser = CrsfParser::new();
    let mut buf = [0u8; 64];

    loop {
        // CRSF frames are small (26 bytes max). Read whatever arrives.
        if let Ok(()) = crsf_rx.read(&mut buf).await {
            if let Some(parsed) = parser.push_bytes(&buf) {
                let data = RcData { channels: parsed.channels };
                let _ = crsf_tx.try_send(data);
            }
        }
    }
}
