use embassy_executor::task;
use embassy_stm32::peripherals::{DMA1_CH1, DMA1_CH3, USART3};
use embassy_stm32::usart::Uart;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Sender;
use embassy_time::{Duration, Timer};
use embassy_futures::select::{select, Either};

use crate::drivers::gps::NmeaParser;
use crate::state::GpsData;

/// GPS task — reads NMEA from USART3 and sends GpsData when a new fix is parsed.
#[task]
pub async fn gps_task(
    mut gps_uart: Uart<'static, USART3, DMA1_CH3, DMA1_CH1>,
    gps_tx: Sender<'static, CriticalSectionRawMutex, GpsData, 1>,
) {
    let mut parser = NmeaParser::new();
    let mut buf = [0u8; 512];

    loop {
        // Wait for a burst of NMEA data (GPS sends at 10 Hz → 100ms window)
        match select(
            gps_uart.read_until_idle(&mut buf),
            Timer::after(Duration::from_millis(110)),
        )
        .await
        {
            Either::First(Ok(n)) => {
                parser.push_data(&buf[..n]);

                let d = &parser.data;
                let data = GpsData {
                    lat: d.lat,
                    lon: d.lon,
                    alt: d.alt,
                    sats: d.sats,
                    fix: d.fix,
                    speed_kts: d.speed,
                    course_deg: d.course,
                };
                let _ = gps_tx.try_send(data);
            }
            Either::First(Err(_)) | Either::Second(_) => {
                // UART error or timeout — keep looping
            }
        }
    }
}
