use embassy_executor::task;
use embassy_stm32::i2c::I2c;
use embassy_stm32::peripherals::{DMA1_CH0, DMA1_CH7, I2C1};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Sender;
use embassy_time::{Duration, Ticker};

use crate::drivers::spl06::Spl06;
use crate::state::BaroData;

/// Barometer task — reads SPL06 at 20 Hz and sends BaroData to the fast loop.
#[task]
pub async fn baro_task(
    mut i2c: I2c<'static, I2C1, DMA1_CH7, DMA1_CH0>,
    baro_tx: Sender<'static, CriticalSectionRawMutex, BaroData, 1>,
) {
    let mut baro = Spl06::new();
    // SPL06 init
    if baro.init(&mut i2c).await.is_err() {
        // If init fails we still loop but data will be zero
    }

    let mut ticker = Ticker::every(Duration::from_hz(20));
    loop {
        ticker.next().await;

        if let Ok((alt_m, press_pa, temp_c)) = baro.read_pressure_altitude(&mut i2c).await {
            let data = BaroData {
                alt_m,
                pressure_hpa: press_pa / 100.0,
                temp_c,
            };
            // Overwrite any unread value — always send latest
            let _ = baro_tx.try_send(data);
        }
    }
}
