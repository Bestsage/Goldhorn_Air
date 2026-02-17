use embassy_stm32::i2c::{I2c, Instance, RxDma, TxDma};
use embassy_time::Timer;

pub const HMC5883L_ADDR: u8 = 0x1E;

pub struct Hmc5883;

impl Hmc5883 {
    pub fn new() -> Self {
        Self
    }

    pub async fn init<T: Instance, Tx: TxDma<T>, Rx: RxDma<T>>(
        &mut self,
        i2c: &mut I2c<'_, T, Tx, Rx>,
    ) -> Result<(), embassy_stm32::i2c::Error> {
        // Configuration Register A: 8-average, 15Hz default, normal measurement
        i2c.blocking_write(HMC5883L_ADDR, &[0x00, 0x70])?;

        // Configuration Register B: Gain 1.3 Ga (default)
        i2c.blocking_write(HMC5883L_ADDR, &[0x01, 0x20])?;

        // Mode Register: Continuous-measurement mode
        i2c.blocking_write(HMC5883L_ADDR, &[0x02, 0x00])?;

        Timer::after_millis(10).await;
        Ok(())
    }

    pub async fn read_mag<T: Instance, Tx: TxDma<T>, Rx: RxDma<T>>(
        &mut self,
        i2c: &mut I2c<'_, T, Tx, Rx>,
    ) -> Result<[i16; 3], embassy_stm32::i2c::Error> {
        let mut data = [0u8; 6];
        // Read starting from 0x03 (Data Output X MSB Register)
        i2c.blocking_write_read(HMC5883L_ADDR, &[0x03], &mut data)?;

        // Note: HMC5883L layout is X, Z, Y
        let x = i16::from_be_bytes([data[0], data[1]]);
        let z = i16::from_be_bytes([data[2], data[3]]);
        let y = i16::from_be_bytes([data[4], data[5]]);

        Ok([x, y, z])
    }
}
