use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{AnyPin, Output};
use embassy_stm32::spi::{Error, Instance, Spi};
use embassy_time::{Duration, Timer};

pub struct Icm42688<'d, T: Instance> {
    spi: Spi<'d, T, NoDma, NoDma>,
    cs: Output<'d, AnyPin>,
}

impl<'d, T: Instance> Icm42688<'d, T> {
    pub fn new(spi: Spi<'d, T, NoDma, NoDma>, cs: Output<'d, AnyPin>) -> Self {
        Self { spi, cs }
    }

    async fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Error> {
        let buf = [reg & 0x7F, value];
        self.cs.set_low();
        let res = self.spi.blocking_write(&buf);
        self.cs.set_high();
        res
    }

    async fn read_reg(&mut self, reg: u8) -> Result<u8, Error> {
        let tx = [reg | 0x80, 0x00];
        let mut rx = [0u8; 2];

        self.cs.set_low();
        let res = self.spi.blocking_transfer(&mut rx, &tx);
        self.cs.set_high();

        let _ = res?;
        Ok(rx[1])
    }

    pub async fn init(&mut self) -> Result<(), Error> {
        // Soft reset
        self.write_reg(0x11, 0x01).await?;
        Timer::after(Duration::from_millis(10)).await;

        // Check ID
        let _id = self.read_reg(0x75).await?;
        // We can inspect ID here if we had logging

        // Enable Gyro and Accel in Low Noise mode
        self.write_reg(0x4E, 0x0F).await?;
        Timer::after(Duration::from_millis(50)).await; // Wait for startup

        Ok(())
    }

    #[allow(dead_code)]
    pub async fn read_who_am_i(&mut self) -> Result<u8, Error> {
        self.read_reg(0x75).await
    }

    pub async fn read_all(&mut self) -> Result<([i16; 3], [i16; 3]), Error> {
        let mut tx = [0u8; 13];
        tx[0] = 0x1F | 0x80;
        let mut rx = [0u8; 13];

        self.cs.set_low();
        self.spi.blocking_transfer(&mut rx, &tx)?;
        self.cs.set_high();

        let a_x = (rx[1] as i16) << 8 | (rx[2] as i16);
        let a_y = (rx[3] as i16) << 8 | (rx[4] as i16);
        let a_z = (rx[5] as i16) << 8 | (rx[6] as i16);

        let g_x = (rx[7] as i16) << 8 | (rx[8] as i16);
        let g_y = (rx[9] as i16) << 8 | (rx[10] as i16);
        let g_z = (rx[11] as i16) << 8 | (rx[12] as i16);

        Ok(([a_x, a_y, a_z], [g_x, g_y, g_z]))
    }
}
