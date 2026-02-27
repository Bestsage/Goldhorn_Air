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
        // Soft reset (Device Config register 0x11, bit 0)
        self.write_reg(0x11, 0x01).await?;
        Timer::after(Duration::from_millis(10)).await;

        // Verify WHO_AM_I = 0x47 for ICM-42688-P
        let _id = self.read_reg(0x75).await?;

        // ── Set ODR to 1 kHz and configure DLPF ──────────────────────────────

        // GYRO_CONFIG0 (0x4F): Full scale ±2000 dps, ODR 1 kHz
        //   [7:5] FS_SEL = 0b000 → ±2000 dps (16.4 LSB/dps)
        //   [3:0] ODR    = 0b0110 → 1 kHz
        self.write_reg(0x4F, 0b000_0_0110).await?;

        // ACCEL_CONFIG0 (0x50): Full scale ±16G, ODR 1 kHz
        //   [7:5] FS_SEL = 0b000 → ±16G (2048 LSB/g)
        //   [3:0] ODR    = 0b0110 → 1 kHz
        self.write_reg(0x50, 0b000_0_0110).await?;

        // GYRO_CONFIG1 (0x51): enable DLPF, BW index 3 → ~258 Hz
        //   [2:0] GYRO_UI_FILT_BW = 0b011  (258 Hz @ 1 kHz ODR per DS table)
        self.write_reg(0x51, 0x03).await?;

        // ACCEL_CONFIG1 (0x53): enable DLPF, BW index 3 → ~258 Hz
        //   [5:3] ACCEL_UI_FILT_BW = 0b011
        self.write_reg(0x53, 0x03 << 3).await?; // bits [5:3]

        // Enable Gyro and Accel in Low Noise mode (PWR_MGMT0 0x4E)
        //   [3:2] GYRO_MODE  = 0b11 (Low Noise)
        //   [1:0] ACCEL_MODE = 0b11 (Low Noise)
        self.write_reg(0x4E, 0x0F).await?;
        Timer::after(Duration::from_millis(50)).await; // Wait for sensor startup

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
