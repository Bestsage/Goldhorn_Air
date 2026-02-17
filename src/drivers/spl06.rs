use embassy_stm32::i2c::{Error, I2c, Instance, RxDma, TxDma};
use embassy_time::{Duration, Timer};
use micromath::F32Ext;

const ADDR: u8 = 0x76;
const REG_CHIP_ID: u8 = 0x0D;
const REG_PRESS_DATA: u8 = 0x00;
const REG_TEMP_DATA: u8 = 0x03;
const REG_PRS_CFG: u8 = 0x06;
const REG_TMP_CFG: u8 = 0x07;
const REG_MEAS_CFG: u8 = 0x08;
#[allow(dead_code)]
const REG_CFG_REG: u8 = 0x09;
#[allow(dead_code)]
const REG_RESET: u8 = 0x0C;
const REG_COEF: u8 = 0x10;

#[allow(dead_code)]
const CHIP_ID: u8 = 0x10;

#[derive(Default, Debug, Clone, Copy)]
pub struct Spl06Coeffs {
    c0: i16,
    c1: i16,
    c00: i32,
    c10: i32,
    c01: i16,
    c11: i16,
    c20: i16,
    c21: i16,
    c30: i16,
}

pub struct Spl06 {
    coeffs: Spl06Coeffs,
    // Scaling factors based on oversampling (assuming defaults for now)
    k_p: f32,
    k_t: f32,
}

impl Spl06 {
    pub fn new() -> Self {
        Self {
            coeffs: Spl06Coeffs::default(),
            k_p: 7864320.0, // Default for 32x oversampling (datasheet typically varies)
            k_t: 7864320.0,
        }
    }

    pub async fn init<T: Instance, Tx: TxDma<T>, Rx: RxDma<T>>(
        &mut self,
        i2c: &mut I2c<'_, T, Tx, Rx>,
    ) -> Result<(), Error> {
        // Soft Reset
        self.write_reg(i2c, REG_RESET, 0x09).await?;
        Timer::after(Duration::from_millis(50)).await;

        let _id = self.read_id(i2c).await?;

        // Read calibration coeffs
        self.read_coeffs(i2c).await?;

        // Configure Pressure (8 samples, 4 measurements/sec) -> PM_RATE=4, PM_PRC=8
        // reg 0x06: BIT 6-4 (PM_RATE), BIT 3-0 (PM_PRC)
        // Let's set PM_PRC to 011 (kP=7864320, 8 times) -> 0x03
        // PM_RATE to 100 (16 meas/sec) -> 0x40
        // Total 0x43
        self.write_reg(i2c, REG_PRS_CFG, 0x43).await?;

        // Configure Temp (8 samples, 4 meas/sec)
        // reg 0x07: similar. 0x83 (TMP_EXT=1, TMP_RATE=0, TMP_PRC=3)
        self.write_reg(i2c, REG_TMP_CFG, 0x83).await?;

        // Measurement Config: Continuous Pressure and Temp
        // reg 0x08: MEAS_CTRL=111 (Cont Temp & Press) -> 0x07
        self.write_reg(i2c, REG_MEAS_CFG, 0x07).await?;

        // Wait for config to take effect
        Timer::after(Duration::from_millis(50)).await;

        // Update K factors based on configuration (oversampling 8x -> scale factor 7864320.0 ? Check datasheet)
        // Datasheet Table 4:
        // 8x oversampling -> Scale Factor (kP/kT) = 7864320
        self.k_p = 7864320.0;
        self.k_t = 7864320.0;

        Ok(())
    }

    pub async fn read_id<T: Instance, Tx: TxDma<T>, Rx: RxDma<T>>(
        &mut self,
        i2c: &mut I2c<'_, T, Tx, Rx>,
    ) -> Result<u8, Error> {
        let mut buf = [0u8; 1];
        i2c.blocking_write_read(ADDR, &[REG_CHIP_ID], &mut buf)?;
        Ok(buf[0])
    }

    // Read 3 bytes from register
    async fn read_24bits<T: Instance, Tx: TxDma<T>, Rx: RxDma<T>>(
        &mut self,
        i2c: &mut I2c<'_, T, Tx, Rx>,
        reg: u8,
    ) -> Result<i32, Error> {
        let mut buf = [0u8; 3];
        i2c.blocking_write_read(ADDR, &[reg], &mut buf)?;
        // Combine: MSB, byte1, LSB
        let val = ((buf[0] as i32) << 16) | ((buf[1] as i32) << 8) | (buf[2] as i32);
        // Sign extend if needed (24 bit 2's complement)
        let val = if val & 0x800000 != 0 {
            val | !0xFFFFFF
        } else {
            val
        };
        Ok(val)
    }

    async fn read_coeffs<T: Instance, Tx: TxDma<T>, Rx: RxDma<T>>(
        &mut self,
        i2c: &mut I2c<'_, T, Tx, Rx>,
    ) -> Result<(), Error> {
        let mut buf = [0u8; 18];
        i2c.blocking_write_read(ADDR, &[REG_COEF], &mut buf)?;

        let c0_raw = ((buf[0] as i16) << 4) | ((buf[1] as i16) >> 4);
        self.coeffs.c0 = if c0_raw & 0x800 != 0 {
            c0_raw | !0xFFF
        } else {
            c0_raw
        };

        let c1_raw = ((buf[1] as i16 & 0x0F) << 8) | (buf[2] as i16);
        self.coeffs.c1 = if c1_raw & 0x800 != 0 {
            c1_raw | !0xFFF
        } else {
            c1_raw
        };

        // c00 is 20 bits
        let c00_raw = ((buf[3] as i32) << 12) | ((buf[4] as i32) << 4) | ((buf[5] as i32) >> 4);
        self.coeffs.c00 = if c00_raw & 0x80000 != 0 {
            c00_raw | !0xFFFFF
        } else {
            c00_raw
        };

        // c10 is 20 bits
        let c10_raw = ((buf[5] as i32 & 0x0F) << 16) | ((buf[6] as i32) << 8) | (buf[7] as i32);
        self.coeffs.c10 = if c10_raw & 0x80000 != 0 {
            c10_raw | !0xFFFFF
        } else {
            c10_raw
        };

        self.coeffs.c01 = ((buf[8] as i16) << 8) | (buf[9] as i16);
        self.coeffs.c11 = ((buf[10] as i16) << 8) | (buf[11] as i16);
        self.coeffs.c20 = ((buf[12] as i16) << 8) | (buf[13] as i16);
        self.coeffs.c21 = ((buf[14] as i16) << 8) | (buf[15] as i16);
        self.coeffs.c30 = ((buf[16] as i16) << 8) | (buf[17] as i16);

        Ok(())
    }

    async fn write_reg<T: Instance, Tx: TxDma<T>, Rx: RxDma<T>>(
        &mut self,
        i2c: &mut I2c<'_, T, Tx, Rx>,
        reg: u8,
        val: u8,
    ) -> Result<(), Error> {
        i2c.blocking_write(ADDR, &[reg, val])
    }

    pub async fn read_pressure_altitude<T: Instance, Tx: TxDma<T>, Rx: RxDma<T>>(
        &mut self,
        i2c: &mut I2c<'_, T, Tx, Rx>,
    ) -> Result<(f32, f32, f32), Error> {
        // Read Raw Data
        let p_raw_val = self.read_24bits(i2c, REG_PRESS_DATA).await?;
        let p_raw = p_raw_val as f32;
        let t_raw = self.read_24bits(i2c, REG_TEMP_DATA).await? as f32;

        // Scalling
        let p_sc = p_raw / self.k_p;
        let t_sc = t_raw / self.k_t;

        // Calculate Temp
        let _temp = self.coeffs.c0 as f32 * 0.5 + self.coeffs.c1 as f32 * t_sc;

        // Calculate Pressure (Compensated)
        // Pcomp = c00 + P_sc*(c10 + P_sc*(c20 + P_sc*c30)) + T_sc*c01 + T_sc*P_sc*(c11 + P_sc*c21)
        let term1 = self.coeffs.c00 as f32;
        let term2 = p_sc
            * (self.coeffs.c10 as f32
                + p_sc * (self.coeffs.c20 as f32 + p_sc * self.coeffs.c30 as f32));
        let term3 = t_sc * self.coeffs.c01 as f32;
        let term4 = t_sc * p_sc * (self.coeffs.c11 as f32 + p_sc * self.coeffs.c21 as f32);

        let pressure = term1 + term2 + term3 + term4; // Pascals

        // Convert to Altitude (Hypsometric Formula)
        // Alt = 44330 * (1.0 - (P / P0)^(1/5.255))
        // P0 = 101325 Pa

        let p0 = 101325.0;
        let power = 1.0 / 5.255;
        let alt = 44330.0 * (1.0 - (pressure / p0).powf(power));

        Ok((alt, pressure, _temp))
    }
}
