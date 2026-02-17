use embassy_stm32::gpio::{AnyPin, Output};
use embassy_stm32::spi::{Error, Instance, Spi};

#[allow(dead_code)]
const CMD_JEDEC_ID: u8 = 0x9F;

#[allow(dead_code)]
pub struct W25qxx<'d, T: Instance, Tx, Rx> {
    spi: Spi<'d, T, Tx, Rx>,
    cs: Output<'d, AnyPin>,
}

#[allow(dead_code)]
impl<'d, T: Instance, Tx, Rx> W25qxx<'d, T, Tx, Rx> {
    pub fn new(spi: Spi<'d, T, Tx, Rx>, cs: Output<'d, AnyPin>) -> Self {
        Self { spi, cs }
    }

    pub async fn read_id(&mut self) -> Result<[u8; 3], Error> {
        let mut id = [0u8; 3];
        self.cs.set_low();
        self.spi.blocking_transfer_in_place(&mut [CMD_JEDEC_ID])?;
        self.spi.blocking_read(&mut id)?;
        self.cs.set_high();
        Ok(id)
    }
}
