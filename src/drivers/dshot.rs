use cortex_m::asm;
use embassy_stm32::gpio::{AnyPin, Level, Output, Speed};

pub struct Dshot300 {
    pin: Output<'static, AnyPin>,
}

impl Dshot300 {
    const BIT_TOTAL_CYCLES: u32 = 560;
    const BIT1_HIGH_CYCLES: u32 = 420;
    const BIT1_LOW_CYCLES: u32 = Self::BIT_TOTAL_CYCLES - Self::BIT1_HIGH_CYCLES;
    const BIT0_HIGH_CYCLES: u32 = 210;
    const BIT0_LOW_CYCLES: u32 = Self::BIT_TOTAL_CYCLES - Self::BIT0_HIGH_CYCLES;
    const FRAME_GAP_CYCLES: u32 = 5200;

    pub fn new(pin: AnyPin) -> Self {
        Self {
            pin: Output::new(pin, Level::Low, Speed::VeryHigh),
        }
    }

    pub fn send_command(&mut self, command_11bit: u16, telemetry: bool) {
        let frame = dshot_frame(command_11bit, telemetry);
        self.send_frame(frame);
    }

    pub fn send_frame(&mut self, frame: u16) {
        // Disable interrupts during bit-bang to prevent timing corruption
        // from UART/I2C/USB ISRs (~60µs critical window)
        critical_section::with(|_cs| {
            for bit in (0..16).rev() {
                let one = ((frame >> bit) & 0x1) != 0;

                self.pin.set_high();
                if one {
                    asm::delay(Self::BIT1_HIGH_CYCLES);
                    self.pin.set_low();
                    asm::delay(Self::BIT1_LOW_CYCLES);
                } else {
                    asm::delay(Self::BIT0_HIGH_CYCLES);
                    self.pin.set_low();
                    asm::delay(Self::BIT0_LOW_CYCLES);
                }
            }

            self.pin.set_low();
            asm::delay(Self::FRAME_GAP_CYCLES);
        });
    }
}

pub fn dshot_frame(command: u16, telemetry: bool) -> u16 {
    let mut packet = (command & 0x07ff) << 1;
    if telemetry {
        packet |= 1;
    }

    let mut csum = 0u16;
    let mut csum_data = packet;
    for _ in 0..3 {
        csum ^= csum_data;
        csum_data >>= 4;
    }
    csum &= 0x000f;

    (packet << 4) | csum
}
