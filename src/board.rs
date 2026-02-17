use embassy_stm32::rcc::*;
use embassy_stm32::time::Hertz as TimeHertz;
use embassy_stm32::Config;

pub struct Board {
    pub p: embassy_stm32::Peripherals,
}

impl Board {
    pub fn init() -> Self {
        let mut config = Config::default();
        config.rcc.hse = Some(Hse {
            freq: TimeHertz(8_000_000), // Quartz 8MHz
            mode: HseMode::Oscillator,
        });
        config.rcc.pll_src = PllSource::HSE;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL168,
            divp: Some(PllPDiv::DIV2), // 168 MHz
            divq: Some(PllQDiv::DIV7), // 48 MHz pour USB
            divr: None,
        });
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV4;
        config.rcc.apb2_pre = APBPrescaler::DIV2;

        let p = embassy_stm32::init(config);

        Self { p }
    }
}
