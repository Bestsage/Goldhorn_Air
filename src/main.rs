#![no_std]
#![no_main]

mod board;
mod drivers;
mod state;
mod tasks;
mod usb;

use core::sync::atomic::{AtomicU16, Ordering};
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Pin, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::spi::{Config as SpiConfig, Spi};
use embassy_stm32::time::Hertz as TimeHertz;
use embassy_stm32::usart::{Config as UsartConfig, Uart};
use embassy_stm32::{bind_interrupts, peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

use crate::board::Board;
use crate::drivers::dshot::Dshot300;
use crate::drivers::gps;
use crate::drivers::icm42688::Icm42688;
use crate::state::{AttitudeState, BaroData, GpsData, RcData};
use crate::tasks::fast_loop::{fast_loop_task, FastLoopConfig};

// ── DShot shared command ──────────────────────────────────────────────────────
pub static TAB_MOTOR_DSHOT_CMD: AtomicU16 = AtomicU16::new(0);

// ── Inter-task channels ───────────────────────────────────────────────────────
//  Cap=1: the fast_loop always wants the LATEST sample; older values are dropped.
static BARO_CHAN:    Channel<CriticalSectionRawMutex, BaroData,     1> = Channel::new();
static GPS_CHAN:     Channel<CriticalSectionRawMutex, GpsData,      1> = Channel::new();
static CRSF_CHAN:    Channel<CriticalSectionRawMutex, RcData,       1> = Channel::new();

// Telemetry task reads attitude from fast_loop and sensor data from its own copies
static ATT_TEL_CHAN:  Channel<CriticalSectionRawMutex, AttitudeState, 1> = Channel::new();
static BARO_TEL_CHAN: Channel<CriticalSectionRawMutex, BaroData,      1> = Channel::new();
static GPS_TEL_CHAN:  Channel<CriticalSectionRawMutex, GpsData,       1> = Channel::new();

// ── Interrupt bindings ────────────────────────────────────────────────────────
bind_interrupts!(struct Irqs {
    I2C1_EV  => embassy_stm32::i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER  => embassy_stm32::i2c::ErrorInterruptHandler<peripherals::I2C1>;
    UART4    => embassy_stm32::usart::InterruptHandler<peripherals::UART4>;
    USART3   => embassy_stm32::usart::InterruptHandler<peripherals::USART3>;
});

// ── DShot task ────────────────────────────────────────────────────────────────
#[embassy_executor::task]
async fn dshot_tab_task(mut dshot: Dshot300) {
    const ESC_OUTPUT_LOCKED: bool = true;
    loop {
        let cmd = if ESC_OUTPUT_LOCKED {
            0
        } else {
            TAB_MOTOR_DSHOT_CMD.load(Ordering::Relaxed)
        };
        dshot.send_command(cmd, false);
        Timer::after(Duration::from_micros(1000)).await;
    }
}

// ── Main ──────────────────────────────────────────────────────────────────────
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // 1. Board init (168 MHz PLL)
    let board = Board::init();
    let p = board.p;

    // 2. USB (CDC-ACM for debug)
    let (usb_dev, usb_serial) = usb::init(p.USB_OTG_FS, p.PA12, p.PA11);
    spawner.spawn(usb::usb_task(usb_dev)).unwrap();

    // 3. I2C1 @ 400 kHz — SPL06 Baro (SCL=PB8, SDA=PB9)
    let i2c = I2c::new(
        p.I2C1,
        p.PB8, p.PB9,
        Irqs,
        p.DMA1_CH7,
        p.DMA1_CH0,
        TimeHertz(400_000),
        Default::default(),
    );

    // 4. SPI1 @ 10 MHz — ICM-42688 IMU (SCK=PA5, MOSI=PA7, MISO=PA6, CS=PB12)
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = TimeHertz(10_000_000);

    use embassy_stm32::dma::NoDma;
    let spi = Spi::new(
        p.SPI1,
        p.PA5, p.PA7, p.PA6,
        NoDma, NoDma,
        spi_config,
    );
    let cs_gyro = Output::new(p.PB12.degrade(), Level::High, Speed::VeryHigh);
    let mut imu = Icm42688::new(spi, cs_gyro);

    // 5. DShot tab motor on PB0 (MOTOR1 resource)
    let dshot_tab_motor = Dshot300::new(p.PB0.degrade());
    spawner.spawn(dshot_tab_task(dshot_tab_motor)).unwrap();

    // 6. GPS USART3 @ 115200 (TX=PB10, RX=PB11)
    let mut gps_config = UsartConfig::default();
    gps_config.baudrate = 115_200;
    let mut gps_uart = Uart::new(
        p.USART3, p.PB11, p.PB10,
        Irqs,
        p.DMA1_CH3, p.DMA1_CH1,
        gps_config,
    ).unwrap();

    // 7. CRSF/ELRS UART4 @ 420000 (TX=PA0, RX=PA1)
    //    Split into Tx (→ telemetry_task) and Rx (→ crsf_task)
    let mut crsf_config = UsartConfig::default();
    crsf_config.baudrate = 420_000;
    let crsf_uart = Uart::new(
        p.UART4, p.PA1, p.PA0,
        Irqs,
        p.DMA1_CH4, p.DMA1_CH2,
        crsf_config,
    ).unwrap();
    let (crsf_uart_tx, crsf_uart_rx) = crsf_uart.split();

    // 8. Heartbeat LED (PC13)
    let mut led = Output::new(p.PC13, Level::High, Speed::Low);

    // 9. IMU hardware init (DLPF 258 Hz, ODR 1 kHz set inside)
    Timer::after(Duration::from_millis(100)).await;
    let _ = imu.init().await;

    // 10. GPS UBX configuration (one-shot at startup)
    Timer::after(Duration::from_millis(200)).await;
    {
        let (buf, len) = gps::ubx_cfg_gnss_all();
        let _ = gps_uart.write(&buf[..len]).await;
        Timer::after(Duration::from_millis(200)).await;
        let (buf, len) = gps::ubx_cfg_nav_sbas_rate();
        let _ = gps_uart.write(&buf[..len]).await;
        Timer::after(Duration::from_millis(200)).await;
    }

    // 11. Static gyro/accel calibration: 100 samples × 10 ms = 1 s
    let mut gyro_bias  = [0.0f32; 3];
    let mut accel_bias = [0.0f32; 3];
    const CALIB_N: usize = 100;
    for i in 0..CALIB_N {
        if let Ok((accel, gyro)) = imu.read_all().await {
            for j in 0..3 {
                accel_bias[j] += accel[j] as f32;
                gyro_bias[j]  += gyro[j]  as f32;
            }
        }
        if i % 10 == 0 { led.toggle(); }
        Timer::after(Duration::from_millis(10)).await;
    }
    for j in 0..3 {
        accel_bias[j] /= CALIB_N as f32;
        gyro_bias[j]  /= CALIB_N as f32;
    }
    accel_bias[2] -= 2048.0; // Remove gravity (1G = 2048 LSB at ±16G)
    led.set_high(); // Calibration done

    // 12. Build IMU for 'static use via a leaked Box-equivalent
    //     Embassy tasks require 'static resources. Since we own `imu` and the
    //     program never ends, leaking is the correct embedded approach.
    let imu_ref: &'static mut Icm42688<'static, peripherals::SPI1> = {
        use static_cell::StaticCell;
        static IMU_CELL: StaticCell<Icm42688<'static, peripherals::SPI1>> = StaticCell::new();
        IMU_CELL.init(imu)
    };

    // 13. Spawn all task
    spawner.spawn(fast_loop_task(
        unsafe { core::ptr::read(imu_ref) },
        FastLoopConfig { gyro_bias, accel_bias },
        BARO_CHAN.receiver(),
        GPS_CHAN.receiver(),
        CRSF_CHAN.receiver(),
        ATT_TEL_CHAN.sender(),
    )).unwrap();

    spawner.spawn(tasks::baro_task::baro_task(
        i2c,
        BARO_CHAN.sender(),
    )).unwrap();

    spawner.spawn(tasks::gps_task::gps_task(
        gps_uart,
        GPS_CHAN.sender(),
    )).unwrap();

    spawner.spawn(tasks::crsf_task::crsf_task(
        crsf_uart_rx,
        CRSF_CHAN.sender(),
    )).unwrap();

    spawner.spawn(tasks::telemetry_task::telemetry_task(
        crsf_uart_tx,
        usb_serial,
        ATT_TEL_CHAN.receiver(),
        GPS_TEL_CHAN.receiver(),
        BARO_TEL_CHAN.receiver(),
    )).unwrap();

    // 14. Main task: LED heartbeat @ 1 Hz
    loop {
        led.toggle();
        Timer::after(Duration::from_millis(500)).await;
    }
}
