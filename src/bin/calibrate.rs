#![no_std]
#![no_main]

//! # Goldhorn_Air — Calibration Longue Durée (1 heure)
//!
//! Enregistre les données brutes de tous les capteurs et les envoie
//! par USB CDC-ACM au format CSV pour analyse Allan Variance dans MATLAB.
//!
//! ## Usage
//! ```sh
//! # Flasher ce binaire
//! cargo flash --release --bin calibrate --chip STM32F405RG
//!
//! # Capturer les données USB
//! cat /dev/ttyACM0 > calib_data.csv   # Linux
//! # ou ouvrir minicom -D /dev/ttyACM0 -b 115200
//! ```
//!
//! ## Format CSV
//! `ts_ms,gx_lsb,gy_lsb,gz_lsb,ax_lsb,ay_lsb,az_lsb,baro_alt_cm,baro_press_pa,baro_temp_mc,mag_x,mag_y,mag_z`
//!
//! ## Script MATLAB (copier-coller)
//!
//! ```matlab
//! %% 1. Charger
//! T = readtable('calib_data.csv', 'CommentStyle', '#');
//! T.Properties.VariableNames = {'ts_ms','gx','gy','gz','ax','ay','az',...
//!     'baro_alt_cm','baro_press_pa','baro_temp_mc','mag_x','mag_y','mag_z'};
//! Fs_imu = 500; Fs_baro = 20;
//!
//! %% 2. Conversion
//! gx = deg2rad(double(T.gx)/16.4); gy = deg2rad(double(T.gy)/16.4); gz = deg2rad(double(T.gz)/16.4);
//! ax = double(T.ax)/2048; ay = double(T.ay)/2048; az = double(T.az)/2048;
//!
//! %% 3. Allan Variance Gyro
//! figure; hold on;
//! for ch = {gx,gy,gz}
//!     [av,tau] = allanvar(ch{1},'octave',Fs_imu); loglog(tau,sqrt(av));
//! end
//! legend('Gx','Gy','Gz'); grid on;
//! xlabel('\tau (s)'); ylabel('Allan Dev (rad/s)');
//! title('Lire: ARW(pente-0.5), Bias Instability(minimum)');
//!
//! %% 4. Allan Variance Accel
//! figure; hold on;
//! for ch = {ax,ay,az}
//!     [av,tau] = allanvar(ch{1},'octave',Fs_imu); loglog(tau,sqrt(av));
//! end
//! legend('Ax','Ay','Az'); grid on;
//! xlabel('\tau (s)'); ylabel('Allan Dev (g)');
//!
//! %% 5. R_ACCEL_NORMAL
//! accel_mag = sqrt(ax.^2+ay.^2+az.^2);
//! R_accel = var(accel_mag - 1.0);
//! fprintf('const R_ACCEL_NORMAL: f32 = %.6f;\n', R_accel);
//!
//! %% 6. Q_QUAT et Q_GBIAS depuis Allan
//! [av_g,tau_g] = allanvar(gz,'octave',Fs_imu);
//! adev_g = sqrt(av_g);
//! ARW = adev_g(1)/sqrt(Fs_imu);
//! [bi,~] = min(adev_g);
//! fprintf('const Q_QUAT:  f32 = %.2e;\n', ARW^2/Fs_imu);
//! fprintf('const Q_GBIAS: f32 = %.2e;\n', bi^2/Fs_imu);
//!
//! %% 7. Baro variance
//! baro_m = double(T.baro_alt_cm(1:Fs_imu/Fs_baro:end))/100;
//! fprintf('Baro noise std: %.4f m\n', std(diff(baro_m)));
//!
//! %% 8. Mag hard-iron
//! figure; plot(T.mag_x,T.mag_y,'.','MarkerSize',1); axis equal; grid on;
//! fprintf('Mag offset (hard-iron): X=%.0f Y=%.0f Z=%.0f LSB\n',...
//!     mean(T.mag_x), mean(T.mag_y), mean(T.mag_z));
//! ```

// ── Modules (chemins explicites depuis src/bin/) ──────────────────────────────
#[path = "../board.rs"]   mod board;
#[path = "../usb.rs"]     mod usb;
#[path = "../drivers/mod.rs"]
mod drivers {
    #[path = "icm42688.rs"] pub mod icm42688;
    #[path = "spl06.rs"]    pub mod spl06;
    #[path = "hmc5883.rs"]  pub mod hmc5883;
    #[path = "crsf.rs"]     pub mod crsf;
    #[path = "dshot.rs"]    pub mod dshot;
    #[path = "ekf.rs"]      pub mod ekf;
    #[path = "filter.rs"]   pub mod filter;
    #[path = "flash.rs"]    pub mod flash;
    #[path = "gps.rs"]      pub mod gps;
    #[path = "kalman.rs"]   pub mod kalman;
    #[path = "roll.rs"]     pub mod roll;
}

use core::fmt::Write;
use core::sync::atomic::{AtomicI32, AtomicU32, Ordering};
use embassy_executor::Spawner;
use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{Level, Output, Pin, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::spi::{Config as SpiConfig, Spi};
use embassy_stm32::time::Hertz as TimeHertz;
use embassy_stm32::{bind_interrupts, peripherals};
use embassy_time::{Duration, Instant, Ticker, Timer};
use {defmt_rtt as _, panic_probe as _};

use crate::board::Board;
use crate::drivers::hmc5883::Hmc5883;
use crate::drivers::icm42688::Icm42688;
use crate::drivers::spl06::Spl06;

// ── Paramètres ────────────────────────────────────────────────────────────────

/// 1 heure en millisecondes
const CALIB_DURATION_MS: u64 = 3_600_000;

/// Fréquence IMU (Hz) — haute pour une bonne résolution Allan Variance
const IMU_RATE_HZ: u64 = 500;

/// Fréquence baro (Hz)
const BARO_RATE_HZ: u64 = 20;

/// Fréquence magnétomètre (Hz)
const MAG_RATE_HZ: u64 = 10;

// ── Données partagées baro/mag (atomes, mis à jour par baro_task) ─────────────
static BARO_ALT_CM:    AtomicI32 = AtomicI32::new(0);
static BARO_PRESS_PA:  AtomicU32 = AtomicU32::new(0);
static BARO_TEMP_MC:   AtomicI32 = AtomicI32::new(0);
static MAG_X:          AtomicI32 = AtomicI32::new(0);
static MAG_Y:          AtomicI32 = AtomicI32::new(0);
static MAG_Z:          AtomicI32 = AtomicI32::new(0);

// ── Interruptions ─────────────────────────────────────────────────────────────
bind_interrupts!(struct Irqs {
    I2C1_EV => embassy_stm32::i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => embassy_stm32::i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

// ── Tâche USB ─────────────────────────────────────────────────────────────────
#[embassy_executor::task]
async fn usb_task(mut device: embassy_usb::UsbDevice<'static, crate::usb::UsbDriver>) -> ! {
    device.run().await
}

// ── Tâche Baro + Mag (I2C) ───────────────────────────────────────────────────
#[embassy_executor::task]
async fn baro_mag_task(
    mut i2c: I2c<'static, peripherals::I2C1, peripherals::DMA1_CH7, peripherals::DMA1_CH0>,
) {
    let mut baro = Spl06::new();
    let mut mag  = Hmc5883::new();
    let _ = baro.init(&mut i2c).await;
    let _ = mag.init(&mut i2c).await;

    let baro_interval = Duration::from_hz(BARO_RATE_HZ);
    let mag_every     = (BARO_RATE_HZ / MAG_RATE_HZ) as u32;
    let mut tick: u32 = 0;

    loop {
        Timer::after(baro_interval).await;
        tick = tick.wrapping_add(1);

        if let Ok((alt_m, press_pa, temp_c)) = baro.read_pressure_altitude(&mut i2c).await {
            BARO_ALT_CM.store((alt_m * 100.0) as i32, Ordering::Relaxed);
            BARO_PRESS_PA.store(press_pa as u32, Ordering::Relaxed);
            BARO_TEMP_MC.store((temp_c * 1000.0) as i32, Ordering::Relaxed);
        }

        if tick % mag_every == 0 {
            if let Ok(m) = mag.read_mag(&mut i2c).await {
                MAG_X.store(m[0] as i32, Ordering::Relaxed);
                MAG_Y.store(m[1] as i32, Ordering::Relaxed);
                MAG_Z.store(m[2] as i32, Ordering::Relaxed);
            }
        }
    }
}

// ── Main ──────────────────────────────────────────────────────────────────────
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let board = Board::init();
    let p = board.p;

    // USB CDC
    let (usb_dev, mut usb_serial) = usb::init(p.USB_OTG_FS, p.PA12, p.PA11);
    spawner.spawn(usb_task(usb_dev)).unwrap();

    // LED (PC13)
    let mut led = Output::new(p.PC13, Level::High, Speed::Low);

    // I2C1 @ 400 kHz
    let i2c = I2c::new(
        p.I2C1, p.PB8, p.PB9, Irqs,
        p.DMA1_CH7, p.DMA1_CH0,
        TimeHertz(400_000), Default::default(),
    );

    // SPI1 @ 10 MHz → ICM-42688
    let mut spi_cfg = SpiConfig::default();
    spi_cfg.frequency = TimeHertz(10_000_000);
    let spi = Spi::new(p.SPI1, p.PA5, p.PA7, p.PA6, NoDma, NoDma, spi_cfg);
    let cs  = Output::new(p.PB12.degrade(), Level::High, Speed::VeryHigh);
    let mut imu = Icm42688::new(spi, cs);

    Timer::after(Duration::from_millis(200)).await;
    let _ = imu.init().await;

    spawner.spawn(baro_mag_task(i2c)).unwrap();

    // ── Attendre connexion USB (max 30s, puis démarre quand même) ─────────────
    for _ in 0..300u32 {
        led.toggle();
        Timer::after(Duration::from_millis(100)).await;
        if usb_serial.dtr() { break; }
    }
    led.set_high();
    Timer::after(Duration::from_millis(200)).await;

    // ── En-tête CSV ───────────────────────────────────────────────────────────
    let hdr = b"# Goldhorn_Air - 1h Allan Variance Calibration\r\n\
                # IMU: ICM-42688 @500Hz | Baro: SPL06 @20Hz | Mag: HMC5883 @10Hz\r\n\
                # LSB scale: gyro=16.4 LSB/dps | accel=2048 LSB/g (see file header for MATLAB)\r\n\
                # ts_ms,gx_lsb,gy_lsb,gz_lsb,ax_lsb,ay_lsb,az_lsb,\
                baro_alt_cm,baro_press_pa,baro_temp_mc,mag_x,mag_y,mag_z\r\n";
    let _ = usb_serial.write_packet(hdr).await;

    // ── Boucle d'acquisition ──────────────────────────────────────────────────
    let start      = Instant::now();
    let mut ticker = Ticker::every(Duration::from_hz(IMU_RATE_HZ));
    let mut n:    u64 = 0;
    let mut errs: u32 = 0;

    loop {
        ticker.next().await;

        let elapsed_ms = start.elapsed().as_millis();
        if elapsed_ms >= CALIB_DURATION_MS { break; }

        // Lecture IMU
        let (accel, gyro) = match imu.read_all().await {
            Ok(v) => v,
            Err(_) => { errs += 1; continue; }
        };

        // Lecture atomiques baro/mag
        let ba = BARO_ALT_CM.load(Ordering::Relaxed);
        let bp = BARO_PRESS_PA.load(Ordering::Relaxed);
        let bt = BARO_TEMP_MC.load(Ordering::Relaxed);
        let mx = MAG_X.load(Ordering::Relaxed);
        let my = MAG_Y.load(Ordering::Relaxed);
        let mz = MAG_Z.load(Ordering::Relaxed);

        // Ligne CSV (max ~110 caractères)
        let mut line = heapless::String::<128>::new();
        let _ = write!(line,
            "{},{},{},{},{},{},{},{},{},{},{},{},{}\r\n",
            elapsed_ms,
            gyro[0], gyro[1], gyro[2],
            accel[0], accel[1], accel[2],
            ba, bp, bt, mx, my, mz,
        );

        // Envoi USB par chunks de 64 octets (limite USB CDC)
        if usb_serial.dtr() {
            let b = line.as_bytes();
            let mut off = 0;
            while off < b.len() {
                let end = (off + 64).min(b.len());
                let _ = usb_serial.write_packet(&b[off..end]).await;
                off = end;
            }
        }

        n += 1;

        // LED 1 Hz
        if n % IMU_RATE_HZ == 0 { led.toggle(); }

        // Rapport toutes les 60 secondes
        if n % (IMU_RATE_HZ * 60) == 0 && usb_serial.dtr() {
            let s = elapsed_ms / 1000;
            let rem = (CALIB_DURATION_MS / 1000).saturating_sub(s);
            let mut msg = heapless::String::<96>::new();
            let _ = write!(msg,
                "# t={}s reste={}s n={}k err={}\r\n",
                s, rem, n / 1000, errs
            );
            let _ = usb_serial.write_packet(msg.as_bytes()).await;
        }
    }

    // ── Fin ───────────────────────────────────────────────────────────────────
    {
        let total_s = start.elapsed().as_millis() / 1000;
        let mut footer = heapless::String::<96>::new();
        let _ = write!(footer,
            "# FIN: {}s | {} echantillons | {} erreurs IMU\r\n",
            total_s, n, errs
        );
        if usb_serial.dtr() {
            let _ = usb_serial.write_packet(footer.as_bytes()).await;
        }
    }

    // Clignote vite → session terminée
    loop {
        led.toggle();
        Timer::after(Duration::from_millis(50)).await;
    }
}
