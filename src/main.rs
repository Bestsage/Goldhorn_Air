#![no_std]
#![no_main]

mod board;
mod drivers;
mod usb;

use core::fmt::Write;
use core::sync::atomic::{AtomicU16, Ordering};
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::spi::{Config as SpiConfig, Spi};
use embassy_stm32::time::Hertz as TimeHertz;
use embassy_stm32::usart::{Config as UsartConfig, Uart};
use embassy_stm32::{bind_interrupts, peripherals};
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

use crate::board::Board;
use crate::drivers::ahrs::Mahony;
use crate::drivers::dshot::Dshot300;
use crate::drivers::filter::{BiquadFilter, LowPassFilter, Pt1Filter};
use crate::drivers::flash::W25qxx;
use crate::drivers::hmc5883::Hmc5883;
use crate::drivers::icm42688::Icm42688;
use crate::drivers::kalman::VerticalKalman;
use crate::drivers::roll::{
    crsf_to_unit, max_roll_setpoint_from_stick, roll_output_to_tab_target_deg,
    signed_unit_to_dshot_3d, unit_to_dshot,
    GearRatio, GearedTabController, RollController,
};
use crate::drivers::spl06::Spl06;

static TAB_MOTOR_DSHOT_CMD: AtomicU16 = AtomicU16::new(0);
const ESC_OUTPUT_LOCKED: bool = true;

#[embassy_executor::task]
async fn dshot_tab_task(mut dshot: Dshot300) {
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

// Binding des interruptions pour les bus (USB est géré dans usb.rs)
bind_interrupts!(struct Irqs {
    I2C1_EV => embassy_stm32::i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => embassy_stm32::i2c::ErrorInterruptHandler<peripherals::I2C1>;
    UART4 => embassy_stm32::usart::InterruptHandler<peripherals::UART4>;
    USART3 => embassy_stm32::usart::InterruptHandler<peripherals::USART3>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // 1. Init Board (Clocks)
    let board = Board::init();
    let p = board.p;

    // 2. Init USB
    let (usb_dev, mut usb_serial) = usb::init(p.USB_OTG_FS, p.PA12, p.PA11);
    spawner.spawn(usb::usb_task(usb_dev)).unwrap();

    // No blocking wait for connection!
    // We check `usb_serial.dtr()` before writing later.

    // 3. CONFIGURATION I2C (Baromètre)
    // D'après le dump: SCL = PB8, SDA = PB9
    // DMA Conflict Resolved: I2C1_TX on Stream 7 (CH7), USART2_TX on Stream 6 (CH6)
    let mut i2c = I2c::new(
        p.I2C1,
        p.PB8,
        p.PB9,
        Irqs,
        p.DMA1_CH7, // Changed from CH6 to CH7 to free Stream 6 for USART2
        p.DMA1_CH0,
        TimeHertz(100_000),
        Default::default(),
    );

    let mut baro = Spl06::new();
    let mut mag = Hmc5883::new();

    // D'après le dump: I2C1 (Shared)
    // We clones the i2c if needed? No, Embassy I2c is not Clone.
    // We need to use sharing or just pass it around.
    // In this simple sequential loop, we can just use the same I2C if the driver allows it,
    // but the driver currently owns it.
    // Let's modify main to initialize sensors then pass them back or use a shared I2C.
    // Actually, I2C1 is already created. Let's see if we can use it for both.
    // We'll need to use `embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice` for true sharing.
    // For now, let's assume we can re-use it or similar.
    // WAIT: I noticed SPL06 is initialized with `i2c`.
    // Let's create a shared bus if we want to add more sensors.

    // STARTUP DELAY: Wait for sensor power stabilization
    // Even after USB connect, sensors might need time if USB was plugged instantly with power.
    Timer::after(Duration::from_millis(100)).await;

    // Init Sensors on I2C1
    if let Err(_) = baro.init(&mut i2c).await {
        if usb_serial.dtr() {
            let _ = usb_serial.write_packet(b"Baro Init Failed\r\n").await;
        }
    }
    if let Err(_) = mag.init(&mut i2c).await {
        if usb_serial.dtr() {
            let _ = usb_serial.write_packet(b"Mag Init Failed\r\n").await;
        }
    }

    // 4. CONFIGURATION SPI (Gyro)
    // D'après le dump: SCK=PA5, MISO=PA6, MOSI=PA7, CS=PB12
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = TimeHertz(1_000_000);

    use embassy_stm32::dma::NoDma;
    let spi = Spi::new(
        p.SPI1, p.PA5, // SCK
        p.PA7, // MOSI
        p.PA6, // MISO
        NoDma, // Tx
        NoDma, // Rx
        spi_config,
    );

    // Chip Select pour le Gyro (PB12)
    use embassy_stm32::gpio::Pin;
    let cs_gyro = Output::new(p.PB12.degrade(), Level::High, Speed::VeryHigh);
    let mut imu = Icm42688::new(spi, cs_gyro);

    // ESC TAB MOTOR (Bluejay) on Betaflight MOTOR1 port
    // CLI mapping: resource MOTOR 1 B00
    let dshot_tab_motor = Dshot300::new(p.PB0.degrade());
    spawner.spawn(dshot_tab_task(dshot_tab_motor)).unwrap();

    // 5. CONFIGURATION FLASH (SPI3)
    // SCK=PC10, MISO=PC11, MOSI=PC12, CS=PB3
    let mut spi3_config = SpiConfig::default();
    spi3_config.frequency = TimeHertz(10_000_000); // 10MHz
    let spi3 = Spi::new(p.SPI3, p.PC10, p.PC12, p.PC11, NoDma, NoDma, spi3_config);
    let cs_flash = Output::new(p.PB3.degrade(), Level::High, Speed::VeryHigh);
    let mut flash = W25qxx::new(spi3, cs_flash);

    // Heartbeat LED
    let mut led = Output::new(p.PC13, Level::High, Speed::Low);

    // Check Flash connection
    let mut _flash_id = [0u8; 3];
    if let Ok(id) = flash.read_id().await {
        _flash_id = id;
    }

    if let Err(_) = imu.init().await {
        // En cas d'erreur d'init, on continuera mais l'IMU ne marchera pas
    }

    // 6. CONFIGURATION GPS (USART3) - Walksnail WS-M181
    // TX=PB10, RX=PB11 (Common USART3)
    let mut gps_config = UsartConfig::default();
    gps_config.baudrate = 115200; // WS-M181 default baud

    // USART3 DMA: Rx=DMA1_Stream1 (Ch4), Tx=DMA1_Stream3 (Ch4)
    // Embassy: DMA1_CH1, DMA1_CH3
    let mut gps_uart = Uart::new(
        p.USART3, p.PB11, p.PB10, Irqs, p.DMA1_CH3, // Tx
        p.DMA1_CH1, // Rx
        gps_config,
    )
    .unwrap();

    let mut gps_parser = crate::drivers::gps::NmeaParser::new();

    // --- GPS STARTUP DIAGNOSTIC (Betaflight-style state machine) ---
    // Block for up to 3s to detect baud, count NMEA frames, report errors
    if usb_serial.dtr() {
        let _ = usb_serial.write_packet(b"[GPS] ===== STARTUP DIAG =====\r\n").await;
        let _ = usb_serial.write_packet(b"[GPS] USART3 @ 115200  WS-M181\r\n").await;
    }
    Timer::after(Duration::from_millis(200)).await; // Let GPS module power up

    {
        use embassy_futures::select::{select, Either};
        let mut diag_buf = [0u8; 64];
        let mut total_rx: usize = 0;
        let mut got_data = false;
        let mut uart_errors: u8 = 0;

        // Try 6 read attempts, 500ms each = 3s max
        for attempt in 0..6u8 {
            match select(
                gps_uart.read(&mut diag_buf),
                Timer::after(Duration::from_millis(500)),
            )
            .await
            {
                Either::First(Ok(())) => {
                    total_rx += diag_buf.len();
                    got_data = true;

                    if usb_serial.dtr() {
                        let mut msg = heapless::String::<160>::new();
                        let _ = write!(msg, "[GPS] #{} rx={}B raw: ", attempt, diag_buf.len());
                        for i in 0..diag_buf.len().min(60) {
                            let b = diag_buf[i];
                            if b >= 0x20 && b <= 0x7E {
                                let _ = msg.push(b as char);
                            } else {
                                let _ = write!(msg, "<{:02X}>", b);
                            }
                        }
                        let _ = write!(msg, "\r\n");
                        let _ = usb_serial.write_packet(msg.as_bytes()).await;
                    }

                    gps_parser.push_data(&diag_buf);
                }
                Either::First(Err(_)) => {
                    uart_errors += 1;
                    if usb_serial.dtr() {
                        let mut msg = heapless::String::<64>::new();
                        let _ = write!(msg, "[GPS] #{} UART ERR (total={})\r\n", attempt, uart_errors);
                        let _ = usb_serial.write_packet(msg.as_bytes()).await;
                    }
                }
                Either::Second(_) => {
                    if usb_serial.dtr() {
                        let mut msg = heapless::String::<64>::new();
                        let _ = write!(msg, "[GPS] #{} timeout (0B in 500ms)\r\n", attempt);
                        let _ = usb_serial.write_packet(msg.as_bytes()).await;
                    }
                }
            }
        }

        if usb_serial.dtr() {
            // Summary line 1: byte-level
            let mut m1 = heapless::String::<160>::new();
            let _ = write!(
                m1,
                "[GPS] SUMMARY: {}B rx | uart_err={} | state={:?}\r\n",
                total_rx, uart_errors, gps_parser.data.state
            );
            let _ = usb_serial.write_packet(m1.as_bytes()).await;

            // Summary line 2: NMEA frame counts
            let mut m2 = heapless::String::<160>::new();
            let _ = write!(
                m2,
                "[GPS] NMEA: total={} GGA={} RMC={} GSA={} GSV={} unk={} ckerr={} frerr={}\r\n",
                gps_parser.data.sentences_rx,
                gps_parser.data.gga_count,
                gps_parser.data.rmc_count,
                gps_parser.data.gsa_count,
                gps_parser.data.gsv_count,
                gps_parser.data.unknown_count,
                gps_parser.data.checksum_errors,
                gps_parser.data.frame_errors,
            );
            let _ = usb_serial.write_packet(m2.as_bytes()).await;

            // Summary line 3: fix / sats
            let mut m3 = heapless::String::<128>::new();
            if got_data {
                let _ = write!(
                    m3,
                    "[GPS] fix={} q={} sats={} view={} hdop={:.1}\r\n",
                    gps_parser.data.fix,
                    gps_parser.data.fix_quality,
                    gps_parser.data.sats,
                    gps_parser.data.sats_in_view,
                    gps_parser.data.hdop,
                );
            } else {
                let _ = write!(
                    m3,
                    "[GPS] FAIL: 0 bytes in 3s. Check wiring TX->RX, power, and baud.\r\n"
                );
            }
            let _ = usb_serial.write_packet(m3.as_bytes()).await;

            // Summary line 4: position if available
            if gps_parser.data.fix {
                let mut m4 = heapless::String::<128>::new();
                let _ = write!(
                    m4,
                    "[GPS] lat={:.7} lon={:.7} alt={:.1}m spd={:.1}kts\r\n",
                    gps_parser.data.lat,
                    gps_parser.data.lon,
                    gps_parser.data.alt,
                    gps_parser.data.speed,
                );
                let _ = usb_serial.write_packet(m4.as_bytes()).await;
            }

            let _ = usb_serial.write_packet(b"[GPS] ===== END DIAG =====\r\n").await;
        }
    }

    // 7. CONFIGURATION CRSF/ELRS (UART4)
    // TX=PA0, RX=PA1 (Alternative UART4 on F405)
    let mut crsf_config = UsartConfig::default();
    crsf_config.baudrate = 420000;

    // UART4 DMA: Rx=DMA1_Stream2 (Ch4), Tx=DMA1_Stream4 (Ch4)
    // Embassy: DMA1_CH2, DMA1_CH4
    let mut crsf_uart = Uart::new(
        p.UART4,
        p.PA1,
        p.PA0,
        Irqs,
        p.DMA1_CH4, // TX DMA
        p.DMA1_CH2, // RX DMA
        crsf_config,
    )
    .unwrap();

    let mut crsf_parser = crate::drivers::crsf::CrsfParser::new();

    // 8. CONFIGURATION ADC (Battery) - PC3 (ADC1_IN13)
    use embassy_stm32::adc::Adc;
    let mut delay = embassy_time::Delay;
    let mut adc = Adc::new(p.ADC1, &mut delay);
    let mut vbat_pin = p.PC3;

    // 5. Init Algorithms
    let mut ahrs = Mahony::new(20.0);
    let mut kalman = VerticalKalman::new();
    let mut roll_ctrl = RollController::new(4.0, 0.8, 0.08, 0.4, 1.0);
    let mut tab_gear_ctrl = GearedTabController::new(0.015, 0.002, 20.0, 1.0, 360.0);

    // 6. Init Filters
    let mut baro_lpf = LowPassFilter::new(0.2); // Alpha 0.2 for barometer
    let mut accel_lpf_z = LowPassFilter::new(0.5); // Old LPF for Kalman Z

    // Advanced IMU Filters (Betaflight style)
    // Assuming loop runs at ~100Hz (Timer::after(Duration::from_millis(10)) in main loop? No, currently 50ms select delay)
    // Actually the outer-outer loop has Timer::after(Duration::from_millis(1000))...
    // Wait, the main loop is `loop { led.toggle(); Timer::after(Duration::from_millis(1000)).await; inner_loop { ... Timer::after(Duration::from_millis(50)).await; } }`
    // So the loop rate is approx 20Hz.
    let loop_freq = 20.0;
    const LOOP_DT: f32 = 1.0 / 20.0;
    const ROLL_MAX_DEG: f32 = 35.0;

    let mut gyro_filt = [
        Pt1Filter::new(10.0, loop_freq), // 10Hz LPF for Gyro at 20Hz sampling
        Pt1Filter::new(10.0, loop_freq),
        Pt1Filter::new(10.0, loop_freq),
    ];
    let mut accel_filt = [
        BiquadFilter::new_lpf(5.0, loop_freq, 0.707), // 5Hz Biquad for Accel
        BiquadFilter::new_lpf(5.0, loop_freq, 0.707),
        BiquadFilter::new_lpf(5.0, loop_freq, 0.707),
    ];

    // Calibrate Barometer ground altitude (basic)
    let mut ground_alt = 0.0;
    // Average 10 samples for ground altitude
    for _ in 0..10 {
        if let Ok((alt, _, _)) = baro.read_pressure_altitude(&mut i2c).await {
            ground_alt += alt;
        }
        Timer::after(Duration::from_millis(50)).await;
    }
    ground_alt /= 10.0;

    // --- CALIBRATION PHASE (Gyro/Accel) ---
    if usb_serial.dtr() {
        let _ = usb_serial
            .write_packet(b"Calibrating... Keep Still!\r\n")
            .await;
    }
    let mut gyro_bias = [0.0f32; 3];
    let mut accel_bias = [0.0f32; 3];
    const CALIB_SAMPLES: usize = 100;

    for _ in 0..CALIB_SAMPLES {
        if let Ok((accel, gyro)) = imu.read_all().await {
            accel_bias[0] += accel[0] as f32;
            accel_bias[1] += accel[1] as f32;
            accel_bias[2] += accel[2] as f32;

            gyro_bias[0] += gyro[0] as f32;
            gyro_bias[1] += gyro[1] as f32;
            gyro_bias[2] += gyro[2] as f32;
        }
        Timer::after(Duration::from_millis(10)).await;
    }

    // Average
    for i in 0..3 {
        accel_bias[i] /= CALIB_SAMPLES as f32;
        gyro_bias[i] /= CALIB_SAMPLES as f32;
    }

    // Accel Z should be 1G (2048 LSB) at rest, so we want bias to be (Measured - 2048)
    // Actually simplicity: We subtract bias from raw, so expected raw is bias + 2048.
    // If we average 2100, bias is 2100. Then Raw - Bias = 0.
    // But we want Z=1G. So we adjust Accel Z bias to be Average - 2048.
    accel_bias[2] -= 2048.0;

    if usb_serial.dtr() {
        let _ = usb_serial.write_packet(b"Calibration Done!\r\n").await;
    }

    // 5. BOUCLE PRINCIPALE
    loop {
        // Blink LED to show we are in outer loop
        led.toggle();

        // Retrieve connection check? No, just wait a bit to avoid spin lock
        Timer::after(Duration::from_millis(1000)).await;

        let mut crsf_telemetry_tick: u32 = 0;
        let mut usb_debug_tick: u32 = 0;
        let mut rc_channels = [0u16; 16]; // Channels persist between loops

        loop {
            // A. Lecture SPI (Gyro) via Driver

            let (accel, gyro) = match imu.read_all().await {
                Ok(res) => res,
                Err(_) => ([0, 0, 0], [0, 0, 0]),
            };

            // Apply Calibration
            let ax_raw = accel[0] as f32 - accel_bias[0];
            let ay_raw = accel[1] as f32 - accel_bias[1];
            let az_raw = accel[2] as f32 - accel_bias[2];

            let gx_raw = gyro[0] as f32 - gyro_bias[0];
            let gy_raw = gyro[1] as f32 - gyro_bias[1];
            let gz_raw = gyro[2] as f32 - gyro_bias[2];

            // Apply Advanced Filtering (Betaflight/iNav style)
            let ax_f = accel_filt[0].filter(ax_raw);
            let ay_f = accel_filt[1].filter(ay_raw);
            let az_f = accel_filt[2].filter(az_raw);

            let gx_f = gyro_filt[0].filter(gx_raw);
            let gy_f = gyro_filt[1].filter(gy_raw);
            let gz_f = gyro_filt[2].filter(gz_raw);

            // B. Lecture I2C
            // B. Lecture I2C (Baro SPL06)

            // Note: Driver 'Spl06' doesn't have read_pressure yet!
            // We need to implement it. For now, we simulate or just run AHRS.

            // --- AHRS UPDATE ---
            // Unit conversion (approx): Accel RAW / 2048 = G, Gyro RAW / 16.4 = deg/s -> rad/s
            // We need proper scale factors from IMU config.
            // Assuming default 16G range -> 2048 LSB/g ?
            // Default 2000dps -> 16.4 LSB/dps

            let ax_g = ax_f / 2048.0;
            let ay_g = ay_f / 2048.0;
            let az_g = az_f / 2048.0;

            let gx_rad = (gx_f / 16.4).to_radians();
            let gy_rad = (gy_f / 16.4).to_radians();
            let gz_rad = (gz_f / 16.4).to_radians();

            // Read Magnetometer
            let (mx, my, mz) = match mag.read_mag(&mut i2c).await {
                Ok(m) => (m[0] as f32, m[1] as f32, m[2] as f32),
                Err(_) => (0.0, 0.0, 0.0),
            };

            ahrs.update_9dof(gx_rad, gy_rad, gz_rad, ax_g, ay_g, az_g, mx, my, mz);

            // Rotate accel to Earth frame and remove gravity
            let (_, _, az_earth) = ahrs.rotate_vector(ax_g, ay_g, az_g);
            let az_lin = az_earth - 1.0; // Subtract 1G gravity
            let az_lin_ms2 = az_lin * 9.81;

            // Apply LPF to vertical acceleration
            let az_filtered = accel_lpf_z.filter(az_lin_ms2);

            // --- KALMAN UPDATE ---
            // Predict
            kalman.predict(LOOP_DT, az_filtered);

            // Update (Correction)
            // Feed Relative Altitude (AGL) to Kalman
            let mut pressure_hpa = 0.0;
            let mut temp_c = 0.0;

            match baro.read_pressure_altitude(&mut i2c).await {
                Ok((alt_m, press_pa, temp)) => {
                    let raw_baro_alt = alt_m - ground_alt;
                    pressure_hpa = press_pa / 100.0;
                    temp_c = temp;
                    let filtered_baro_alt = baro_lpf.filter(raw_baro_alt);
                    kalman.update(filtered_baro_alt);
                }
                Err(_) => {
                    // let _ = raw_baro_alt;
                }
            }

            let k_state = kalman.state();

            // Telemetry Packet Construction
            // @T<temp>P<press>A<alt>v<vel>l<lon>L<lat>V<volt>S<sats>x<gx>y<gy>z<gz>X<ax>Y<ay>Z<az>s<rssi>

            // Stubs / Conversions
            let t = temp_c;
            let p = pressure_hpa;
            let a = k_state.position;
            let v = k_state.velocity;

            // Gyro (dps) and Accel (m/s2)
            let gx_dps = gx_raw / 16.4;
            let gy_dps = gy_raw / 16.4;
            let gz_dps = gz_raw / 16.4;

            let ax_ms2 = ax_g * 9.81;
            let ay_ms2 = ay_g * 9.81;
            let az_ms2 = az_g * 9.81;

            let rssi = -49;

            // Poll GPS — drain UART buffer fully each cycle
            // At 115200 baud ≈ 11520 B/s → ~576B per 50ms cycle
            // We loop with short timeouts to grab everything available
            use embassy_futures::select::{select, Either};

            let mut gps_buf = [0u8; 128]; // Sized to hold 1-2 full NMEA sentences
            let mut gps_rx_bytes: usize = 0;
            let mut gps_rx_err = false;

            // Read up to 8 chunks (8 × 128 = 1024B max, enough for a full NMEA cycle)
            for _drain in 0..8u8 {
                match select(
                    gps_uart.read(&mut gps_buf),
                    Timer::after(Duration::from_millis(5)),
                )
                .await
                {
                    Either::First(Ok(())) => {
                        gps_rx_bytes += gps_buf.len();
                        gps_parser.push_data(&gps_buf);
                    }
                    Either::First(Err(_)) => {
                        gps_rx_err = true;
                        break;
                    }
                    Either::Second(_) => {
                        // No more data waiting — UART drained
                        break;
                    }
                }
            }

            // Update GPS state machine & timing (Betaflight-style)
            let now_ms = embassy_time::Instant::now().as_millis() as u32;
            gps_parser.data.last_byte_ms = now_ms;
            gps_parser.update_timing(now_ms, gps_rx_bytes);

            // Poll CRSF
            // let mut rc_channels = [0u16; 16]; // REMOVED: Do not reset every loop
            let mut buf_crsf = [0u8; 64];
            match select(
                crsf_uart.read(&mut buf_crsf),
                Timer::after(Duration::from_ticks(0)),
            )
            .await
            {
                Either::First(Ok(())) => {
                    if let Some(parsed) = crsf_parser.push_bytes(&buf_crsf) {
                        rc_channels = parsed.channels;
                    }
                }
                Either::First(Err(_)) => {}
                Either::Second(_) => {}
            }

            let lon = gps_parser.data.lon;
            let lat = gps_parser.data.lat;
            let sats = gps_parser.data.sats;
            let throttle = rc_channels[2]; // CH2 is often Throttle in AETR (or CH0 in TAER). ELRS is usually AETR (Roll, Pitch, Thr, Yaw).
                                           // Actually, wait: CRSF over ELRS usually sends AETR by default or matches handset.
                                           // Let's assume CH2 is Throttle for now.

            let roll_stick = crsf_to_unit(rc_channels[0]);
            let throttle_unit = ((rc_channels[2] as f32 - 172.0) / (1811.0 - 172.0)).clamp(0.0, 1.0);
            let armed = rc_channels[4] > 1200;
            let gear_ratio = GearRatio::from_aux_channel(rc_channels[5]);
            let (roll_measured_rad, _, _) = ahrs.get_euler_angles();
            let roll_setpoint_rad = max_roll_setpoint_from_stick(roll_stick, ROLL_MAX_DEG);

            let tab_cmd_roll = if armed {
                roll_ctrl.update(
                    LOOP_DT,
                    roll_setpoint_rad,
                    roll_measured_rad,
                    gx_rad,
                )
            } else {
                roll_ctrl.reset();
                0.0
            };

            let motor_throttle = if armed { throttle_unit } else { 0.0 };
            let esc_cmd_dshot = unit_to_dshot(motor_throttle, armed);

            let tab_target_deg = roll_output_to_tab_target_deg(tab_cmd_roll, 20.0);
            let (tab_est_deg, tab_motor_cmd_signed) = if armed {
                tab_gear_ctrl.update(LOOP_DT, tab_target_deg, gear_ratio)
            } else {
                tab_gear_ctrl.reset();
                (0.0, 0.0)
            };

            let tab_motor_dir = if tab_motor_cmd_signed >= 0.0 { 1 } else { 0 };
            let tab_motor_dshot = if ESC_OUTPUT_LOCKED {
                0
            } else {
                signed_unit_to_dshot_3d(tab_motor_cmd_signed, armed)
            };
            TAB_MOTOR_DSHOT_CMD.store(tab_motor_dshot, Ordering::Relaxed);

            // Read Voltage (every USB cycle approx 25Hz)
            // ADC is 12-bit (0-4095). Ref 3.3V. Scale 11.0 (110 from dump)
            let v_raw = adc.read(&mut vbat_pin);
            let v_volt = (v_raw as f32 / 4095.0) * 3.3 * 11.0;

            // --- USB DEBUG OUTPUT (spread across ticks to reduce spam) ---
            usb_debug_tick = usb_debug_tick.wrapping_add(1);

            if usb_serial.dtr() {
                // ══════ GPS DEBUG (every 10 ticks ≈ 0.5s) — Betaflight-level detail ══════
                if usb_debug_tick % 10 == 0 {
                    let gd = &gps_parser.data;

                    // Line 1: State machine + UART health
                    let mut m1 = heapless::String::<160>::new();
                    if write!(
                        m1,
                        "[GPS] state={:?} rx={}B err={} timeouts={}\r\n",
                        gd.state, gps_rx_bytes, gps_rx_err, gd.timeouts
                    ).is_ok() {
                        let _ = usb_serial.write_packet(m1.as_bytes()).await;
                    }

                    // Line 2: NMEA frame counters (Betaflight DEBUG_GPS_CONNECTION style)
                    let mut m2 = heapless::String::<192>::new();
                    if write!(
                        m2,
                        "[GPS] nmea={} GGA={} RMC={} GSA={} GSV={} unk={} ckerr={} frerr={}\r\n",
                        gd.sentences_rx,
                        gd.gga_count,
                        gd.rmc_count,
                        gd.gsa_count,
                        gd.gsv_count,
                        gd.unknown_count,
                        gd.checksum_errors,
                        gd.frame_errors,
                    ).is_ok() {
                        let _ = usb_serial.write_packet(m2.as_bytes()).await;
                    }

                    // Line 3: Fix + satellites + DOP (Betaflight DEBUG_GPS_DOP)
                    let mut m3 = heapless::String::<160>::new();
                    if write!(
                        m3,
                        "[GPS] fix={} q={} sats={} view={} pdop={} hdop={} vdop={} navInt={}ms\r\n",
                        gd.fix,
                        gd.fix_quality,
                        gd.sats,
                        gd.sats_in_view,
                        gd.pdop_i,
                        gd.hdop_i,
                        gd.vdop_i,
                        gd.nav_interval_ms,
                    ).is_ok() {
                        let _ = usb_serial.write_packet(m3.as_bytes()).await;
                    }

                    // Line 4: Position + alt
                    let mut m4 = heapless::String::<160>::new();
                    if write!(
                        m4,
                        "[GPS] lat={:.7} lon={:.7} alt={:.1}m\r\n",
                        gd.lat, gd.lon, gd.alt
                    ).is_ok() {
                        let _ = usb_serial.write_packet(m4.as_bytes()).await;
                    }

                    // Line 5: Speed + course + UTC
                    let mut m5 = heapless::String::<160>::new();
                    if write!(
                        m5,
                        "[GPS] spd={:.1}kts ({} cm/s) crs={:.1}d gcrs={} utc={} date={}\r\n",
                        gd.speed,
                        gd.speed_cms,
                        gd.course,
                        gd.ground_course,
                        gd.utc_time,
                        gd.utc_date,
                    ).is_ok() {
                        let _ = usb_serial.write_packet(m5.as_bytes()).await;
                    }

                    // Line 6: Active PRNs (from GSA)
                    let mut m6 = heapless::String::<128>::new();
                    let _ = write!(m6, "[GPS] PRN({}):", gd.active_count);
                    for i in 0..gd.active_count {
                        let _ = write!(m6, " {}", gd.active_ids[i as usize]);
                    }
                    let _ = write!(m6, "\r\n");
                    let _ = usb_serial.write_packet(m6.as_bytes()).await;
                }

                // Per-satellite SNR table (every 50 ticks ≈ 2.5s, like Betaflight GSV/svinfo)
                if usb_debug_tick % 50 == 0 && gps_parser.data.sv_count > 0 {
                    let gd = &gps_parser.data;
                    let mut ms = heapless::String::<256>::new();
                    let _ = write!(ms, "[GPS] SV({}):", gd.sv_count);
                    for i in 0..gd.sv_count.min(crate::drivers::gps::GPS_SV_MAXSATS as u8) {
                        let sv = &gd.svinfo[i as usize];
                        if sv.svid > 0 {
                            // PRN:SNR format
                            let _ = write!(ms, " {}:{}", sv.svid, sv.cno);
                        }
                    }
                    let _ = write!(ms, "\r\n");
                    let _ = usb_serial.write_packet(ms.as_bytes()).await;
                }

                // Raw byte count per tick (no longer dumping partial buffer)
                if gps_rx_bytes > 0 && usb_debug_tick % 10 == 5 {
                    let mut msg_raw = heapless::String::<64>::new();
                    let _ = write!(msg_raw, "[GPS] drain={}B this tick\r\n", gps_rx_bytes);
                    let _ = usb_serial.write_packet(msg_raw.as_bytes()).await;
                }
            }

            // Toggle LED every cycle (20Hz blink)
            led.toggle();

            // --- CRSF Telemetry ---
            crsf_telemetry_tick = crsf_telemetry_tick.wrapping_add(1);
            let mut pkt_buf = [0u8; 64];

            let pkt_len = if crsf_telemetry_tick % 30 == 2 {
                // Battery (1Hz approx)
                // CRSF: Voltage (0.1V step)
                let v_u16 = (v_volt * 10.0) as u16;
                let payload = crate::drivers::crsf::payload_battery(v_u16, 0, 0, 0);
                crate::drivers::crsf::build_telemetry_packet(
                    &mut pkt_buf,
                    crate::drivers::crsf::CRSF_FRAMETYPE_BATTERY_SENSOR,
                    &payload,
                )
            } else if crsf_telemetry_tick % 5 == 0 {
                // GPS (6Hz approx)
                let lat = (gps_parser.data.lat * 10_000_000.0) as i32;
                let lon = (gps_parser.data.lon * 10_000_000.0) as i32;

                // Speed (Knots -> km/h * 10)
                let gspd = (gps_parser.data.speed * 1.852 * 10.0) as u16;

                // Heading (Deg -> Deg * 100)
                let hdg = (gps_parser.data.course * 100.0) as u16;

                // CRSF Alt: m + 1000
                let gps_alt = (gps_parser.data.alt + 1000.0) as u16;
                let sats = gps_parser.data.sats;

                let payload = crate::drivers::crsf::payload_gps(lat, lon, gspd, hdg, gps_alt, sats);
                crate::drivers::crsf::build_telemetry_packet(
                    &mut pkt_buf,
                    crate::drivers::crsf::CRSF_FRAMETYPE_GPS,
                    &payload,
                )
            } else if crsf_telemetry_tick % 5 == 1 {
                // Attitude (6Hz approx)
                let (roll, pitch, yaw) = ahrs.get_euler_angles();
                let payload = crate::drivers::crsf::payload_attitude(
                    (pitch * 10000.0) as i16,
                    (roll * 10000.0) as i16,
                    (yaw * 10000.0) as i16,
                );
                crate::drivers::crsf::build_telemetry_packet(
                    &mut pkt_buf,
                    crate::drivers::crsf::CRSF_FRAMETYPE_ATTITUDE,
                    &payload,
                )
            } else if crsf_telemetry_tick % 5 == 3 {
                // Vario (Wait for next tick)
                // Vario 0x09: Altitude (decimeters + 10000), VSpeed (cm/s).
                // Use Kalman Position and Velocity
                let k_pos = k_state.position; // in meters
                let k_vel = k_state.velocity; // in m/s

                // Alt (dm) + 10000. e.g. 0m -> 10000.
                let alt_dm = (k_pos * 10.0) as i32 + 10000;
                let alt_u16 = if alt_dm < 0 {
                    0
                } else if alt_dm > 65535 {
                    65535
                } else {
                    alt_dm as u16
                };

                // Vertical Speed (cm/s)
                let vspd_cms = (k_vel * 100.0) as i16;

                let payload = crate::drivers::crsf::payload_vario(alt_u16, vspd_cms);
                crate::drivers::crsf::build_telemetry_packet(
                    &mut pkt_buf,
                    crate::drivers::crsf::CRSF_FRAMETYPE_VARIO,
                    &payload,
                )
            } else if crsf_telemetry_tick % 30 == 4 {
                // Barometer 0x11 (1Hz approx)
                // Pressure (Pa), Temp (0.01 C)
                let temp_centi = (temp_c * 100.0) as i16;
                let press_pa_u32 = (pressure_hpa * 100.0) as u32;

                let payload = crate::drivers::crsf::payload_barometer(press_pa_u32, temp_centi);
                crate::drivers::crsf::build_telemetry_packet(
                    &mut pkt_buf,
                    crate::drivers::crsf::CRSF_FRAMETYPE_BAROMETRIC_SENSORS,
                    &payload,
                )
            } else if crsf_telemetry_tick % 30 == 29 {
                // Flight Mode (1Hz approx)
                // Cycle between "ROCKET" and Satellite IDs
                // Every 2 cycles (approx 2s period each? No, this runs at 1Hz)
                // Let's toggle every time.

                // Use a static counter or derive from tick
                // tick increments by 1 every 50ms.
                // This block runs when tick % 30 == 29 (every 1.5s)

                let toggle = (crsf_telemetry_tick / 30) % 2 == 0;

                if toggle {
                    let payload = crate::drivers::crsf::payload_flight_mode("ROCKET");
                    crate::drivers::crsf::build_telemetry_packet(
                        &mut pkt_buf,
                        crate::drivers::crsf::CRSF_FRAMETYPE_FLIGHT_MODE,
                        &payload,
                    )
                } else {
                    // Build satellite string: "Sats: 1 5 12"
                    let mut s = heapless::String::<32>::new();
                    if write!(s, "S:").is_ok() {
                        for i in 0..gps_parser.data.active_count {
                            let id = gps_parser.data.active_ids[i as usize];
                            if write!(s, " {}", id).is_err() {
                                break;
                            }
                        }
                    }
                    let payload = crate::drivers::crsf::payload_flight_mode(s.as_str());
                    crate::drivers::crsf::build_telemetry_packet(
                        &mut pkt_buf,
                        crate::drivers::crsf::CRSF_FRAMETYPE_FLIGHT_MODE,
                        &payload,
                    )
                }
            } else {
                0
            };

            if pkt_len > 0 {
                let _ = crsf_uart.write(&pkt_buf[0..pkt_len]).await;
            }

            Timer::after(Duration::from_millis(50)).await;
        }
    }
}
