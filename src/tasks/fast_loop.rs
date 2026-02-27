use embassy_executor::task;
use embassy_stm32::peripherals::SPI1;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};
use embassy_time::{Duration, Instant, Ticker};

use crate::drivers::ekf::AttitudeEkf;
use crate::drivers::filter::BiquadFilter;
use crate::drivers::icm42688::Icm42688;
use crate::drivers::kalman::VerticalKalman;
use crate::drivers::roll::{
    crsf_to_unit, max_roll_setpoint_from_stick, roll_output_to_tab_target_deg,
    signed_unit_to_dshot_3d, unit_to_dshot, GearRatio, GearedTabController, RollController,
};
use crate::state::{AttitudeState, BaroData, GpsData, RcData};
use crate::TAB_MOTOR_DSHOT_CMD;
use core::sync::atomic::Ordering;

// ── Filter chain constants ────────────────────────────────────────────────────

/// Fast loop target: 1 kHz
const FAST_LOOP_HZ: u64 = 1000;
/// Nominal sample rate for Biquad coefficient pre-computation
const SAMPLE_RATE: f32 = 1000.0;
/// Notch filter center frequency (Hz) — set to dominant rocket body resonance
const NOTCH_FREQ: f32 = 80.0;
/// Notch Q factor (higher = narrower notch)
const NOTCH_Q: f32 = 10.0;
/// Gyro low-pass cutoff (Hz) — post-notch, anti-alias before EKF
const GYRO_LPF_CUTOFF: f32 = 70.0;
/// Gyro LPF Q (Butterworth)
const GYRO_LPF_Q: f32 = 0.707;
/// Accel LPF cutoff (Hz)
const ACCEL_LPF_CUTOFF: f32 = 20.0;

const ESC_OUTPUT_LOCKED: bool = true;
const ROLL_MAX_DEG: f32 = 35.0;

// ── Calibration parameters (filled from main after static calib) ──────────────

pub struct FastLoopConfig {
    pub gyro_bias: [f32; 3],
    pub accel_bias: [f32; 3],
}

// ── Task ─────────────────────────────────────────────────────────────────────

#[task]
pub async fn fast_loop_task(
    mut imu: Icm42688<'static, SPI1>,
    config: FastLoopConfig,
    baro_rx: Receiver<'static, CriticalSectionRawMutex, BaroData, 1>,
    gps_rx: Receiver<'static, CriticalSectionRawMutex, GpsData, 1>,
    crsf_rx: Receiver<'static, CriticalSectionRawMutex, RcData, 1>,
    attitude_tx: Sender<'static, CriticalSectionRawMutex, AttitudeState, 1>,
) {
    // ── Filter instances ──────────────────────────────────────────────────────
    // Notch filter per gyro axis
    let mut notch = [
        BiquadFilter::new_notch(NOTCH_FREQ, SAMPLE_RATE, NOTCH_Q),
        BiquadFilter::new_notch(NOTCH_FREQ, SAMPLE_RATE, NOTCH_Q),
        BiquadFilter::new_notch(NOTCH_FREQ, SAMPLE_RATE, NOTCH_Q),
    ];
    // LPF after notch
    let mut gyro_lpf = [
        BiquadFilter::new_lpf(GYRO_LPF_CUTOFF, SAMPLE_RATE, GYRO_LPF_Q),
        BiquadFilter::new_lpf(GYRO_LPF_CUTOFF, SAMPLE_RATE, GYRO_LPF_Q),
        BiquadFilter::new_lpf(GYRO_LPF_CUTOFF, SAMPLE_RATE, GYRO_LPF_Q),
    ];
    // Accel LPF
    let mut accel_lpf = [
        BiquadFilter::new_lpf(ACCEL_LPF_CUTOFF, SAMPLE_RATE, GYRO_LPF_Q),
        BiquadFilter::new_lpf(ACCEL_LPF_CUTOFF, SAMPLE_RATE, GYRO_LPF_Q),
        BiquadFilter::new_lpf(ACCEL_LPF_CUTOFF, SAMPLE_RATE, GYRO_LPF_Q),
    ];
    // Vertical LPF for accel_z fed into Kalman
    let mut az_lpf = BiquadFilter::new_lpf(10.0, SAMPLE_RATE, GYRO_LPF_Q);

    // ── Estimators ────────────────────────────────────────────────────────────
    let mut ekf = AttitudeEkf::new();
    let mut kalman = VerticalKalman::new();

    // ── Controllers ───────────────────────────────────────────────────────────
    let mut roll_ctrl = RollController::new(4.0, 0.8, 0.08, 0.4, 1.0);
    let mut tab_gear_ctrl = GearedTabController::new(0.015, 0.002, 20.0, 1.0, 360.0);

    // ── Cached slow-loop data (updated from channels when available) ──────────
    let mut baro = BaroData::default();
    let mut gps  = GpsData::default();
    let mut rc   = RcData::default();
    let mut ground_alt = 0.0f32;
    let mut ground_calibrated = false;

    // ── Timing ────────────────────────────────────────────────────────────────
    let mut ticker = Ticker::every(Duration::from_hz(FAST_LOOP_HZ));
    let mut last = Instant::now();

    loop {
        ticker.next().await;

        // Precise dt measurement
        let now = Instant::now();
        let dt = (now - last).as_micros() as f32 / 1_000_000.0;
        let dt = dt.clamp(0.0005, 0.01); // 0.5ms … 10ms guard
        last = now;

        // ── A. Read IMU (SPI @ 10 MHz, non-blocking) ─────────────────────────
        let (accel_raw, gyro_raw) = match imu.read_all().await {
            Ok(v) => v,
            Err(_) => continue, // skip iteration on SPI error
        };

        // ── B. Calibration correction ─────────────────────────────────────────
        let ax_c = accel_raw[0] as f32 - config.accel_bias[0];
        let ay_c = accel_raw[1] as f32 - config.accel_bias[1];
        let az_c = accel_raw[2] as f32 - config.accel_bias[2];

        let gx_c = gyro_raw[0] as f32 - config.gyro_bias[0];
        let gy_c = gyro_raw[1] as f32 - config.gyro_bias[1];
        let gz_c = gyro_raw[2] as f32 - config.gyro_bias[2];

        // ── C. Filter pyramid ─────────────────────────────────────────────────
        // 1) Hardware DLPF ~258 Hz already applied inside ICM42688
        // 2) Software Notch (body resonance)
        let gx_n = notch[0].filter(gx_c);
        let gy_n = notch[1].filter(gy_c);
        let gz_n = notch[2].filter(gz_c);

        // 3) Software Biquad LPF ~70 Hz
        let gx_f = gyro_lpf[0].filter(gx_n);
        let gy_f = gyro_lpf[1].filter(gy_n);
        let gz_f = gyro_lpf[2].filter(gz_n);

        // Accel LPF
        let ax_f = accel_lpf[0].filter(ax_c);
        let ay_f = accel_lpf[1].filter(ay_c);
        let az_f = accel_lpf[2].filter(az_c);

        // ── D. Unit conversion ────────────────────────────────────────────────
        // Gyro: LSB → rad/s  (±2000 dps → 16.4 LSB/dps)
        let gx_rad = (gx_f / 16.4).to_radians();
        let gy_rad = (gy_f / 16.4).to_radians();
        let gz_rad = (gz_f / 16.4).to_radians();

        // Accel: LSB → G  (±16G → 2048 LSB/g)
        let ax_g = ax_f / 2048.0;
        let ay_g = ay_f / 2048.0;
        let az_g = az_f / 2048.0;

        // ── E. EKF predict + update ───────────────────────────────────────────
        ekf.predict(dt, gx_rad, gy_rad, gz_rad);
        ekf.update_accel(ax_g, ay_g, az_g);

        let (roll_rad, pitch_rad, yaw_rad) = ekf.get_euler();

        // ── F. Vertical Kalman (altitude) ─────────────────────────────────────
        // Rotate accel to earth frame for vertical acceleration
        let (_, _, az_earth) = ekf.rotate_to_earth(ax_g, ay_g, az_g);
        let az_lin_ms2 = (az_earth - 1.0) * 9.81; // remove 1G gravity, → m/s²
        let az_filt = az_lpf.filter(az_lin_ms2);
        kalman.predict(dt, az_filt);

        // Check for new baro data
        if let Ok(new_baro) = baro_rx.try_receive() {
            baro = new_baro;
            // Ground calibration on first valid sample
            if !ground_calibrated && baro.alt_m != 0.0 {
                ground_alt = baro.alt_m;
                ground_calibrated = true;
            }
            let agl = (baro.alt_m - ground_alt).max(-500.0); // AGL
            kalman.update(agl);
        }

        let k_state = kalman.state();

        // ── G. Slow data refresh (non-blocking) ───────────────────────────────
        if let Ok(new_gps) = gps_rx.try_receive() {
            gps = new_gps;
        }
        if let Ok(new_rc) = crsf_rx.try_receive() {
            rc = new_rc;
        }

        // ── H. Flight control ─────────────────────────────────────────────────
        let roll_stick   = crsf_to_unit(rc.channels[0]);
        let throttle_unit = ((rc.channels[2] as f32 - 172.0) / (1811.0 - 172.0)).clamp(0.0, 1.0);
        let armed        = rc.channels[4] > 1200;
        let gear_ratio   = GearRatio::from_aux_channel(rc.channels[5]);
        let roll_setpoint = max_roll_setpoint_from_stick(roll_stick, ROLL_MAX_DEG);

        let tab_cmd_roll = if armed {
            roll_ctrl.update(dt, roll_setpoint, roll_rad, gx_rad)
        } else {
            roll_ctrl.reset();
            0.0
        };

        let motor_throttle = if armed { throttle_unit } else { 0.0 };
        let _esc_cmd = unit_to_dshot(motor_throttle, armed);

        let tab_target_deg = roll_output_to_tab_target_deg(tab_cmd_roll, 20.0);
        let (_, tab_motor_cmd_signed) = if armed {
            tab_gear_ctrl.update(dt, tab_target_deg, gear_ratio)
        } else {
            tab_gear_ctrl.reset();
            (0.0, 0.0)
        };

        let tab_motor_dshot = if ESC_OUTPUT_LOCKED {
            0
        } else {
            signed_unit_to_dshot_3d(tab_motor_cmd_signed, armed)
        };
        TAB_MOTOR_DSHOT_CMD.store(tab_motor_dshot, Ordering::Relaxed);

        // ── I. Publish attitude state for telemetry task ──────────────────────
        let state = AttitudeState {
            roll_rad,
            pitch_rad,
            yaw_rad,
            alt_m:   k_state.position,
            vel_ms:  k_state.velocity,
            is_high_g: ekf.debug.is_high_g,
        };
        // Non-blocking send; telemetry task may miss a frame if it's busy
        let _ = attitude_tx.try_send(state);
    }
}
