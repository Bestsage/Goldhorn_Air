use core::fmt::Write;

use embassy_executor::task;
use embassy_stm32::peripherals::{DMA1_CH4, UART4};
use embassy_stm32::usart::UartTx;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Receiver;
use embassy_time::{Duration, Ticker};

use crate::state::{AttitudeState, BaroData, GpsData};
use crate::usb::UsbSerial;

const USB_DEBUG_ENABLED: bool = true;

/// Telemetry task — 20 Hz.
/// Receives attitude from fast_loop and slow sensor data via channels.
/// Sends CRSF telemetry frames and USB debug lines.
#[task]
pub async fn telemetry_task(
    mut crsf_tx: UartTx<'static, UART4, DMA1_CH4>,
    mut usb_serial: UsbSerial<'static>,
    attitude_rx: Receiver<'static, CriticalSectionRawMutex, AttitudeState, 1>,
    gps_rx: Receiver<'static, CriticalSectionRawMutex, GpsData, 1>,
    baro_rx: Receiver<'static, CriticalSectionRawMutex, BaroData, 1>,
) {
    let mut tick: u32 = 0;

    // Local cached data
    let mut attitude = AttitudeState::default();
    let mut gps = GpsData::default();
    let mut baro = BaroData::default();

    let mut ticker = Ticker::every(Duration::from_hz(20));

    loop {
        ticker.next().await;
        tick = tick.wrapping_add(1);

        // Refresh from channels (non-blocking)
        if let Ok(a) = attitude_rx.try_receive() { attitude = a; }
        if let Ok(g) = gps_rx.try_receive()      { gps = g; }
        if let Ok(b) = baro_rx.try_receive()      { baro = b; }

        // ── USB Debug (every 10 ticks = 0.5s) ────────────────────────────────
        if USB_DEBUG_ENABLED && usb_serial.dtr() && tick % 10 == 0 {
            let roll_deg  = attitude.roll_rad.to_degrees();
            let pitch_deg = attitude.pitch_rad.to_degrees();
            let yaw_deg   = attitude.yaw_rad.to_degrees();

            let mut m = heapless::String::<128>::new();
            let _ = write!(m,
                "[ATT] r={:.1} p={:.1} y={:.1} hg={} alt={:.1}m v={:.2}m/s\r\n",
                roll_deg, pitch_deg, yaw_deg,
                attitude.is_high_g as u8,
                attitude.alt_m, attitude.vel_ms
            );
            let _ = usb_serial.write_packet(m.as_bytes()).await;

            let mut m = heapless::String::<128>::new();
            let _ = write!(m,
                "[GPS] fix={} s={} lat={:.6} lon={:.6} alt={:.0}m\r\n",
                gps.fix as u8, gps.sats, gps.lat, gps.lon, gps.alt
            );
            let _ = usb_serial.write_packet(m.as_bytes()).await;

            let mut m = heapless::String::<64>::new();
            let _ = write!(m,
                "[BARO] {:.1}hPa {:.1}m {:.1}C\r\n",
                baro.pressure_hpa, baro.alt_m, baro.temp_c
            );
            let _ = usb_serial.write_packet(m.as_bytes()).await;
        }

        // ── CRSF Telemetry ─────────────────────────────────────────────────
        let mut pkt_buf = [0u8; 64];
        let pkt_len = if tick % 20 == 2 {
            // Battery placeholder — no ADC here; extend later
            crate::drivers::crsf::build_telemetry_packet(
                &mut pkt_buf,
                crate::drivers::crsf::CRSF_FRAMETYPE_BATTERY_SENSOR,
                &crate::drivers::crsf::payload_battery(0, 0, 0, 0),
            )
        } else if tick % 4 == 0 {
            // GPS ~5 Hz
            let lat_i = (gps.lat * 10_000_000.0) as i32;
            let lon_i = (gps.lon * 10_000_000.0) as i32;
            let spd_u = (gps.speed_kts * 1.852 * 10.0) as u16;
            let hdg_u = (gps.course_deg * 100.0) as u16;
            let alt_u = (gps.alt + 1000.0).max(0.0) as u16;
            crate::drivers::crsf::build_telemetry_packet(
                &mut pkt_buf,
                crate::drivers::crsf::CRSF_FRAMETYPE_GPS,
                &crate::drivers::crsf::payload_gps(lat_i, lon_i, spd_u, hdg_u, alt_u, gps.sats),
            )
        } else if tick % 4 == 1 {
            // Attitude ~5 Hz
            crate::drivers::crsf::build_telemetry_packet(
                &mut pkt_buf,
                crate::drivers::crsf::CRSF_FRAMETYPE_ATTITUDE,
                &crate::drivers::crsf::payload_attitude(
                    (attitude.pitch_rad * 10000.0) as i16,
                    (attitude.roll_rad  * 10000.0) as i16,
                    (attitude.yaw_rad   * 10000.0) as i16,
                ),
            )
        } else if tick % 4 == 2 {
            // Vario ~5 Hz
            let alt_dm = (attitude.alt_m * 10.0) as i32 + 10000;
            let alt_u  = alt_dm.clamp(0, 65535) as u16;
            let vspd   = (attitude.vel_ms * 100.0) as i16;
            crate::drivers::crsf::build_telemetry_packet(
                &mut pkt_buf,
                crate::drivers::crsf::CRSF_FRAMETYPE_VARIO,
                &crate::drivers::crsf::payload_vario(alt_u, vspd),
            )
        } else if tick % 20 == 4 {
            // Barometer ~1 Hz
            let temp_centi = (baro.temp_c * 100.0) as i16;
            let press_pa   = (baro.pressure_hpa * 100.0) as u32;
            crate::drivers::crsf::build_telemetry_packet(
                &mut pkt_buf,
                crate::drivers::crsf::CRSF_FRAMETYPE_BAROMETRIC_SENSORS,
                &crate::drivers::crsf::payload_barometer(press_pa, temp_centi),
            )
        } else if tick % 20 == 6 {
            // Flight mode ~1 Hz
            let mode_str = if attitude.is_high_g { "BOOST" } else { "COAST" };
            crate::drivers::crsf::build_telemetry_packet(
                &mut pkt_buf,
                crate::drivers::crsf::CRSF_FRAMETYPE_FLIGHT_MODE,
                &crate::drivers::crsf::payload_flight_mode(mode_str),
            )
        } else {
            0
        };

        if pkt_len > 0 {
            let _ = crsf_tx.write(&pkt_buf[..pkt_len]).await;
        }
    }
}
