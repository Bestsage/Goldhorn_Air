/// Shared state types for inter-task communication via Embassy channels.
///
/// All types are `Copy` to minimise overhead when sent through channels.

// ── Data types ────────────────────────────────────────────────────────────────

#[derive(Clone, Copy, Default)]
pub struct BaroData {
    pub alt_m: f32,
    pub pressure_hpa: f32,
    pub temp_c: f32,
}

#[derive(Clone, Copy, Default)]
pub struct GpsData {
    pub lat: f32,
    pub lon: f32,
    pub alt: f32,
    pub sats: u8,
    pub fix: bool,
    pub speed_kts: f32,
    pub course_deg: f32,
}

#[derive(Clone, Copy)]
pub struct RcData {
    pub channels: [u16; 16],
}

impl Default for RcData {
    fn default() -> Self {
        Self { channels: [0u16; 16] }
    }
}

/// Shared EKF state readable by the telemetry task (written only by fast_loop).
/// Protected by a mutex, but since fast_loop is the only writer and telemetry
/// only reads, using an AtomicCell pattern is acceptable (we'll use a signal).
#[derive(Clone, Copy, Default)]
pub struct AttitudeState {
    pub roll_rad: f32,
    pub pitch_rad: f32,
    pub yaw_rad: f32,
    pub alt_m: f32,
    pub vel_ms: f32,
    pub is_high_g: bool,
}
