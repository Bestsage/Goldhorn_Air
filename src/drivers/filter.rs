use micromath::F32Ext;

/// Biquad Filter (Second order, Direct Form 2 Transpose)
/// Supports Low-Pass and Notch (Band-Stop) configurations.
pub struct BiquadFilter {
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    z1: f32,
    z2: f32,
    initialized: bool,
}

impl BiquadFilter {
    /// Low-pass Biquad filter.
    /// - `cutoff_freq` : cutoff frequency in Hz
    /// - `sample_rate` : sample rate in Hz
    /// - `q`           : quality factor (0.707 = Butterworth / critically damped)
    pub fn new_lpf(cutoff_freq: f32, sample_rate: f32, q: f32) -> Self {
        let omega = 2.0 * core::f32::consts::PI * cutoff_freq / sample_rate;
        let sn = omega.sin();
        let cs = omega.cos();
        let alpha = sn / (2.0 * q);

        let b0 = (1.0 - cs) / 2.0;
        let b1 = 1.0 - cs;
        let b2 = (1.0 - cs) / 2.0;
        let a0 = 1.0 + alpha;
        let a1 = -2.0 * cs;
        let a2 = 1.0 - alpha;

        Self {
            b0: b0 / a0,
            b1: b1 / a0,
            b2: b2 / a0,
            a1: a1 / a0,
            a2: a2 / a0,
            z1: 0.0,
            z2: 0.0,
            initialized: false,
        }
    }

    /// Notch (Band-Stop) Biquad filter.
    /// Attenuates a specific frequency (e.g. structural resonance of the rocket).
    /// - `notch_freq`  : center frequency to attenuate, in Hz
    /// - `sample_rate` : sample rate in Hz
    /// - `q`           : quality factor — higher Q = narrower notch (typical: 5–20)
    pub fn new_notch(notch_freq: f32, sample_rate: f32, q: f32) -> Self {
        let omega = 2.0 * core::f32::consts::PI * notch_freq / sample_rate;
        let cs = omega.cos();
        let alpha = omega.sin() / (2.0 * q);

        // Notch coefficients
        let b0 = 1.0;
        let b1 = -2.0 * cs;
        let b2 = 1.0;
        let a0 = 1.0 + alpha;
        let a1 = -2.0 * cs;
        let a2 = 1.0 - alpha;

        Self {
            b0: b0 / a0,
            b1: b1 / a0,
            b2: b2 / a0,
            a1: a1 / a0,
            a2: a2 / a0,
            z1: 0.0,
            z2: 0.0,
            initialized: false,
        }
    }

    pub fn filter(&mut self, input: f32) -> f32 {
        if !self.initialized {
            // Initialize state to steady-state for first sample (avoids startup transient)
            self.z1 = input * (1.0 - self.b0);
            self.z2 = input * (self.b0 + self.b1 + self.b2 - 1.0 - self.a1 - self.a2);
            self.initialized = true;
        }

        let output = self.b0 * input + self.z1;
        self.z1 = self.b1 * input - self.a1 * output + self.z2;
        self.z2 = self.b2 * input - self.a2 * output;
        output
    }

    /// Reset filter state (call on re-init or after a gap in data)
    pub fn reset(&mut self) {
        self.z1 = 0.0;
        self.z2 = 0.0;
        self.initialized = false;
    }
}
