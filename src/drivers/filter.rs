use micromath::F32Ext;

pub struct LowPassFilter {
    alpha: f32,
    last_output: f32,
    initialized: bool,
}

impl LowPassFilter {
    pub fn new(alpha: f32) -> Self {
        Self {
            alpha: alpha.clamp(0.0, 1.0),
            last_output: 0.0,
            initialized: false,
        }
    }

    pub fn filter(&mut self, input: f32) -> f32 {
        if !self.initialized {
            self.last_output = input;
            self.initialized = true;
        } else {
            self.last_output = self.alpha * input + (1.0 - self.alpha) * self.last_output;
        }
        self.last_output
    }

    pub fn reset(&mut self) {
        self.initialized = false;
    }
}

/// PT1 Filter (First order low pass)
/// Frequency based alpha calculation
pub struct Pt1Filter {
    alpha: f32,
    state: f32,
    initialized: bool,
}

impl Pt1Filter {
    pub fn new(cutoff_freq: f32, sample_rate: f32) -> Self {
        let dt = 1.0 / sample_rate;
        let rc = 1.0 / (2.0 * core::f32::consts::PI * cutoff_freq);
        let alpha = dt / (rc + dt);
        Self {
            alpha: alpha.clamp(0.0, 1.0),
            state: 0.0,
            initialized: false,
        }
    }

    pub fn filter(&mut self, input: f32) -> f32 {
        if !self.initialized {
            self.state = input;
            self.initialized = true;
        } else {
            self.state = self.state + self.alpha * (input - self.state);
        }
        self.state
    }
}

/// Biquad Filter (Second order)
/// Using Direct Form 2 Transpose
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

    pub fn filter(&mut self, input: f32) -> f32 {
        if !self.initialized {
            self.z1 = input * (1.0 - self.b0);
            self.z2 = input * (self.b1 - self.a1); // Simple steady-state init
            self.initialized = true;
        }

        let output = self.b0 * input + self.z1;
        self.z1 = self.b1 * input - self.a1 * output + self.z2;
        self.z2 = self.b2 * input - self.a2 * output;

        output
    }
}
