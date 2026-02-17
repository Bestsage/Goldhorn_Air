#[derive(Default)]
pub struct KalmanState {
    pub position: f32, // Altitude (m)
    pub velocity: f32, // Vertical Velocity (m/s)
}

pub struct VerticalKalman {
    // State vector [pos, vel]
    x: [f32; 2],

    // Covariance matrix P (2x2)
    p: [[f32; 2]; 2],

    // Process noise covariance Q
    q: [f32; 2],

    // Measurement noise covariance R
    #[allow(dead_code)]
    r: f32,
}

impl VerticalKalman {
    pub fn new() -> Self {
        Self {
            x: [0.0, 0.0],
            // Initial uncertainty
            p: [[100.0, 0.0], [0.0, 100.0]],

            // Tunable parameters
            // Q: Process noise (trust in physics model/accelerometer)
            // Higher Q = more trust in measurement, faster response, more noise
            q: [0.01, 0.1],

            // R: Measurement noise (trust in barometer)
            // Higher R = less trust in baro, smoother but laggy
            r: 50.0,
        }
    }

    /// Predict state based on acceleration (model)
    /// dt: time step in seconds
    /// accel_z: vertical acceleration in m/s^2 (Earth frame, gravity removed)
    pub fn predict(&mut self, dt: f32, accel_z: f32) {
        // State transition FMatrix:
        // pos = pos + vel*dt + 0.5*acc*dt^2
        // vel = vel + acc*dt
        let dt2 = 0.5 * dt * dt;

        self.x[0] += self.x[1] * dt + accel_z * dt2;
        self.x[1] += accel_z * dt;

        // Update Covariance P = F*P*F' + Q
        // Simplified algebraic expansion for 2x2
        let p00 = self.p[0][0];
        let p01 = self.p[0][1];
        let p10 = self.p[1][0];
        let p11 = self.p[1][1];

        // F = [[1, dt], [0, 1]]
        // P_new = [[p00 + dt*p10 + dt*(p01 + dt*p11), p01 + dt*p11],
        //          [p10 + dt*p11,                     p11]]
        // + Q

        let p00_new = p00 + dt * (p10 + p01) + dt * dt * p11 + self.q[0];
        let p01_new = p01 + dt * p11;
        let p10_new = p10 + dt * p11;
        let p11_new = p11 + self.q[1];

        self.p = [[p00_new, p01_new], [p10_new, p11_new]];
    }

    /// Update state with measurement (barometer)
    /// meas_alt: measured altitude in meters
    #[allow(dead_code)]
    pub fn update(&mut self, meas_alt: f32) {
        // H = [1, 0] (Measured position only)
        // K = P * H' / (H * P * H' + R)
        // S = P[0][0] + R
        let s = self.p[0][0] + self.r;

        // Kalman Gain K
        let k0 = self.p[0][0] / s;
        let k1 = self.p[1][0] / s;

        // Innovation y = z - Hx
        let y = meas_alt - self.x[0];

        // Update State x = x + Ky
        self.x[0] += k0 * y;
        self.x[1] += k1 * y;

        // Update Covariance P = (I - KH)P
        // P00 = P00 - K0*P00
        // P01 = P01 - K0*P01
        // P10 = P10 - K1*P00
        // P11 = P11 - K1*P01

        let p00 = self.p[0][0];
        let p01 = self.p[0][1];

        self.p[0][0] -= k0 * p00;
        self.p[0][1] -= k0 * p01;
        self.p[1][0] -= k1 * p00;
        self.p[1][1] -= k1 * p01;
    }

    pub fn state(&self) -> KalmanState {
        KalmanState {
            position: self.x[0],
            velocity: self.x[1],
        }
    }
}
