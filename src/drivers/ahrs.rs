use micromath::F32Ext;

#[derive(Clone, Copy, Debug)]
pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Default for Quaternion {
    fn default() -> Self {
        Self {
            w: 1.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }
}

pub struct Mahony {
    // PID constants
    kp: f32,
    ki: f32,

    // Integral error
    ix: f32,
    iy: f32,
    iz: f32,

    pub q: Quaternion,
}

impl Mahony {
    pub fn new() -> Self {
        Self {
            kp: 2.0,   // Default Kp
            ki: 0.005, // Default Ki (slightly higher for mag)
            ix: 0.0,
            iy: 0.0,
            iz: 0.0,
            q: Quaternion::default(),
        }
    }

    pub fn update(&mut self, dt: f32, gx: f32, gy: f32, gz: f32, ax: f32, ay: f32, az: f32) {
        let mut q0 = self.q.w;
        let mut q1 = self.q.x;
        let mut q2 = self.q.y;
        let mut q3 = self.q.z;

        // Normalise accelerometer measurement
        let mut recip_norm = ax * ax + ay * ay + az * az;
        if recip_norm == 0.0 {
            return;
        }
        recip_norm = recip_norm.sqrt().recip();
        let ax = ax * recip_norm;
        let ay = ay * recip_norm;
        let az = az * recip_norm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        let halfvx = q1 * q3 - q0 * q2;
        let halfvy = q0 * q1 + q2 * q3;
        let halfvz = q0 * q0 - 0.5 + q3 * q3;

        // Error is sum of cross product between estimated and measured direction of gravity
        let halfex = ay * halfvz - az * halfvy;
        let halfey = az * halfvx - ax * halfvz;
        let halfez = ax * halfvy - ay * halfvx;

        // Compute and apply integral feedback if enabled
        if self.ki > 0.0 {
            self.ix += self.ki * halfex * dt;
            self.iy += self.ki * halfey * dt;
            self.iz += self.ki * halfez * dt;
        } else {
            self.ix = 0.0;
            self.iy = 0.0;
            self.iz = 0.0;
        }

        // Apply proportional feedback
        let gx = gx + (self.kp * halfex + self.ix);
        let gy = gy + (self.kp * halfey + self.iy);
        let gz = gz + (self.kp * halfez + self.iz);

        // Integrate rate of change of quaternion
        let gx = gx * (0.5 * dt);
        let gy = gy * (0.5 * dt);
        let gz = gz * (0.5 * dt);

        let qa = q0;
        let qb = q1;
        let qc = q2;
        q0 += -qb * gx - qc * gy - q3 * gz;
        q1 += qa * gx + qc * gz - q3 * gy;
        q2 += qa * gy - qb * gz + q3 * gx;
        q3 += qa * gz + qb * gy - qc * gx;

        // Normalise quaternion
        recip_norm = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
        recip_norm = recip_norm.sqrt().recip();
        self.q.w = q0 * recip_norm;
        self.q.x = q1 * recip_norm;
        self.q.y = q2 * recip_norm;
        self.q.z = q3 * recip_norm;
    }

    pub fn update_9dof(
        &mut self,
        dt: f32,
        gx: f32,
        gy: f32,
        gz: f32,
        ax: f32,
        ay: f32,
        az: f32,
        mx: f32,
        my: f32,
        mz: f32,
    ) {
        let mut q0 = self.q.w;
        let mut q1 = self.q.x;
        let mut q2 = self.q.y;
        let mut q3 = self.q.z;

        // Normalise accelerometer measurement
        let mut recip_norm = ax * ax + ay * ay + az * az;
        if recip_norm > 0.0 {
            recip_norm = recip_norm.sqrt().recip();
            let ax = ax * recip_norm;
            let ay = ay * recip_norm;
            let az = az * recip_norm;

            // Normalise magnetometer measurement
            let mut recip_norm_m = mx * mx + my * my + mz * mz;
            if recip_norm_m > 0.0 {
                recip_norm_m = recip_norm_m.sqrt().recip();
                let mx = mx * recip_norm_m;
                let my = my * recip_norm_m;
                let mz = mz * recip_norm_m;

                // Reference direction of Earth's magnetic field
                let hx = mx * (q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3)
                    + my * (2.0 * (q1 * q2 - q0 * q3))
                    + mz * (2.0 * (q1 * q3 + q0 * q2));
                let hy = mx * (2.0 * (q1 * q2 + q0 * q3))
                    + my * (q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3)
                    + mz * (2.0 * (q2 * q3 - q0 * q1));
                let bx = (hx * hx + hy * hy).sqrt();
                let bz = mx * (2.0 * (q1 * q3 - q0 * q2))
                    + my * (2.0 * (q2 * q3 + q0 * q1))
                    + mz * (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);

                // Estimated direction of gravity and magnetic field
                let vx = 2.0 * (q1 * q3 - q0 * q2);
                let vy = 2.0 * (q0 * q1 + q2 * q3);
                let vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
                let wx = 2.0 * bx * (0.5 - q2 * q2 - q3 * q3) + 2.0 * bz * (q1 * q3 - q0 * q2);
                let wy = 2.0 * bx * (q1 * q2 - q0 * q3) + 2.0 * bz * (q0 * q1 + q2 * q3);
                let wz = 2.0 * bx * (q0 * q2 + q1 * q3) + 2.0 * bz * (0.5 - q1 * q1 - q2 * q2);

                // Error is sum of cross product between estimated direction and measured direction of field vectors
                let ex = (ay * vz - az * vy) + (my * wz - mz * wy);
                let ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
                let ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

                if self.ki > 0.0 {
                    self.ix += self.ki * ex * dt;
                    self.iy += self.ki * ey * dt;
                    self.iz += self.ki * ez * dt;
                }

                // Apply dynamic feedback
                let gx = gx + (self.kp * ex + self.ix);
                let gy = gy + (self.kp * ey + self.iy);
                let gz = gz + (self.kp * ez + self.iz);

                // Integrate rate of change of quaternion
                let dt_05 = 0.5 * dt;
                let (pa, pb, pc) = (q0, q1, q2);
                q0 += (-pb * gx - pc * gy - q3 * gz) * dt_05;
                q1 += (pa * gx + pc * gz - q3 * gy) * dt_05;
                q2 += (pa * gy - pb * gz + q3 * gx) * dt_05;
                q3 += (pa * gz + pb * gy - pc * gx) * dt_05;

                // Normalise quaternion
                let recip_norm = (q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3).sqrt().recip();
                self.q.w = q0 * recip_norm;
                self.q.x = q1 * recip_norm;
                self.q.y = q2 * recip_norm;
                self.q.z = q3 * recip_norm;
            } else {
                self.update(dt, gx, gy, gz, ax, ay, az);
            }
        }
    }

    /// Rotate the given vector (x, y, z) from BODY frame to EARTH frame
    /// Returns (x_earth, y_earth, z_earth)
    /// Used to get vertical acceleration (Z-earth)
    pub fn rotate_vector(&self, x: f32, y: f32, z: f32) -> (f32, f32, f32) {
        // q * v * q_conj
        // Implementation of vector rotation by quaternion
        let q0 = self.q.w;
        let q1 = self.q.x;
        let q2 = self.q.y;
        let q3 = self.q.z;

        // https://gamedev.stackexchange.com/questions/28395/rotating-vector3-by-a-quaternion
        let num12 = q0 * q0;
        let num02 = q1 * q1;
        let num13 = q2 * q2;
        let num03 = q3 * q3;

        let x_out = x * (num12 + num02 - num13 - num03)
            + y * (2. * (q1 * q2 - q0 * q3))
            + z * (2. * (q1 * q3 + q0 * q2));
        let y_out = x * (2. * (q1 * q2 + q0 * q3))
            + y * (num12 - num02 + num13 - num03)
            + z * (2. * (q2 * q3 - q0 * q1));
        let z_out = x * (2. * (q1 * q3 - q0 * q2))
            + y * (2. * (q2 * q3 + q0 * q1))
            + z * (num12 - num02 - num13 + num03);

        (x_out, y_out, z_out)
    }
    pub fn get_euler_angles(&self) -> (f32, f32, f32) {
        let q0 = self.q.w;
        let q1 = self.q.x;
        let q2 = self.q.y;
        let q3 = self.q.z;

        // Roll (x-axis rotation)
        let sinr_cosp = 2.0 * (q0 * q1 + q2 * q3);
        let cosr_cosp = 1.0 - 2.0 * (q1 * q1 + q2 * q2);
        let roll = sinr_cosp.atan2(cosr_cosp);

        // Pitch (y-axis rotation)
        let sinp = 2.0 * (q0 * q2 - q3 * q1);
        let pitch = if sinp.abs() >= 1.0 {
            // use 90 degrees if out of range
            core::f32::consts::FRAC_PI_2.copysign(sinp)
        } else {
            sinp.asin()
        };

        // Yaw (z-axis rotation)
        let siny_cosp = 2.0 * (q0 * q3 + q1 * q2);
        let cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3);
        let yaw = siny_cosp.atan2(cosy_cosp);

        (roll, pitch, yaw)
    }
}
