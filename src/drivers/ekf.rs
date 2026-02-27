/// Extended Kalman Filter for Attitude Estimation
///
/// State vector (10 elements):
///   [0..3] = quaternion (q0, q1, q2, q3)  -- scalar-first convention
///   [4..6] = gyro bias (bx, by, bz)       -- rad/s
///   [7..9] = accel bias (ax, ay, az)      -- normalized g
///
/// This is a no_std, no-alloc implementation using flat f32 arrays.
/// No nalgebra dependency needed.

use micromath::F32Ext;

// ── Constants ────────────────────────────────────────────────────────────────

/// Process noise for quaternion integration (very small - gyro is trusted)
const Q_QUAT: f32 = 1e-6;
/// Process noise for gyro bias drift
const Q_GBIAS: f32 = 1e-7;
/// Process noise for accel bias drift
const Q_ABIAS: f32 = 1e-7;

/// Measurement noise for accelerometer under normal flight (< 1.5G total)
const R_ACCEL_NORMAL: f32 = 0.05;
/// Measurement noise when high-G detected (rocket burn / high thrust): EKF trusts only gyro
const R_ACCEL_HIGH_G: f32 = 500.0;

/// Threshold in G above which we boost accelerometer noise
const HIGH_G_THRESHOLD: f32 = 1.5; // G (includes gravity = ~1G at rest, so ~0.5G net accel)

/// Initial covariance diagonal for quaternion states
const P0_QUAT: f32 = 0.01;
/// Initial covariance diagonal for bias states
const P0_BIAS: f32 = 0.1;

// ── Data types ───────────────────────────────────────────────────────────────

#[derive(Clone, Copy)]
pub struct EkfDebug {
    pub is_high_g: bool,
    pub accel_mag_g: f32,
}

// ── Helper matrix functions (10×10 flat arrays) ──────────────────────────────

const N: usize = 10;
type Mat = [f32; N * N];
type Vec10 = [f32; N];

/// Zero matrix
#[inline]
fn mat_zero() -> Mat {
    [0.0f32; N * N]
}

/// Identity matrix
#[inline]
fn mat_identity() -> Mat {
    let mut m = mat_zero();
    for i in 0..N {
        m[i * N + i] = 1.0;
    }
    m
}

/// m[r][c]
#[inline]
fn m(mat: &Mat, r: usize, c: usize) -> f32 {
    mat[r * N + c]
}

/// mat[r][c] = v
#[inline]
fn mset(mat: &mut Mat, r: usize, c: usize, v: f32) {
    mat[r * N + c] = v;
}

/// C = A * B  (10×10 full multiply)
fn mat_mul(a: &Mat, b: &Mat) -> Mat {
    let mut c = mat_zero();
    for i in 0..N {
        for j in 0..N {
            let mut s = 0.0f32;
            for k in 0..N {
                s += m(a, i, k) * m(b, k, j);
            }
            mset(&mut c, i, j, s);
        }
    }
    c
}

/// C = A + B
fn mat_add(a: &Mat, b: &Mat) -> Mat {
    let mut c = mat_zero();
    for i in 0..N * N {
        c[i] = a[i] + b[i];
    }
    c
}

/// Transpose
fn mat_transpose(a: &Mat) -> Mat {
    let mut t = mat_zero();
    for i in 0..N {
        for j in 0..N {
            mset(&mut t, j, i, m(a, i, j));
        }
    }
    t
}

/// C = A * B^T
fn mat_mul_t(a: &Mat, b: &Mat) -> Mat {
    let bt = mat_transpose(b);
    mat_mul(a, &bt)
}

// ── EKF struct ───────────────────────────────────────────────────────────────

pub struct AttitudeEkf {
    /// State vector: [q0,q1,q2,q3, gbx,gby,gbz, abx,aby,abz]
    x: Vec10,
    /// Error covariance matrix P (10×10)
    p: Mat,
    /// Debug info from last update
    pub debug: EkfDebug,
}

impl AttitudeEkf {
    pub fn new() -> Self {
        let mut x = [0.0f32; N];
        x[0] = 1.0; // q0 = 1 (identity quaternion)

        let mut p = mat_identity();
        for i in 0..4 {
            mset(&mut p, i, i, P0_QUAT);
        }
        for i in 4..N {
            mset(&mut p, i, i, P0_BIAS);
        }

        Self {
            x,
            p,
            debug: EkfDebug { is_high_g: false, accel_mag_g: 1.0 },
        }
    }

    /// Get current quaternion [q0, q1, q2, q3]
    pub fn get_quaternion(&self) -> [f32; 4] {
        [self.x[0], self.x[1], self.x[2], self.x[3]]
    }

    /// Get gyro bias [bx, by, bz] in rad/s
    pub fn get_gyro_bias(&self) -> [f32; 3] {
        [self.x[4], self.x[5], self.x[6]]
    }

    /// Get Euler angles (roll, pitch, yaw) in radians
    pub fn get_euler(&self) -> (f32, f32, f32) {
        let q0 = self.x[0];
        let q1 = self.x[1];
        let q2 = self.x[2];
        let q3 = self.x[3];

        // Roll (x-axis)
        let sinr_cosp = 2.0 * (q0 * q1 + q2 * q3);
        let cosr_cosp = 1.0 - 2.0 * (q1 * q1 + q2 * q2);
        let roll = sinr_cosp.atan2(cosr_cosp);

        // Pitch (y-axis)
        let sinp = 2.0 * (q0 * q2 - q3 * q1);
        let pitch = if sinp.abs() >= 1.0 {
            core::f32::consts::FRAC_PI_2.copysign(sinp)
        } else {
            sinp.asin()
        };

        // Yaw (z-axis)
        let siny_cosp = 2.0 * (q0 * q3 + q1 * q2);
        let cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3);
        let yaw = siny_cosp.atan2(cosy_cosp);

        (roll, pitch, yaw)
    }

    /// Rotate a body-frame vector to earth frame using current attitude
    pub fn rotate_to_earth(&self, bx: f32, by: f32, bz: f32) -> (f32, f32, f32) {
        let q0 = self.x[0]; let q1 = self.x[1];
        let q2 = self.x[2]; let q3 = self.x[3];
        let n12 = q0*q0; let n02 = q1*q1;
        let n13 = q2*q2; let n03 = q3*q3;
        let ex = bx*(n12+n02-n13-n03) + by*(2.*(q1*q2-q0*q3)) + bz*(2.*(q1*q3+q0*q2));
        let ey = bx*(2.*(q1*q2+q0*q3)) + by*(n12-n02+n13-n03) + bz*(2.*(q2*q3-q0*q1));
        let ez = bx*(2.*(q1*q3-q0*q2)) + by*(2.*(q2*q3+q0*q1)) + bz*(n12-n02-n13+n03);
        (ex, ey, ez)
    }

    // ── Predict step ─────────────────────────────────────────────────────────

    /// Propagate state forward by `dt` seconds with raw gyroscope measurement (rad/s).
    /// Gyro bias is estimated and subtracted internally.
    pub fn predict(&mut self, dt: f32, gx_raw: f32, gy_raw: f32, gz_raw: f32) {
        // Correct gyro with estimated bias
        let gx = gx_raw - self.x[4];
        let gy = gy_raw - self.x[5];
        let gz = gz_raw - self.x[6];

        let q0 = self.x[0]; let q1 = self.x[1];
        let q2 = self.x[2]; let q3 = self.x[3];

        // Quaternion kinematics: q_dot = 0.5 * Omega(gyro) * q
        let dq0 = 0.5 * (-q1*gx - q2*gy - q3*gz) * dt;
        let dq1 = 0.5 * ( q0*gx + q2*gz - q3*gy) * dt;
        let dq2 = 0.5 * ( q0*gy - q1*gz + q3*gx) * dt;
        let dq3 = 0.5 * ( q0*gz + q1*gy - q2*gx) * dt;

        self.x[0] += dq0;
        self.x[1] += dq1;
        self.x[2] += dq2;
        self.x[3] += dq3;

        // Normalise quaternion
        self.normalise_quat();

        // Build Jacobian F (state transition matrix)
        // F = I + dt * dF/dx  (linearised)
        let mut f = mat_identity();

        // df(q)/dq : omega cross matrix scaled by 0.5*dt
        let h = 0.5 * dt;
        // Row 0 (dq0): d/dq1=-gx*h, d/dq2=-gy*h, d/dq3=-gz*h
        mset(&mut f, 0,1, -gx*h); mset(&mut f, 0,2, -gy*h); mset(&mut f, 0,3, -gz*h);
        // Row 1 (dq1): d/dq0= gx*h, d/dq2= gz*h, d/dq3=-gy*h
        mset(&mut f, 1,0,  gx*h); mset(&mut f, 1,2,  gz*h); mset(&mut f, 1,3, -gy*h);
        // Row 2 (dq2): d/dq0= gy*h, d/dq1=-gz*h, d/dq3= gx*h
        mset(&mut f, 2,0,  gy*h); mset(&mut f, 2,1, -gz*h); mset(&mut f, 2,3,  gx*h);
        // Row 3 (dq3): d/dq0= gz*h, d/dq1= gy*h, d/dq2=-gx*h
        mset(&mut f, 3,0,  gz*h); mset(&mut f, 3,1,  gy*h); mset(&mut f, 3,2, -gx*h);

        // df(q)/d(gbias) : coupling term  (−0.5*dt * Omega_q)
        //   dq0/dgbx = −0.5*dt*q1, dq0/dgby = −0.5*dt*q2, dq0/dgbz = −0.5*dt*q3
        mset(&mut f, 0,4, 0.5*dt*q1); mset(&mut f, 0,5, 0.5*dt*q2); mset(&mut f, 0,6, 0.5*dt*q3);
        mset(&mut f, 1,4,-0.5*dt*q0); mset(&mut f, 1,5, 0.5*dt*q3); mset(&mut f, 1,6,-0.5*dt*q2);
        mset(&mut f, 2,4,-0.5*dt*q3); mset(&mut f, 2,5,-0.5*dt*q0); mset(&mut f, 2,6, 0.5*dt*q1);
        mset(&mut f, 3,4, 0.5*dt*q2); mset(&mut f, 3,5,-0.5*dt*q1); mset(&mut f, 3,6,-0.5*dt*q0);

        // Build process noise Q
        let mut q_noise = mat_zero();
        for i in 0..4 { mset(&mut q_noise, i, i, Q_QUAT * dt); }
        for i in 4..7 { mset(&mut q_noise, i, i, Q_GBIAS * dt); }
        for i in 7..10 { mset(&mut q_noise, i, i, Q_ABIAS * dt); }

        // P = F*P*F' + Q
        let fp   = mat_mul(&f, &self.p);
        let fpft = mat_mul_t(&fp, &f);
        self.p   = mat_add(&fpft, &q_noise);
    }

    // ── Update step (accelerometer) ──────────────────────────────────────────

    /// Correct state with accelerometer measurement (raw, in G or LSB-normalised).
    /// `ax, ay, az` must be in units of G (divide raw by LSB/G before calling).
    ///
    /// **Dynamic Noise**: if total |accel| > HIGH_G_THRESHOLD, we massively increase
    /// R_accel so the EKF ignores the accelerometer and trusts only the gyro.
    pub fn update_accel(&mut self, ax: f32, ay: f32, az: f32) {
        // Detect high-G (thrust / hard manoeuvre)
        let accel_mag = (ax*ax + ay*ay + az*az).sqrt();
        self.debug.accel_mag_g = accel_mag;
        let r_accel = if accel_mag > HIGH_G_THRESHOLD {
            self.debug.is_high_g = true;
            R_ACCEL_HIGH_G
        } else {
            self.debug.is_high_g = false;
            R_ACCEL_NORMAL
        };

        // Normalise accelerometer (pointing towards real gravity direction)
        if accel_mag < 0.01 { return; } // near-zero: guard division
        let recip = accel_mag.recip();
        let ax_n = ax * recip;
        let ay_n = ay * recip;
        let az_n = az * recip;

        // Expected gravity direction in body frame from current quaternion
        // g_body = R^T * [0,0,1] (gravity points DOWN in NED convention)
        let q0 = self.x[0]; let q1 = self.x[1];
        let q2 = self.x[2]; let q3 = self.x[3];

        // Expected accel = R^T * e_z (third column of R^T = third row of R)
        let hx = 2.0 * (q1*q3 - q0*q2);
        let hy = 2.0 * (q0*q1 + q2*q3);
        let hz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

        // Innovation y = measured - predicted
        let y0 = ax_n - hx;
        let y1 = ay_n - hy;
        let y2 = az_n - hz;

        // Jacobian H (3×10): dh/dx  (only quaternion columns are non-zero)
        // dh/dq0 = [ -2*q2,  2*q1,  2*q0 ]^T   etc.
        // H is 3×10, stored row-major
        let mut h_jac = [0.0f32; 3 * N];
        // Row 0 → hx
        h_jac[0*N+0] = -2.*q2; h_jac[0*N+1] =  2.*q3; h_jac[0*N+2] = -2.*q0; h_jac[0*N+3] =  2.*q1;
        // Row 1 → hy
        h_jac[1*N+0] =  2.*q1; h_jac[1*N+1] =  2.*q0; h_jac[1*N+2] =  2.*q3; h_jac[1*N+3] =  2.*q2;
        // Row 2 → hz
        h_jac[2*N+0] =  2.*q0; h_jac[2*N+1] = -2.*q1; h_jac[2*N+2] = -2.*q2; h_jac[2*N+3] =  2.*q3;

        // S = H * P * H' + R*I  (3×3)
        // K = P * H' * S^{-1}   (10×3)
        // x = x + K * y
        // P = (I - K*H) * P

        // Compute H * P  (3×10)
        let mut hp = [0.0f32; 3 * N];
        for r in 0..3 {
            for c in 0..N {
                let mut s = 0.0f32;
                for k in 0..N {
                    s += h_jac[r*N+k] * m(&self.p, k, c);
                }
                hp[r*N+c] = s;
            }
        }

        // S = H*P*H' + R*I  (3×3)
        let mut s_mat = [0.0f32; 9];
        for r in 0..3 {
            for c in 0..3 {
                let mut v = if r==c { r_accel } else { 0.0 };
                for k in 0..N {
                    v += h_jac[r*N+k] * hp[c*N+k]; // hp[c] is H[c,:]*P = (HP)[c,:]
                }
                // Wait, HP is (HP)[r,c] already. H' is H[k,c]. Need HP * H'
                s_mat[r*3+c] = s_mat[r*3+c]; // will redo below
                let _ = v;
            }
        }
        // Redo: S[r,c] = sum_k HP[r,k] * H[c,k]
        for r in 0..3 {
            for c in 0..3 {
                let mut v = if r==c { r_accel } else { 0.0 };
                for k in 0..N {
                    v += hp[r*N+k] * h_jac[c*N+k];
                }
                s_mat[r*3+c] = v;
            }
        }

        // Invert 3×3 S analytically
        let s_inv = match mat3_invert(&s_mat) {
            Some(inv) => inv,
            None => return, // singular, skip update
        };

        // K = P * H' * S^{-1}  (10×3)
        // PH' (10×3)
        let mut pht = [0.0f32; N * 3];
        for r in 0..N {
            for c in 0..3 {
                let mut v = 0.0f32;
                for k in 0..N {
                    v += m(&self.p, r, k) * h_jac[c*N+k];
                }
                pht[r*3+c] = v;
            }
        }

        // K = PH' * S^{-1}  (10×3)
        let mut kk = [0.0f32; N * 3];
        for r in 0..N {
            for c in 0..3 {
                let mut v = 0.0f32;
                for k in 0..3 {
                    v += pht[r*3+k] * s_inv[k*3+c];
                }
                kk[r*3+c] = v;
            }
        }

        // State update: x = x + K*y
        // y = [y0, y1, y2]
        for r in 0..N {
            self.x[r] += kk[r*3+0]*y0 + kk[r*3+1]*y1 + kk[r*3+2]*y2;
        }

        // Covariance update: P = (I - K*H)*P = P - K*H*P = P - K*(HP)
        // K*HP (10×10)
        let mut khp = mat_zero();
        for r in 0..N {
            for c in 0..N {
                let mut v = 0.0f32;
                for k in 0..3 {
                    v += kk[r*3+k] * hp[k*N+c];
                }
                mset(&mut khp, r, c, v);
            }
        }
        for i in 0..N*N {
            self.p[i] -= khp[i];
        }

        // Normalise quaternion after update
        self.normalise_quat();
    }

    // ── Internal helpers ─────────────────────────────────────────────────────

    fn normalise_quat(&mut self) {
        let n = (self.x[0]*self.x[0]
                +self.x[1]*self.x[1]
                +self.x[2]*self.x[2]
                +self.x[3]*self.x[3]).sqrt();
        if n > 1e-6 {
            let inv_n = n.recip();
            self.x[0] *= inv_n;
            self.x[1] *= inv_n;
            self.x[2] *= inv_n;
            self.x[3] *= inv_n;
        }
    }
}

// ── 3×3 matrix inversion ─────────────────────────────────────────────────────

fn mat3_invert(m: &[f32; 9]) -> Option<[f32; 9]> {
    // Cofactor expansion
    let det = m[0]*(m[4]*m[8]-m[5]*m[7])
             -m[1]*(m[3]*m[8]-m[5]*m[6])
             +m[2]*(m[3]*m[7]-m[4]*m[6]);
    if det.abs() < 1e-10 {
        return None;
    }
    let inv_det = det.recip();
    Some([
         (m[4]*m[8]-m[5]*m[7])*inv_det, -(m[1]*m[8]-m[2]*m[7])*inv_det,  (m[1]*m[5]-m[2]*m[4])*inv_det,
        -(m[3]*m[8]-m[5]*m[6])*inv_det,  (m[0]*m[8]-m[2]*m[6])*inv_det, -(m[0]*m[5]-m[2]*m[3])*inv_det,
         (m[3]*m[7]-m[4]*m[6])*inv_det, -(m[0]*m[7]-m[1]*m[6])*inv_det,  (m[0]*m[4]-m[1]*m[3])*inv_det,
    ])
}
