use core::f32::consts::PI;

#[derive(Clone, Copy)]
pub enum GearRatio {
    R10,
    R20,
}

impl GearRatio {
    pub fn from_aux_channel(ch_value: u16) -> Self {
        if ch_value > 1500 {
            Self::R20
        } else {
            Self::R10
        }
    }

    pub fn as_f32(self) -> f32 {
        match self {
            Self::R10 => 10.0,
            Self::R20 => 20.0,
        }
    }

    pub fn as_u8(self) -> u8 {
        match self {
            Self::R10 => 10,
            Self::R20 => 20,
        }
    }
}

pub struct RollController {
    kp: f32,
    ki: f32,
    kd: f32,
    integral: f32,
    integral_limit: f32,
    output_limit: f32,
}

pub struct GearedTabController {
    kp_motor_pos: f32,
    kd_motor_pos: f32,
    max_tab_deg: f32,
    max_motor_cmd: f32,
    max_motor_deg_s: f32,
    motor_pos_est_deg: f32,
    prev_motor_pos_est_deg: f32,
}

impl GearedTabController {
    pub fn new(
        kp_motor_pos: f32,
        kd_motor_pos: f32,
        max_tab_deg: f32,
        max_motor_cmd: f32,
        max_motor_deg_s: f32,
    ) -> Self {
        Self {
            kp_motor_pos,
            kd_motor_pos,
            max_tab_deg: max_tab_deg.abs(),
            max_motor_cmd: max_motor_cmd.abs(),
            max_motor_deg_s: max_motor_deg_s.abs(),
            motor_pos_est_deg: 0.0,
            prev_motor_pos_est_deg: 0.0,
        }
    }

    pub fn reset(&mut self) {
        self.motor_pos_est_deg = 0.0;
        self.prev_motor_pos_est_deg = 0.0;
    }

    pub fn update(
        &mut self,
        dt: f32,
        target_tab_deg: f32,
        gear_ratio: GearRatio,
    ) -> (f32, f32) {
        let ratio = gear_ratio.as_f32();
        let tab_target_deg = target_tab_deg.clamp(-self.max_tab_deg, self.max_tab_deg);
        let motor_target_deg = tab_target_deg * ratio;

        let motor_error_deg = motor_target_deg - self.motor_pos_est_deg;
        let motor_rate_est_deg_s = if dt > 0.0 {
            (self.motor_pos_est_deg - self.prev_motor_pos_est_deg) / dt
        } else {
            0.0
        };

        let motor_cmd = (self.kp_motor_pos * motor_error_deg
            - self.kd_motor_pos * motor_rate_est_deg_s)
            .clamp(-self.max_motor_cmd, self.max_motor_cmd);

        self.prev_motor_pos_est_deg = self.motor_pos_est_deg;
        self.motor_pos_est_deg += motor_cmd * self.max_motor_deg_s * dt;

        let tab_est_deg = (self.motor_pos_est_deg / ratio).clamp(-self.max_tab_deg, self.max_tab_deg);
        (tab_est_deg, motor_cmd)
    }
}

impl RollController {
    pub fn new(kp: f32, ki: f32, kd: f32, integral_limit: f32, output_limit: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral: 0.0,
            integral_limit: integral_limit.abs(),
            output_limit: output_limit.abs(),
        }
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
    }

    pub fn update(
        &mut self,
        dt: f32,
        roll_setpoint_rad: f32,
        roll_measured_rad: f32,
        roll_rate_rad_s: f32,
    ) -> f32 {
        let error = roll_setpoint_rad - roll_measured_rad;

        self.integral += error * dt;
        self.integral = self
            .integral
            .clamp(-self.integral_limit, self.integral_limit);

        let output = self.kp * error + self.ki * self.integral - self.kd * roll_rate_rad_s;
        output.clamp(-self.output_limit, self.output_limit)
    }
}

pub fn crsf_to_unit(ch_value: u16) -> f32 {
    let normalized = (ch_value as f32 - 992.0) / 820.0;
    normalized.clamp(-1.0, 1.0)
}

pub fn unit_to_dshot(unit_throttle: f32, armed: bool) -> u16 {
    if !armed {
        return 0;
    }

    let t = unit_throttle.clamp(0.0, 1.0);
    let dshot_min = 48.0;
    let dshot_max = 2047.0;
    let value = dshot_min + t * (dshot_max - dshot_min);
    value as u16
}

pub fn signed_unit_to_dshot_3d(unit_cmd: f32, armed: bool) -> u16 {
    if !armed {
        return 0;
    }

    let cmd = unit_cmd.clamp(-1.0, 1.0);
    if cmd >= 0.0 {
        let start = 1048.0;
        let max = 2047.0;
        (start + cmd * (max - start)) as u16
    } else {
        let start = 1047.0;
        let min = 48.0;
        (start - (-cmd) * (start - min)) as u16
    }
}

pub fn max_roll_setpoint_from_stick(stick: f32, max_roll_deg: f32) -> f32 {
    let deg = stick.clamp(-1.0, 1.0) * max_roll_deg;
    deg * PI / 180.0
}

pub fn roll_output_to_tab_target_deg(roll_output: f32, max_tab_deg: f32) -> f32 {
    (roll_output * max_tab_deg).clamp(-max_tab_deg.abs(), max_tab_deg.abs())
}

pub fn dshot_frame(command: u16, telemetry: bool) -> u16 {
    let mut packet = (command & 0x07ff) << 1;
    if telemetry {
        packet |= 1;
    }

    let mut csum = 0u16;
    let mut csum_data = packet;
    for _ in 0..3 {
        csum ^= csum_data;
        csum_data >>= 4;
    }
    csum &= 0x000f;

    (packet << 4) | csum
}
