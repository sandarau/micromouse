#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PidGains {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
}

impl PidGains {
    pub const fn new(kp: f32, ki: f32, kd: f32) -> Self {
        Self { kp, ki, kd }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PidLimits {
    pub output_min: f32,
    pub output_max: f32,
    pub integrator_min: f32,
    pub integrator_max: f32,
}

impl PidLimits {
    pub const fn symmetric(output_abs_max: f32, integrator_abs_max: f32) -> Self {
        Self {
            output_min: -output_abs_max,
            output_max: output_abs_max,
            integrator_min: -integrator_abs_max,
            integrator_max: integrator_abs_max,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pid {
    gains: PidGains,
    limits: PidLimits,
    integrator: f32,
    previous_measurement: Option<f32>,
    previous_error: Option<f32>,
}

impl Pid {
    pub const fn new(gains: PidGains, limits: PidLimits) -> Self {
        Self {
            gains,
            limits,
            integrator: 0.0,
            previous_measurement: None,
            previous_error: None,
        }
    }

    pub fn reset(&mut self) {
        self.integrator = 0.0;
        self.previous_measurement = None;
        self.previous_error = None;
    }

    pub fn set_gains(&mut self, gains: PidGains) {
        self.gains = gains;
    }

    pub fn gains(&self) -> PidGains {
        self.gains
    }

    pub fn integrator(&self) -> f32 {
        self.integrator
    }

    pub fn update(&mut self, setpoint: f32, measurement: f32, dt_s: f32) -> f32 {
        let error = setpoint - measurement;
        let derivative = match self.previous_measurement {
            Some(previous) if dt_s > 0.0 => -(measurement - previous) / dt_s,
            _ => 0.0,
        };

        self.previous_measurement = Some(measurement);
        self.previous_error = Some(error);
        self.finish_update(error, derivative, dt_s)
    }

    pub fn update_error(&mut self, error: f32, dt_s: f32) -> f32 {
        let derivative = match self.previous_error {
            Some(previous) if dt_s > 0.0 => (error - previous) / dt_s,
            _ => 0.0,
        };

        self.previous_error = Some(error);
        self.previous_measurement = None;
        self.finish_update(error, derivative, dt_s)
    }

    fn finish_update(&mut self, error: f32, derivative: f32, dt_s: f32) -> f32 {
        if dt_s > 0.0 {
            self.integrator += error * dt_s;
            self.integrator = clamp(
                self.integrator,
                self.limits.integrator_min,
                self.limits.integrator_max,
            );
        }

        let output =
            self.gains.kp * error + self.gains.ki * self.integrator + self.gains.kd * derivative;

        clamp(output, self.limits.output_min, self.limits.output_max)
    }
}

pub fn clamp(value: f32, min: f32, max: f32) -> f32 {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}
