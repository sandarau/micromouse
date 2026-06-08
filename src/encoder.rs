#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EncoderSample {
    pub delta_ticks: i32,
    pub total_ticks: i64,
    pub delta_distance_m: f32,
    pub total_distance_m: f32,
    pub velocity_mps: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WheelEncoder {
    ticks_per_rev: i32,
    wheel_circumference_m: f32,
    filter_alpha: f32,
    initialized: bool,
    last_count: i32,
    total_ticks: i64,
    total_distance_m: f32,
    velocity_mps: f32,
}

impl WheelEncoder {
    pub const fn new(ticks_per_rev: i32, wheel_diameter_m: f32, filter_alpha: f32) -> Self {
        Self {
            ticks_per_rev,
            wheel_circumference_m: wheel_diameter_m * 3.1415927,
            filter_alpha,
            initialized: false,
            last_count: 0,
            total_ticks: 0,
            total_distance_m: 0.0,
            velocity_mps: 0.0,
        }
    }

    pub fn reset(&mut self, current_count: i32) {
        self.initialized = true;
        self.last_count = current_count;
        self.total_ticks = 0;
        self.total_distance_m = 0.0;
        self.velocity_mps = 0.0;
    }

    pub fn update_count(&mut self, count: i32, dt_s: f32) -> EncoderSample {
        let delta_ticks = if self.initialized {
            count - self.last_count
        } else {
            self.initialized = true;
            0
        };

        self.last_count = count;
        self.apply_delta(delta_ticks, dt_s)
    }

    pub fn update_wrapping_u16(&mut self, count: u16, dt_s: f32) -> EncoderSample {
        let count_i32 = count as i32;
        let delta_ticks = if self.initialized {
            wrapping_delta_u16(self.last_count as u16, count) as i32
        } else {
            self.initialized = true;
            0
        };

        self.last_count = count_i32;
        self.apply_delta(delta_ticks, dt_s)
    }

    pub fn total_distance_m(&self) -> f32 {
        self.total_distance_m
    }

    pub fn velocity_mps(&self) -> f32 {
        self.velocity_mps
    }

    fn apply_delta(&mut self, delta_ticks: i32, dt_s: f32) -> EncoderSample {
        self.total_ticks += delta_ticks as i64;

        let delta_distance_m = if self.ticks_per_rev == 0 {
            0.0
        } else {
            delta_ticks as f32 * self.wheel_circumference_m / self.ticks_per_rev as f32
        };

        self.total_distance_m += delta_distance_m;

        if dt_s > 0.0 {
            let raw_velocity = delta_distance_m / dt_s;
            let alpha = clamp01(self.filter_alpha);
            self.velocity_mps = alpha * raw_velocity + (1.0 - alpha) * self.velocity_mps;
        }

        EncoderSample {
            delta_ticks,
            total_ticks: self.total_ticks,
            delta_distance_m,
            total_distance_m: self.total_distance_m,
            velocity_mps: self.velocity_mps,
        }
    }
}

pub fn wrapping_delta_u16(previous: u16, current: u16) -> i16 {
    current.wrapping_sub(previous) as i16
}

fn clamp01(value: f32) -> f32 {
    if value < 0.0 {
        0.0
    } else if value > 1.0 {
        1.0
    } else {
        value
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wrapping_u16_delta_handles_rollover() {
        assert_eq!(wrapping_delta_u16(65_530, 4), 10);
        assert_eq!(wrapping_delta_u16(4, 65_530), -10);
    }
}
