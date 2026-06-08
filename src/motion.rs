use crate::motor::MotorControl;
use crate::pid::{clamp, Pid};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DriveGeometry {
    pub track_width_m: f32,
    pub max_wheel_speed_mps: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Twist {
    pub linear_mps: f32,
    pub angular_rad_s: f32,
}

pub struct DifferentialDrive<LEFT, RIGHT> {
    left: LEFT,
    right: RIGHT,
    geometry: DriveGeometry,
}

impl<LEFT, RIGHT> DifferentialDrive<LEFT, RIGHT>
where
    LEFT: MotorControl,
    RIGHT: MotorControl,
{
    pub fn new(left: LEFT, right: RIGHT, geometry: DriveGeometry) -> Self {
        Self {
            left,
            right,
            geometry,
        }
    }

    pub fn command_twist(&mut self, twist: Twist) {
        let half_track = self.geometry.track_width_m * 0.5;
        let mut left_mps = twist.linear_mps - twist.angular_rad_s * half_track;
        let mut right_mps = twist.linear_mps + twist.angular_rad_s * half_track;

        let max_requested = max_f32(abs_f32(left_mps), abs_f32(right_mps));
        if max_requested > self.geometry.max_wheel_speed_mps && max_requested > 0.0 {
            let scale = self.geometry.max_wheel_speed_mps / max_requested;
            left_mps *= scale;
            right_mps *= scale;
        }

        self.left
            .set_target_normalized(left_mps / self.geometry.max_wheel_speed_mps);
        self.right
            .set_target_normalized(right_mps / self.geometry.max_wheel_speed_mps);
    }

    pub fn stop_gradual(&mut self) {
        self.left.stop_gradual();
        self.right.stop_gradual();
    }

    pub fn emergency_brake_now(&mut self) {
        self.left.emergency_brake_now();
        self.right.emergency_brake_now();
    }

    pub fn update(&mut self, dt_s: f32) {
        self.left.update(dt_s);
        self.right.update(dt_s);
    }

    pub fn split(self) -> (LEFT, RIGHT) {
        (self.left, self.right)
    }
}

pub struct WallFollowController {
    pid: Pid,
    target_left_m: f32,
    target_right_m: f32,
    max_angular_rad_s: f32,
}

impl WallFollowController {
    pub const fn new(
        pid: Pid,
        target_left_m: f32,
        target_right_m: f32,
        max_angular_rad_s: f32,
    ) -> Self {
        Self {
            pid,
            target_left_m,
            target_right_m,
            max_angular_rad_s,
        }
    }

    pub fn reset(&mut self) {
        self.pid.reset();
    }

    pub fn update_centered(
        &mut self,
        left_distance_m: Option<f32>,
        right_distance_m: Option<f32>,
        base_speed_mps: f32,
        dt_s: f32,
    ) -> Twist {
        let error = match (left_distance_m, right_distance_m) {
            (Some(left), Some(right)) => left - right,
            (Some(left), None) => left - self.target_left_m,
            (None, Some(right)) => self.target_right_m - right,
            (None, None) => 0.0,
        };

        let angular = clamp(
            self.pid.update_error(error, dt_s),
            -self.max_angular_rad_s,
            self.max_angular_rad_s,
        );

        Twist {
            linear_mps: base_speed_mps,
            angular_rad_s: angular,
        }
    }
}

pub struct DistanceMove {
    target_m: f32,
    cruise_speed_mps: f32,
    slow_down_distance_m: f32,
    tolerance_m: f32,
}

impl DistanceMove {
    pub const fn new(
        target_m: f32,
        cruise_speed_mps: f32,
        slow_down_distance_m: f32,
        tolerance_m: f32,
    ) -> Self {
        Self {
            target_m,
            cruise_speed_mps,
            slow_down_distance_m,
            tolerance_m,
        }
    }

    pub fn update(&self, traveled_m: f32) -> (Twist, bool) {
        let remaining = self.target_m - traveled_m;
        if remaining <= self.tolerance_m {
            return (Twist::default(), true);
        }

        let speed_scale = if self.slow_down_distance_m > 0.0 {
            clamp(remaining / self.slow_down_distance_m, 0.20, 1.0)
        } else {
            1.0
        };

        (
            Twist {
                linear_mps: self.cruise_speed_mps * speed_scale,
                angular_rad_s: 0.0,
            },
            false,
        )
    }
}

fn abs_f32(value: f32) -> f32 {
    if value < 0.0 {
        -value
    } else {
        value
    }
}

fn max_f32(a: f32, b: f32) -> f32 {
    if a > b {
        a
    } else {
        b
    }
}
