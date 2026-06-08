use crate::hal::{OutputPin, PwmPin};
use crate::pid::clamp;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ZeroBehavior {
    Coast,
    Brake,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorConfig {
    pub max_duty: u16,
    pub accel_duty_per_s: f32,
    pub brake_duty_per_s: f32,
    pub zero_behavior: ZeroBehavior,
}

impl MotorConfig {
    pub const fn new(
        max_duty: u16,
        accel_duty_per_s: f32,
        brake_duty_per_s: f32,
        zero_behavior: ZeroBehavior,
    ) -> Self {
        Self {
            max_duty,
            accel_duty_per_s,
            brake_duty_per_s,
            zero_behavior,
        }
    }
}

pub trait MotorControl {
    fn set_target_normalized(&mut self, speed: f32);
    fn stop_gradual(&mut self);
    fn emergency_brake_now(&mut self);
    fn update(&mut self, dt_s: f32);
    fn target_normalized(&self) -> f32;
    fn current_normalized(&self) -> f32;
}

pub struct DcMotor<PWM, IN1, IN2> {
    pwm: PWM,
    in1: IN1,
    in2: IN2,
    config: MotorConfig,
    target_duty: f32,
    current_duty: f32,
}

impl<PWM, IN1, IN2> DcMotor<PWM, IN1, IN2>
where
    PWM: PwmPin,
    IN1: OutputPin,
    IN2: OutputPin,
{
    pub fn new(pwm: PWM, in1: IN1, in2: IN2, mut config: MotorConfig) -> Self {
        if config.max_duty == 0 {
            config.max_duty = pwm.max_duty();
        }

        Self {
            pwm,
            in1,
            in2,
            config,
            target_duty: 0.0,
            current_duty: 0.0,
        }
    }

    pub fn set_target_duty_signed(&mut self, duty: i16) {
        let max = self.config.max_duty as i16;
        self.target_duty = duty.clamp(-max, max) as f32;
    }

    pub fn config(&self) -> MotorConfig {
        self.config
    }

    pub fn current_duty_signed(&self) -> i16 {
        self.current_duty as i16
    }

    pub fn target_duty_signed(&self) -> i16 {
        self.target_duty as i16
    }

    pub fn release(self) -> (PWM, IN1, IN2) {
        (self.pwm, self.in1, self.in2)
    }

    fn apply_outputs(&mut self) {
        let duty = abs_f32(self.current_duty);

        if duty < 1.0 {
            match self.config.zero_behavior {
                ZeroBehavior::Coast => {
                    self.pwm.set_duty(0);
                    self.in1.set_low();
                    self.in2.set_low();
                }
                ZeroBehavior::Brake => {
                    self.in1.set_high();
                    self.in2.set_high();
                    self.pwm.set_duty(self.config.max_duty);
                }
            }
            return;
        }

        let duty_u16 = duty_to_u16(duty, self.config.max_duty);

        self.pwm.set_duty(0);
        if self.current_duty > 0.0 {
            self.in1.set_high();
            self.in2.set_low();
        } else {
            self.in1.set_low();
            self.in2.set_high();
        }
        self.pwm.set_duty(duty_u16);
    }
}

impl<PWM, IN1, IN2> MotorControl for DcMotor<PWM, IN1, IN2>
where
    PWM: PwmPin,
    IN1: OutputPin,
    IN2: OutputPin,
{
    fn set_target_normalized(&mut self, speed: f32) {
        let normalized = if speed.is_finite() { speed } else { 0.0 };
        let normalized = clamp(normalized, -1.0, 1.0);
        self.target_duty = normalized * self.config.max_duty as f32;
    }

    fn stop_gradual(&mut self) {
        self.target_duty = 0.0;
    }

    fn emergency_brake_now(&mut self) {
        self.target_duty = 0.0;
        self.current_duty = 0.0;
        self.in1.set_high();
        self.in2.set_high();
        self.pwm.set_duty(self.config.max_duty);
    }

    fn update(&mut self, dt_s: f32) {
        let diff = self.target_duty - self.current_duty;
        if abs_f32(diff) < 0.5 {
            self.current_duty = self.target_duty;
            self.apply_outputs();
            return;
        }

        let same_direction = same_sign(self.current_duty, self.target_duty);
        let speeding_up = same_direction && abs_f32(self.target_duty) > abs_f32(self.current_duty);
        let rate = if speeding_up {
            self.config.accel_duty_per_s
        } else {
            self.config.brake_duty_per_s
        };

        if dt_s <= 0.0 || rate <= 0.0 {
            self.current_duty = self.target_duty;
        } else {
            let max_step = rate * dt_s;
            self.current_duty = move_toward(self.current_duty, self.target_duty, max_step);
        }

        self.apply_outputs();
    }

    fn target_normalized(&self) -> f32 {
        if self.config.max_duty == 0 {
            0.0
        } else {
            self.target_duty / self.config.max_duty as f32
        }
    }

    fn current_normalized(&self) -> f32 {
        if self.config.max_duty == 0 {
            0.0
        } else {
            self.current_duty / self.config.max_duty as f32
        }
    }
}

fn move_toward(current: f32, target: f32, max_step: f32) -> f32 {
    let diff = target - current;
    if abs_f32(diff) <= max_step {
        target
    } else if diff > 0.0 {
        current + max_step
    } else {
        current - max_step
    }
}

fn same_sign(a: f32, b: f32) -> bool {
    (a >= 0.0 && b >= 0.0) || (a <= 0.0 && b <= 0.0)
}

fn abs_f32(value: f32) -> f32 {
    if value < 0.0 {
        -value
    } else {
        value
    }
}

fn duty_to_u16(duty: f32, max_duty: u16) -> u16 {
    if duty <= 0.0 {
        0
    } else if duty >= max_duty as f32 {
        max_duty
    } else {
        duty as u16
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Clone, Copy)]
    struct FakePwm {
        duty: u16,
        max: u16,
    }

    impl PwmPin for FakePwm {
        fn max_duty(&self) -> u16 {
            self.max
        }

        fn set_duty(&mut self, duty: u16) {
            self.duty = duty;
        }
    }

    #[derive(Clone, Copy)]
    struct FakePin(bool);

    impl OutputPin for FakePin {
        fn set_high(&mut self) {
            self.0 = true;
        }

        fn set_low(&mut self) {
            self.0 = false;
        }
    }

    #[test]
    fn motor_accelerates_and_brakes_gradually() {
        let config = MotorConfig::new(1000, 100.0, 200.0, ZeroBehavior::Coast);
        let mut motor = DcMotor::new(
            FakePwm { duty: 0, max: 1000 },
            FakePin(false),
            FakePin(false),
            config,
        );

        motor.set_target_normalized(1.0);
        motor.update(1.0);
        assert_eq!(motor.current_duty_signed(), 100);

        motor.stop_gradual();
        motor.update(0.25);
        assert_eq!(motor.current_duty_signed(), 50);

        motor.update(0.25);
        assert_eq!(motor.current_duty_signed(), 0);
    }
}
