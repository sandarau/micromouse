use crate::hal::{ClockMicros, InputPin, OutputPin};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RangeReading {
    pub distance_m: f32,
    pub echo_high_us: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UltrasonicError {
    Busy,
    Timeout,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Idle,
    TriggerHigh { since_us: u32 },
    WaitingForEchoRise { since_us: u32 },
    WaitingForEchoFall { since_us: u32 },
}

pub struct Ultrasonic<TRIG, ECHO, CLOCK> {
    trigger: TRIG,
    echo: ECHO,
    clock: CLOCK,
    state: State,
    temperature_c: f32,
    max_echo_wait_us: u32,
    max_echo_high_us: u32,
}

impl<TRIG, ECHO, CLOCK> Ultrasonic<TRIG, ECHO, CLOCK>
where
    TRIG: OutputPin,
    ECHO: InputPin,
    CLOCK: ClockMicros,
{
    pub fn new(trigger: TRIG, echo: ECHO, clock: CLOCK) -> Self {
        Self {
            trigger,
            echo,
            clock,
            state: State::Idle,
            temperature_c: 20.0,
            max_echo_wait_us: 3_000,
            max_echo_high_us: 25_000,
        }
    }

    pub fn set_temperature_c(&mut self, temperature_c: f32) {
        self.temperature_c = temperature_c;
    }

    pub fn start(&mut self) -> Result<(), UltrasonicError> {
        if self.state != State::Idle {
            return Err(UltrasonicError::Busy);
        }

        let now = self.clock.now_us();
        self.trigger.set_high();
        self.state = State::TriggerHigh { since_us: now };
        Ok(())
    }

    pub fn poll(&mut self) -> Result<Option<RangeReading>, UltrasonicError> {
        let now = self.clock.now_us();

        match self.state {
            State::Idle => Ok(None),
            State::TriggerHigh { since_us } => {
                if elapsed_us(since_us, now) >= 10 {
                    self.trigger.set_low();
                    self.state = State::WaitingForEchoRise { since_us: now };
                }
                Ok(None)
            }
            State::WaitingForEchoRise { since_us } => {
                if self.echo.is_high() {
                    self.state = State::WaitingForEchoFall { since_us: now };
                    Ok(None)
                } else if elapsed_us(since_us, now) > self.max_echo_wait_us {
                    self.state = State::Idle;
                    Err(UltrasonicError::Timeout)
                } else {
                    Ok(None)
                }
            }
            State::WaitingForEchoFall { since_us } => {
                let pulse_us = elapsed_us(since_us, now);
                if self.echo.is_low() {
                    self.state = State::Idle;
                    Ok(Some(RangeReading {
                        distance_m: distance_from_echo_us(pulse_us, self.temperature_c),
                        echo_high_us: pulse_us,
                    }))
                } else if pulse_us > self.max_echo_high_us {
                    self.state = State::Idle;
                    Err(UltrasonicError::Timeout)
                } else {
                    Ok(None)
                }
            }
        }
    }

    pub fn release(self) -> (TRIG, ECHO, CLOCK) {
        (self.trigger, self.echo, self.clock)
    }
}

pub fn distance_from_echo_us(echo_high_us: u32, temperature_c: f32) -> f32 {
    let speed_of_sound_mps = 331.3 + 0.606 * temperature_c;
    echo_high_us as f32 * speed_of_sound_mps / 2_000_000.0
}

fn elapsed_us(start: u32, now: u32) -> u32 {
    now.wrapping_sub(start)
}
