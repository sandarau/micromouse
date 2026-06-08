pub trait OutputPin {
    fn set_high(&mut self);
    fn set_low(&mut self);
}

pub trait InputPin {
    fn is_high(&self) -> bool;

    fn is_low(&self) -> bool {
        !self.is_high()
    }
}

pub trait PwmPin {
    fn max_duty(&self) -> u16;
    fn set_duty(&mut self, duty: u16);
}

pub trait ClockMicros {
    fn now_us(&self) -> u32;
}
