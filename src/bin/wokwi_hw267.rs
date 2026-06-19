#![no_std]
#![no_main]

use cortex_m_rt::entry;
use nb::block;
use panic_halt as _;
use stm32f1xx_hal::{pac, prelude::*, rcc::Config, timer::Timer};

#[entry]
fn main() -> ! {
    let core = cortex_m::Peripherals::take().unwrap();
    let device = pac::Peripherals::take().unwrap();

    let mut flash = device.FLASH.constrain();
    let mut rcc = device.RCC.constrain().freeze(
        Config::hse(8.MHz()).sysclk(72.MHz()).pclk1(36.MHz()),
        &mut flash.acr,
    );

    let mut gpioc = device.GPIOC.split(&mut rcc);
    let mut gpioa = device.GPIOA.split(&mut rcc);
    let mut gpiob = device.GPIOB.split(&mut rcc);

    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    // These three pins are just probes for Wokwi's logic analyzer:
    // PA8 simulates a motor PWM signal, PB0/PB1 simulate motor direction pins.
    let mut pwm_probe = gpioa.pa8.into_push_pull_output(&mut gpioa.crh);
    let mut in1 = gpiob.pb0.into_push_pull_output(&mut gpiob.crl);
    let mut in2 = gpiob.pb1.into_push_pull_output(&mut gpiob.crl);

    let mut tick = Timer::syst(core.SYST, &rcc.clocks).counter_hz();
    tick.start(1.kHz()).unwrap();

    let mut pwm_phase: u16 = 0;
    let mut duty_steps: u16 = 0;
    let mut ramp_up = true;
    let mut elapsed_ms: u32 = 0;

    loop {
        block!(tick.wait()).unwrap();
        elapsed_ms = elapsed_ms.wrapping_add(1);

        // PC13 LED is active-low on Blue Pill boards.
        if elapsed_ms % 1000 < 500 {
            led.set_low();
        } else {
            led.set_high();
        }

        // Direction changes every four seconds:
        // 0-2s forward, 2-4s brake/coast-ish, 4-6s reverse, 6-8s stop.
        match (elapsed_ms / 2000) % 4 {
            0 => {
                in1.set_high();
                in2.set_low();
            }
            1 => {
                in1.set_low();
                in2.set_low();
            }
            2 => {
                in1.set_low();
                in2.set_high();
            }
            _ => {
                in1.set_low();
                in2.set_low();
            }
        }

        // Slow software PWM for easy visual inspection in Wokwi.
        // This is not the final motor PWM implementation; timer PWM comes next.
        pwm_phase = (pwm_phase + 1) % 50;
        if pwm_phase < duty_steps {
            pwm_probe.set_high();
        } else {
            pwm_probe.set_low();
        }

        if pwm_phase == 0 {
            if ramp_up {
                duty_steps += 1;
                if duty_steps >= 50 {
                    ramp_up = false;
                }
            } else {
                duty_steps = duty_steps.saturating_sub(1);
                if duty_steps == 0 {
                    ramp_up = true;
                }
            }
        }
    }
}
