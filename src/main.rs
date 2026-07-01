#![no_std]
#![no_main]
//kHz and MHz are showing red squiggly lines. gone back and forth for 2hrs before i checked github for similar issues, saw that another way of using the hse was by "time". tried it and vs code recommended i type "cargo add time" and suddenly every single error is gone. fuck.
use core::panic::PanicInfo;

use cortex_m_rt::entry;

use stm32f1xx_hal::{pac, prelude::*};
use stm32f1xx_hal::timer::Timer;
use embedded_hal::delay::DelayNs;// gives SysDelay its .delay_ms () methods

use crate::motor::DcMotor;
use crate::motor::MotorConfig;
use crate::motor::MotorControl; // trait that set_target_normalized/update live on
use crate::motor::ZeroBehavior;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}
// tests for cortex entry. ignore
    // prelude::*,//.split(), .constrain(). low-level pac::RCC implements Deref<Target = pac::rcc::RegisterBlock>, the compiler is trying to resolve .cfgr as a method call on that underlying private register layout rather than treating it as a public field of my high-level HAL wrapper.
    /* timer::Channel
    timer::Counter */ 
    // Tim4NoRemap // would bring the Tim4NoRemap type into scope, which is needed for the pwm_hz method; also targets channel1

// prelude::* brings the U32Ext trait into scope, which provides the kHz and MHz methods for u32 values
// prelude::* also has delay_ms trait
// which is also pulled in from embedded_hal::delay::DelayNs, and it becomes ambiguous hence some issues
// maybe the IDE (vs code rust analyzer) is analyzing the file for my laptop's system architecture, which is not ARM, so it doesn't recognize the kHz and MHz methods. But when I compile the code for the STM32F103C8T6, it should work fine.
// fix by creating a config.toml with target thumbv7m-none-eabi to force the IDE to analyze the code for the ARM architecture
// analyzer/IDE noise or conflicts with HAL's own time types...may be the reason rcc.cfgr is stubborn

// wrapped hal initialization in hal.rs
mod motor;
mod hal;
mod pid; //contains clamp function

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = pac::CorePeripherals::take().unwrap();

    // order is quite important. constrain flash and rcc first

    // Constrain the raw RCC into a HAL Rcc struct so .cfgr works
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain(); // creates the HAL wrapper
    // rcc must be mutable to access public cfgr
    // constrain yields an Rcc struct containing a cfgr field
    // now rcc.cfgr is a public hal builder, not a raw reg block

    // Rcc::freeze(self, cfgr, acr) -> Self
    // this takes 'self' by aluse. it consumes rcc and hands back a new Rcc with clock config baked in.
    // but the og rcc binding is now dead and each line that does &mut rcc is reaching for a variable that doesnt have a value; "borrow of moved value error"
    // og was: let clock = rcc.freeze()
    // capture freeze's return val back into rcc to shadow it
    // then replace clocks.clocks with rcc.clocks

    let mut rcc = rcc.freeze(
        stm32f1xx_hal::rcc::Config::default()
            .use_hse(8.MHz())// .MHz extenstion trait is implemented for u32 via fugit's RateExtU32
            .sysclk(72.MHz()),
        &mut flash.acr,
    );// this returns an Rcc struct with a ".clocks: Clocks" field, not a freq val
        
        //.cfgr() method doesnt exist on the Rcc wrapper hence why its an issue
    // going back n forth with removing the () to see if it'll work; with: becomes a method call. without: a config property field

    let mut afio = dp.AFIO.constrain(&mut rcc);
    let mut gpiob = dp.GPIOB.split(&mut rcc);
    let mut gpioa = dp.GPIOA.split(&mut rcc);


    // pwm using tim4 ch1 pwm output on pb6
    let pb6 = gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl);
    let mut pwm = dp.TIM4.pwm_hz(
        pb6, 
        &mut afio.mapr, 
        20.kHz(),
        &mut rcc
    );// compiler transforms freq rate into period duration
    // was having issues with stm32f4xx_hal dependency leaking into the project
    // pwm_hz is what i was using, and its native to the stm32f4xx_hal API
    // correct method is .pwm()
    // also, tim4 only implements the right pwm traits when passed as a pin configured as an alternate function push-pull pin

    pwm.enable(stm32f1xx_hal::timer::Channel::C1);
    let left_motor_pwm = pwm.split(); // channel 1 is the first element of the tuple returned by split()

    //dir pins for ln298
    let motor_in1 = gpioa.pa1.into_push_pull_output(&mut gpioa.crl);
    let motor_in2 = gpioa.pa2.into_push_pull_output(&mut gpioa.crl);

    // motor config. the struct. maxduty is grabbed auomatically if left at 0
    //ramping vals: 500 units per sec means 2sec to reach full speed of 1k
    let config = MotorConfig::new(0, 500.0, 1000.0, ZeroBehavior::Coast);
    let mut left_motor = DcMotor::new(left_motor_pwm, motor_in1, motor_in2, config);

    let mut delay = Timer::syst(cp.SYST, &rcc.clocks).delay();

    //for testing:
    loop {
        left_motor.set_target_normalized(0.5);//half speed
        for _ in 0..200 {
            left_motor.update(0.01);// 10ms step time
            embedded_hal::delay::DelayNs::delay_ms(&mut delay, 10u32);// wait 10ms before next update; it uses syst (hardware timer) to block execution for 10ms
        }

        //hold at half speed for 2 sec
        for _ in 0..200 {
            left_motor.update(0.01);
            embedded_hal::delay::DelayNs::delay_ms(&mut delay, 10u32);
        }

        //stop gradually
        left_motor.stop_gradual();
        for _ in 0..200 {
            left_motor.update(0.01);
            embedded_hal::delay::DelayNs::delay_ms(&mut delay, 10u32);
        }

        embedded_hal::delay::DelayNs::delay_ms(&mut delay, 2000u32); //wait 2sec b4 repeating
    }
}
// use the command: cargo build --release --features stm32
// to explicitly as for the stm32 feature
// it tells cargo to look at cargo.toml and find the [features] section, and then look for the stm32 feature, and then look for the dependencies that are listed under that feature, and then build those dependencies as well. This is necessary because the stm32f1xx-hal crate is not a default dependency, so it needs to be explicitly included in the build process.