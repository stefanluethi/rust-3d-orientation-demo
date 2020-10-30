#![deny(unsafe_code)]
#![no_main]
#![no_std]

// Halt on panic
#[allow(unused_extern_crates)] // NOTE(allow) bug rust-lang/rust#53964
//extern crate panic_halt; // panic handler
extern crate panic_itm;
//use panic_rtt_target as _;

use cortex_m;
use cortex_m_rt::entry;
use stm32f4xx_hal as hal;
use crate::hal::{prelude::*, stm32};
use hal::time::KiloHertz;

mod a_module;
use a_module::Stm32Io;
mod a_module_interface;
use a_module_interface::AModule;

use embedded_hal::digital::v2::{InputPin, OutputPin};
use stm32f4xx_hal::stm32::I2C1;
use stm32f4xx_hal::stm32::i2c1::cr1::SMBUS_A::I2C;
use embedded_hal::spi::Mode;

extern crate lsm6dso;
use lsm6dso::{Lsm6dso, SlaveAddr, RawAccelerometer, Error, Accelerometer};


#[entry]
fn main() -> ! {
    //rtt_target::rtt_init_print!();

    let mut dp = stm32::Peripherals::take().unwrap();
    let mut cp = cortex_m::peripheral::Peripherals::take().unwrap();

    /* delay */
    // Set up the system clock. We want to run at 48MHz for this one.
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(48.mhz()).freeze();

    // Create a delay abstraction based on SysTick
    let mut delay = hal::delay::Delay::new(cp.SYST, clocks);

    /* itm output */
    let stim = &mut cp.ITM.stim[1];

    /* GPIOs */
    let gpioa = dp.GPIOA.split();
    let mut led = gpioa.pa5.into_push_pull_output();

    let gpioc = dp.GPIOC.split();
    let button = gpioc.pc13.into_floating_input();

    /* sensors */
    let gpiob = dp.GPIOB.split();
    let sda = gpiob.pb9.into_alternate_af4_open_drain();
    let scl = gpiob.pb8.into_alternate_af4_open_drain();
    let i2c = hal::i2c::I2c::i2c1(
        dp.I2C1,
        (scl, sda),
        KiloHertz(400),
        clocks
    );

    cortex_m::iprintln!(stim, "init. sensor start");
    let mut acc = Lsm6dso::new(
        i2c,
        SlaveAddr::Alternate
    ).expect("LSM3DSO could not be initalized");
    cortex_m::iprintln!(stim, "init. sensor finished");
    acc.set_range(lsm6dso::Range::G8);

    let mut my_object = Stm32Io::new(led, button);

    loop {
        my_object.do_something();
        let acceleration = acc.accel_norm().unwrap();
        cortex_m::iprintln!(
            stim,
            "{{\
                \"meas\":\"acc\",\
                \"values\":[\
                    \"x\":{},\
                    \"y\":{},\
                    \"z\":{}\
                ],\
                \"unit\":\"g\"\
            }}",
            acceleration.x,
            acceleration.y,
            acceleration.z
        );
        delay.delay_ms(500_u16);
    }

    loop {}
}