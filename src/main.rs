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

mod a_module;
use a_module::Stm32Io;

mod a_module_interface;
use a_module_interface::AModule;
use embedded_hal::digital::v2::{InputPin, OutputPin};

extern crate lsm6dso;

#[entry]
fn main() -> ! {
    //rtt_target::rtt_init_print!();

    let dp = stm32::Peripherals::take().unwrap();
    let mut cp = cortex_m::peripheral::Peripherals::take().unwrap();

    let gpioa = dp.GPIOA.split();
    let mut led = gpioa.pa5.into_push_pull_output();

    let gpioc = dp.GPIOC.split();
    let button = gpioc.pc13.into_floating_input();

    // Set up the system clock. We want to run at 48MHz for this one.
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(48.mhz()).freeze();

    // Create a delay abstraction based on SysTick
    let mut delay = hal::delay::Delay::new(cp.SYST, clocks);

    let mut my_object = Stm32Io::new(led, button);

    let mut stim = &mut cp.ITM.stim[1];

    loop {
        my_object.do_something();
        cortex_m::iprintln!(stim, "Hello, world!");
        delay.delay_ms(500_u16);
    }

    loop {}
}