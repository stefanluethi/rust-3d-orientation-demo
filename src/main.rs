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
use stm32f4xx_hal::rcc::Clocks;
use embedded_hal::spi::Mode;

use lsm6dso::{Lsm6dso, SlaveAddr, Accelerometer};
use lis2mdl::{Lis2mdl, Magnetometer};
use stts751::{Stts751, Temperature};

use core::fmt::Write;

#[entry]
fn main() -> ! {
    //rtt_target::rtt_init_print!();

    let mut dp = stm32::Peripherals::take().expect("cannot take stm32 peripherals");
    let mut cp = cortex_m::peripheral::Peripherals::take().expect("cannot take cortex peripherals");

    /* delay */
    // Set up the system clock. We want to run at 48MHz for this one.
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(48.mhz()).freeze();

    // Create a delay abstraction based on SysTick
    let mut delay = hal::delay::Delay::new(cp.SYST, clocks);

    /* gpio's */
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    /* uart */
    let txd = gpioa.pa2.into_alternate_af7();
    let rxd = gpioa.pa3.into_alternate_af7();
    let mut serial = hal::serial::Serial::usart2(
        dp.USART2,
        (txd, rxd),
        hal::serial::config::Config::default().baudrate(115_200.bps()),
        clocks
    ).unwrap();
    let (mut tx, _rx) = serial.split();

    /* itm output */
    let stim = &mut cp.ITM.stim[1];

    /* GPIOs */
    let mut led = gpioa.pa5.into_push_pull_output();
    let button = gpioc.pc13.into_floating_input();

    /* sensors */
    let sda = gpiob.pb9.into_alternate_af4_open_drain();
    let scl = gpiob.pb8.into_alternate_af4_open_drain();
    let i2c = hal::i2c::I2c::i2c1(
        dp.I2C1,
        (scl, sda),
        KiloHertz(100),
        clocks
    );

    cortex_m::iprintln!(stim, "init. sensor start");
    // let mut acc = Lsm6dso::new(
    //     i2c,
    //     SlaveAddr::Alternate
    // ).expect("LSM3DSO could not be initialized");
    // cortex_m::iprintln!(stim, "init. sensor finished");
    // acc.set_range(lsm6dso::Range::G2);

    // todo: not responding i2c bus
    // let mut mag = Lis2mdl::new(i2c)
    //     .expect("LIS2MDL could not be initialized");
    // cortex_m::iprintln!(stim, "init. sensor finished");

    let mut sensor_temp = Stts751::new(i2c)
        .expect("STTS751 could not be initialized");
    cortex_m::iprintln!(stim, "init. sensor finished");

    let mut my_object = Stm32Io::new(led, button);

    loop {
        my_object.do_something();
        // let acceleration = acc.accel_norm().unwrap();
        // writeln!(tx,
        //     "{{\
        //         \"meas\":\"acc\",\
        //         \"values\":{{\
        //             \"x\":{},\
        //             \"y\":{},\
        //             \"z\":{}\
        //         }},\
        //         \"unit\":\"g\"\
        //     }}",
        //     acceleration.x,
        //     acceleration.y,
        //     acceleration.z
        // ).unwrap();

        // let magnetic_field = mag.mag_norm().unwrap();
        // writeln!(tx,
        //     "{{\
        //         \"meas\":\"mag\",\
        //         \"values\":{{\
        //             \"x\":{},\
        //             \"y\":{},\
        //             \"z\":{}\
        //         }},\
        //         \"unit\":\"gauss\"\
        //     }}",
        //      magnetic_field.x,
        //      magnetic_field.y,
        //      magnetic_field.z
        // ).unwrap();

        let temp = sensor_temp.temp_norm().unwrap();
        writeln!(tx,
            "{{\
                \"meas\":\"temp\",\
                \"values\":{{\
                    \"T\":{}\
                }},\
                \"unit\":\"°C\"\
            }}",
             temp
        ).unwrap();
        delay.delay_ms(50_u16);
    }
}