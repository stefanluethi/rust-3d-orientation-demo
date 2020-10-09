use crate::a_module_interface::AModule;

extern crate embedded_hal as hal;
use hal::prelude::*;
use hal::digital::v2::{InputPin, OutputPin};

extern crate stm32f4xx_hal;
use stm32f4xx_hal::{prelude::*, stm32};

use cortex_m;
use cortex_m_rt::entry;

pub struct Stm32Io<LED, BUTTON> {
    led: LED,
    button: BUTTON,
}

impl <LED, BUTTON> Stm32Io<LED, BUTTON> {
    pub fn new(led: LED, button: BUTTON) -> Self {
        Stm32Io {
            led,
            button
        }
    }
}

impl <LED, BUTTON> AModule for Stm32Io<LED, BUTTON>
where
    LED: OutputPin,
    BUTTON: InputPin,
{
    fn do_something(&mut self) -> () {

        if match self.button.is_low() {
            Ok(value) => value,
            Err(_) => false,
        } {
            self.led.set_high();
        } else {
            self.led.set_low();
        }
    }

    fn do_something_else(&mut self) -> () {

    }
}