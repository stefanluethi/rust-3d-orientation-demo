/// Example for trait abstraction
///
/// Maps a button to a led using embedded-hal instead of the specific
/// stm32f4-hal.
use crate::button_led_handler::ButtonLedHandler;

extern crate embedded_hal as hal;
use hal::digital::v2::{InputPin, OutputPin};

/// Data structure
pub struct ButtonLedHandlerImplementation<Led, Button> {
    led: Led,
    button: Button,
}

/// Methods for our data structure
impl <Led, Button> ButtonLedHandlerImplementation<Led, Button> {
    pub fn new(led: Led, button: Button) -> Self {
        ButtonLedHandlerImplementation {
            led,
            button
        }
    }
}

#[allow(dead_code)]
/// Methods defined in ButtonLedHandler implemented for our data structure
impl <Led, Button> ButtonLedHandler for ButtonLedHandlerImplementation<Led, Button>
where
    Led: OutputPin,
    Button: InputPin,
{
    fn map_button_to_led(&mut self) -> () {
        if match self.button.is_low() {
            Ok(value) => value,
            Err(_) => false,
        } {
            self.led.set_high().unwrap_err();
        } else {
            self.led.set_low().unwrap_err();
        }
    }

    fn do_something_else(&mut self) -> () {
        unimplemented!()
    }
}