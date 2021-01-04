/// Example generic interface for hardware access
pub trait ButtonLedHandler {
    fn map_button_to_led(&mut self) -> ();
    fn do_something_else(&mut self) -> ();
}
