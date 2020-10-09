// based on https://github.com/japaric/enc28j60/blob/master/src/lib.rs

pub trait AModule<LED, BUTTON> {
    //fn new(led: LED, button: BUTTON) -> Self;
    fn do_something(&mut self) -> ();
    fn do_something_else(&mut self) -> ();
}
