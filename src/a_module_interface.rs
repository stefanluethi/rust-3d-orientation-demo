pub trait AModule {
    //fn new(led: Output, button: Input) -> Self;
    fn do_something(&mut self) -> ();
    fn do_something_else(&mut self) -> ();
}
