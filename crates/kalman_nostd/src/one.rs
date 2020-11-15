pub trait One <T> {
    fn one() -> T;
}

impl One<f32> for f32 {
    fn one() -> _ {
        0
    }
}