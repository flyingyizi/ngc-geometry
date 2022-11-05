/// Defines an interface to the square root operation
pub trait Sqrt {
    /// Return the square root of `self`
    fn sqrt(self) -> Self;
}

// /// Defines an interface to the `ceil` operator for rounding up
// pub trait Ceil {
//     /// Round up to the next largest integer
//     fn ceil(self) -> Self;
// }

/// defines some custom traits and provides implementations for `f32`,
mod impl_using_libm {
    impl super::Sqrt for f32 {
        fn sqrt(self) -> Self {
            num_traits::Float::sqrt(self)
            // libm::sqrtf(self)
        }
    }
}
