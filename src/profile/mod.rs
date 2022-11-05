/// ! step travel profile
mod linearmotion;
mod scurve;
pub mod trapezoidal;

pub use linearmotion::{LinearMotionSCurve, LinearMotionTrapezoidal};
pub use scurve::{SCurve, SCurveConstraints, SCurveStartConditions};

use crate::Vec3;
/// Abstract interface for motion profiles. e.g. using SCureve profile, get related motion profile
///
/// Implemented by all motion profiles in this library. Can be used to
/// write abstract code that doesn't care about the specific motion profile
/// used.
pub trait LinearMotionProfile: Sized {
    /// Return the next step positon and exist-velocity
    ///
    /// Produces the velocity for the next step. The unit of this velocity is
    /// implementation-defined. when no more steps need to betaken, `None` is returned.
    /// on other word, the motion has ended.
    ///
    fn next_profile(&mut self) -> Option<(Vec3<i32>, f32 /*exist-velocity*/)>;
}
