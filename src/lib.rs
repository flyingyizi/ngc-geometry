//! arc, line and pid algorithms unsed in cnc

#![cfg_attr(not(test), no_std)]
#![cfg_attr(test, feature(test))]

mod arc;
mod line;
mod lookahead;
mod pid;
pub mod profile;
mod traits;
mod vecx;
extern crate alloc;

pub use arc::Arc;
pub use line::{Line2D, Line3D};
pub use lookahead::{CNCCfgs, Direction, PlanBlock, PlanCondition, PlanLineData, Planer};
pub use pid::PID;
pub use vecx::{CanonPlane, Point, Point3, Vec2, Vec3, Vec5};

#[allow(unused_imports)]
use num_traits::Float;

/// after compensaton, the orig end pos will be to the new-end pos
pub fn compensation(
    start: &Vec2<f32>,
    end: &Vec2<f32>,
    radius: f32,
    side_is_left: bool,
) -> Vec2<f32> {
    let (cx, cy) = (start.0, start.1);
    let (px, py) = (end.0, end.1);

    let distance = (px - cx).hypot(py - cy);

    let theta = (radius / distance).acos();

    let alpha = if side_is_left {
        (cy - py).atan2(cx - px) - theta
    } else {
        (cy - py).atan2(cx - px) + theta
    };

    /* reset to end location */
    let cx = px + radius * alpha.cos();
    let cy = py + radius * alpha.sin();

    let end = Vec2::<f32>::new(cx, cy);
    end
}

// pub fn add(left: usize, right: usize) -> usize {
//     left + right
// }

// #[cfg(test)]
// mod tests {
//     use super::*;

//     #[test]
//     fn it_works() {
//         let result = add(2, 2);
//         assert_eq!(result, 4);
//     }
// }
