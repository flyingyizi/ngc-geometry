use crate::{CanonPlane, Vec3};
#[allow(unused_imports)]
use num_traits::Float;
/// in plane_XY coordiate, define steps per mm
const DEFAULT_X_STEPS_PER_MM: f32 = 10.;
const DEFAULT_Y_STEPS_PER_MM: f32 = 10.;
const DEFAULT_Z_STEPS_PER_MM: f32 = 10.;

/// Minimum planner junction speed. Sets the default minimum junction speed the planner plans to at
/// every buffer block junction, except for starting from rest and end of the buffer, which are always
/// zero. This value controls how fast the machine moves through junctions with no regard for acceleration
/// limits or angle between neighboring block line move directions. This is useful for machines that can't
/// tolerate the tool dwelling for a split second, i.e. 3d printers or laser cutters. If used, this value
/// should not be much greater than zero or to the minimum value necessary for the machine to work.
pub const MINIMUM_JUNCTION_SPEED: f32 = 0.0; // (mm/min)

/// Sets the minimum feed rate the planner will allow. Any value below it will be set to this minimum
/// value. This also ensures that a planned motion always completes and accounts for any floating-point
/// round-off errors. Although not recommended, a lower value than 1.0 mm/min will likely work in smaller
/// machines, perhaps to 0.1mm/min, but your success may vary based on multiple factors.
pub const MINIMUM_FEED_RATE: f32 = 1.0; // (mm/min)
pub const DEFAULT_JUNCTION_DEVIATION: f32 = 0.01; // mm

/// unit mm/min
const DEFAULT_X_MAX_RATE: f32 = 500.0;
/// unit mm/min
const DEFAULT_Y_MAX_RATE: f32 = 500.0;
/// unit mm/min
const DEFAULT_Z_MAX_RATE: f32 = 500.0;
/// 10*60*60 mm/min^2 = 10 mm/sec^2
const DEFAULT_X_ACCELERATION: f32 = 10.0 * 60. * 60.;
/// 10*60*60 mm/min^2 = 10 mm/sec^2
const DEFAULT_Y_ACCELERATION: f32 = 10.0 * 60. * 60.;
/// 10*60*60 mm/min^2 = 10 mm/sec^2
const DEFAULT_Z_ACCELERATION: f32 = 10.0 * 60. * 60.;

pub const DEFAULT_X_MAX_TRAVEL: f32 = 400.0; // mm NOTE: Must be a positive value.
pub const DEFAULT_Y_MAX_TRAVEL: f32 = 300.0; // mm NOTE: Must be a positive value.
pub const DEFAULT_Z_MAX_TRAVEL: f32 = 500.0; // mm NOTE: Must be a positive value.
pub const DEFAULT_A_MAX_TRAVEL: f32 = 360.0; // °
pub const DEFAULT_B_MAX_TRAVEL: f32 = 360.0; // °
pub const DEFAULT_HOMING_FEED_RATE: f32 = 50.0; // mm/min
pub const DEFAULT_HOMING_SEEK_RATE: f32 = 500.0; // mm/min

///Rapids override value in percent. 1.0 represent 100%
pub const DEFAULT_RAPID_OVERRIDE: f32 = 1.; // 100%. Don't change this value.

/// based on unit vector with plane convert a mm(unit is mm) value to a value that unit is step.
///
/// it used in scenarios, e.g. convert distance,velocity that unit is (mm/T) to distance, velocity that unit is (step/T).
pub fn mm_to_steps(unit_vec: &Vec3<f32>, plane: &CanonPlane, mm: f32) -> f32 {
    //confirm input is unit vector
    assert!((unit_vec.distance() - 1.).abs() < 1.0e-5);

    if mm == 0. {
        return 0.;
    }

    let t = unit_vec * mm;
    let t = mm_pos_to_step_pos(&t, plane);

    // to avoid overflow, temporary convert to unit meter, at last, convert back.
    let (a, b, c) = (t.0 as f32 / 1000., t.1 as f32 / 1000., t.2 as f32 / 1000.);
    let length = (a * a + b * b + c * c).sqrt();
    length * 1000.
}

/// the plane is the plane that the represent orig data
pub fn step_pos_to_mm_pos(orig: &Vec3<i32>, plane: &CanonPlane) -> Vec3<f32> {
    let orig_plane = plane;
    let dest_plane = &CanonPlane::CanonPlaneXY;

    let d = orig_plane.to_plane(orig, dest_plane);
    let dest = Vec3::<f32>::new(
        d.0 as f32 / DEFAULT_X_STEPS_PER_MM,
        d.1 as f32 / DEFAULT_Y_STEPS_PER_MM,
        d.2 as f32 / DEFAULT_Z_STEPS_PER_MM,
    );
    if plane == &CanonPlane::CanonPlaneXY {
        return dest;
    }

    let dest_plane = plane;
    let orig_plane = &CanonPlane::CanonPlaneXY;

    let dest = orig_plane.to_plane(&dest, dest_plane);

    dest
}
/// the plane is the plane that the represent orig data
pub fn mm_pos_to_step_pos(orig: &Vec3<f32>, plane: &CanonPlane) -> Vec3<i32> {
    let orig_plane = plane;
    let dest_plane = &CanonPlane::CanonPlaneXY;

    let d = orig_plane.to_plane(orig, dest_plane);
    let dest = Vec3::<i32>::new(
        (d.0 * DEFAULT_X_STEPS_PER_MM).round() as i32,
        (d.1 * DEFAULT_Y_STEPS_PER_MM).round() as i32,
        (d.2 * DEFAULT_Z_STEPS_PER_MM).round() as i32,
    );
    if plane == &CanonPlane::CanonPlaneXY {
        return dest;
    }

    let dest_plane = plane;
    let orig_plane = &CanonPlane::CanonPlaneXY;

    let dest = orig_plane.to_plane(&dest, dest_plane);
    dest
}

/// get max information
pub trait MaxByAxisTrait<T> {
    /// self as the direction vecotr, get the max veclocity in this direction
    fn get_max_velocity(&self) -> f32;
    /// self as the direction vector, get the max acceleration in this direction
    fn get_max_acc(&self) -> f32;
}

impl<'a> MaxByAxisTrait<&'a Vec3<f32>> for &'a Vec3<f32> {
    fn get_max_velocity(&self) -> f32 {
        let unit_vec = self.as_unit_vec();
        return limit_value_by_axis_maximum(
            DEFAULT_X_MAX_RATE,
            DEFAULT_Y_MAX_RATE,
            DEFAULT_Z_MAX_RATE,
            &unit_vec,
        );
    }
    fn get_max_acc(&self) -> f32 {
        let unit_vec = self.as_unit_vec();
        return limit_value_by_axis_maximum(
            DEFAULT_X_ACCELERATION,
            DEFAULT_Y_ACCELERATION,
            DEFAULT_Z_ACCELERATION,
            &unit_vec,
        );
    }
}

///input: max_value is  restraint on each Axial. unit_vec is unit vector define the direction.
///
///  result is the restraint on the direction
fn limit_value_by_axis_maximum(max_x: f32, max_y: f32, max_z: f32, unit_vec: &Vec3<f32>) -> f32 {
    // according vector projection
    let (x, y, z) = {
        // if is zero , means there is no restraint on this axial
        let _x = if unit_vec.0 == 0. {
            f32::MAX
        } else {
            (max_x / unit_vec.0).abs()
        };
        let _y = if unit_vec.1 == 0. {
            f32::MAX
        } else {
            (max_y / unit_vec.1).abs()
        };
        let _z = if unit_vec.2 == 0. {
            f32::MAX
        } else {
            (max_z / unit_vec.2).abs()
        };
        (_x, _y, _z)
    };

    x.min(y).min(z)
}
