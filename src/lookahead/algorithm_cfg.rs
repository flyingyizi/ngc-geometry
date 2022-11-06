use crate::{CanonPlane, Vec3};
#[allow(unused_imports)]
use num_traits::Float;

/// Struct which represents the cnc motion algorithm paramenters. normally, you should provide it based on your cnc machine.
/// suggest create yourself based on the its default.
///
/// example
/// ```
/// use ngc_geometry::CNCCfgs;
/// let sample:CNCCfgs = CNCCfgs{
///     x_steps_per_mm: 20.,
///     ..CNCCfgs::default()
/// };
/// ```
#[derive(Clone, Debug)]
pub struct CNCCfgs {
    /// mm note: must be a positive value.
    pub x_max_travel: f32,
    /// mm note: must be a positive value.
    pub y_max_travel: f32, // mm note: must be a positive value.
    /// mm note: must be a positive value.
    pub z_max_travel: f32, // mm note: must be a positive value.
    pub a_max_travel: f32, // °
    pub b_max_travel: f32, // °

    /// in plane_XY coordiate, define steps per mm
    pub x_steps_per_mm: f32,
    pub y_steps_per_mm: f32,
    pub z_steps_per_mm: f32,

    /// Minimum planner junction speed. Sets the default minimum junction speed the planner plans to at
    /// every buffer block junction, except for starting from rest and end of the buffer, which are always
    /// zero. This value controls how fast the machine moves through junctions with no regard for acceleration
    /// limits or angle between neighboring block line move directions. This is useful for machines that can't
    /// tolerate the tool dwelling for a split second, i.e. 3d printers or laser cutters. If used, this value
    /// should not be much greater than zero or to the minimum value necessary for the machine to work.
    pub minimum_junction_speed: f32, // (mm/min)

    /// Sets the minimum feed rate the planner will allow. Any value below it will be set to this minimum
    /// value. This also ensures that a planned motion always completes and accounts for any floating-point
    /// round-off errors. Although not recommended, a lower value than 1.0 mm/min will likely work in smaller
    /// machines, perhaps to 0.1mm/min, but your success may vary based on multiple factors.
    pub minimum_feed_rate: f32, // (mm/min)
    /// unit mm
    pub default_junction_deviation: f32, // mm

    /// unit mm/min
    pub default_x_max_rate: f32,
    /// unit mm/min
    pub default_y_max_rate: f32,
    /// unit mm/min
    pub default_z_max_rate: f32,
    /// 10*60*60 mm/min^2 = 10 mm/sec^2
    pub default_x_acceleration: f32,
    /// 10*60*60 mm/min^2 = 10 mm/sec^2
    pub default_y_acceleration: f32,
    /// 10*60*60 mm/min^2 = 10 mm/sec^2
    pub default_z_acceleration: f32,

    pub default_homing_feed_rate: f32, // mm/min
    pub default_homing_seek_rate: f32, // mm/min

    ///rapids override value in percent. 1.0 represent 100%
    pub default_rapid_override: f32, // 100%. don't change this value.
}

impl core::default::Default for CNCCfgs {
    fn default() -> Self {
        Self {
            x_max_travel: 400.0, // mm note: must be a positive value.
            y_max_travel: 300.0, // mm note: must be a positive value.
            z_max_travel: 500.0, // mm note: must be a positive value.
            a_max_travel: 360.0, // °
            b_max_travel: 360.0, // °

            x_steps_per_mm: 10.,
            y_steps_per_mm: 10.,
            z_steps_per_mm: 10.,

            minimum_junction_speed: 0.0,      // (mm/min)
            minimum_feed_rate: 1.0,           // (mm/min)
            default_junction_deviation: 0.01, // mm
            default_x_max_rate: 500.0,
            default_y_max_rate: 500.0,
            default_z_max_rate: 500.0,
            default_x_acceleration: 10.0 * 60. * 60.,
            default_y_acceleration: 10.0 * 60. * 60.,
            default_z_acceleration: 10.0 * 60. * 60.,

            default_homing_feed_rate: 50.0,  // mm/min
            default_homing_seek_rate: 500.0, // mm/min

            default_rapid_override: 1., // 100%. don't change this value.
        }
    }
}

impl CNCCfgs {
    /// the plane is the plane that the represent orig data
    pub fn step_pos_to_mm_pos(&self, orig: &Vec3<i32>, plane: &CanonPlane) -> Vec3<f32> {
        let orig_plane = plane;
        let dest_plane = &CanonPlane::CanonPlaneXY;

        let d = orig_plane.to_plane(orig, dest_plane);
        let dest = Vec3::<f32>::new(
            d.0 as f32 / self.x_steps_per_mm,
            d.1 as f32 / self.y_steps_per_mm,
            d.2 as f32 / self.z_steps_per_mm,
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
    pub fn mm_pos_to_step_pos(&self, orig: &Vec3<f32>, plane: &CanonPlane) -> Vec3<i32> {
        let orig_plane = plane;
        let dest_plane = &CanonPlane::CanonPlaneXY;

        let d = orig_plane.to_plane(orig, dest_plane);
        let dest = Vec3::<i32>::new(
            (d.0 * self.x_steps_per_mm).round() as i32,
            (d.1 * self.y_steps_per_mm).round() as i32,
            (d.2 * self.z_steps_per_mm).round() as i32,
        );
        if plane == &CanonPlane::CanonPlaneXY {
            return dest;
        }

        let dest_plane = plane;
        let orig_plane = &CanonPlane::CanonPlaneXY;

        let dest = orig_plane.to_plane(&dest, dest_plane);
        dest
    }

    /// based on unit vector with plane convert a mm(unit is mm) value to a value that unit is step.
    ///
    /// it used in scenarios, e.g. convert distance,velocity that unit is (mm/T) to distance, velocity that unit is (step/T).
    pub fn mm_to_steps(&self, unit_vec: &Vec3<f32>, plane: &CanonPlane, mm: f32) -> f32 {
        //confirm input is unit vector
        assert!((unit_vec.distance() - 1.).abs() < 1.0e-5);

        if mm == 0. {
            return 0.;
        }

        let t = unit_vec * mm;
        let t = self.mm_pos_to_step_pos(&t, plane);

        // to avoid overflow, temporary convert to unit meter, at last, convert back.
        let (a, b, c) = (t.0 as f32 / 1000., t.1 as f32 / 1000., t.2 as f32 / 1000.);
        let length = (a * a + b * b + c * c).sqrt();
        length * 1000.
    }

    /// according rapid_rate and MINIMUM_FEED_RATE limit, return vaild velocity.
    #[inline]
    pub fn get_valid_velocity(&self, rate: &f32, rapid_rate: &f32) -> f32 {
        let nominal_speed = rate.min(*rapid_rate);
        nominal_speed.max(self.minimum_feed_rate)
    }

    /// input as the direction vecotr, get the max veclocity in this direction
    pub fn get_max_velocity(&self, input: &Vec3<f32>) -> f32 {
        let unit_vec = input.as_unit_vec();
        return limit_value_by_axis_maximum(
            self.default_x_max_rate,
            self.default_y_max_rate,
            self.default_z_max_rate,
            &unit_vec,
        );
    }
    /// input as the direction vector, get the max acceleration in this direction
    pub fn get_max_acc(&self, input: &Vec3<f32>) -> f32 {
        let unit_vec = input.as_unit_vec();
        return limit_value_by_axis_maximum(
            self.default_x_acceleration,
            self.default_y_acceleration,
            self.default_z_acceleration,
            &unit_vec,
        );
    }

    /// Compute maximum allowable entry speed at junction by centripetal acceleration approximation.
    pub fn calc_max_junction_speed_sqr(
        &self,
        previous_unit_vec: &Vec3<f32>,
        unit_vec: &Vec3<f32>,
    ) -> f32 {
        let junction_cos_theta = -previous_unit_vec.dot(*unit_vec);
        // NOTE: Computed without any expensive trig, sin() or acos(), by trig half angle identity of cos(theta).
        if junction_cos_theta > 0.999999 {
            //  For a 0 degree acute junction, just set minimum junction speed.
            return self.minimum_junction_speed * self.minimum_junction_speed;
        } else if junction_cos_theta < -0.999999 {
            // Junction is a straight line or 180 degrees. Junction speed is infinite.
            return f32::MAX; // SOME_LARGE_VALUE;
        }

        // 法向量
        let junction_unit_vec = (*unit_vec - *previous_unit_vec).as_unit_vec();

        let junction_acceleration = self.get_max_acc(&junction_unit_vec);

        let sin_theta_d2 = (0.5 * (1.0 - junction_cos_theta)).sqrt(); // Trig half angle identity. Always positive.

        let min_junction_speed_sqr = self.minimum_junction_speed.powi(2);
        let vecolicity_sqr =
            (junction_acceleration * self.default_junction_deviation * sin_theta_d2)
                / (1.0 - sin_theta_d2);

        let max_junction_speed_sqr = min_junction_speed_sqr.max(vecolicity_sqr);

        max_junction_speed_sqr
    }
    /// Computes and returns nominal speed based on running condition and override values.
    /// NOTE: All system motion commands, such as homing/parking, are not subject to overrides.
    pub fn get_nominal_speed(
        &self,
        rapid_rate: &f32,
        programmed_rate: &f32,
        is_rapid_motion: bool,
        is_no_feed_override: bool,
    ) -> f32 {
        let mut nominal_speed = *programmed_rate;
        if is_rapid_motion {
            nominal_speed *= self.default_rapid_override;
        } else {
            if false == is_no_feed_override {
                nominal_speed *= self.default_rapid_override;
            }

            nominal_speed = nominal_speed.min(*rapid_rate);
        }

        nominal_speed.max(self.minimum_feed_rate)
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
