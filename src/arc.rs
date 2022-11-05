///! circle interpolation
///!  satisfy rs274ngc arc requirements
///!
use super::vecx::{Vec2, Vec3};

mod config {
    /// The arc G2/3 g-code standard is problematic by definition. Radius-based arcs have horrible numerical
    /// errors when arc at semi-circles(pi) or full-circles(2*pi). Offset-based arcs are much more accurate
    /// but still have a problem when arcs are full-circles (2*pi). This define accounts for the floating
    /// point issues when offset-based arcs are commanded as full circles, but get interpreted as extremely
    /// small arcs with around machine epsilon (1.2e-7rad) due to numerical round-off and precision issues.
    /// This define value sets the machine epsilon cutoff to determine if the arc is a full-circle or not.
    /// NOTE: Be very careful when adjusting this value. It should always be greater than 1.2e-7 but not too
    /// much greater than this. The default setting should capture most, if not all, full arc error situations.
    /// note: "num_traits::Float::epsilon()" is `const EPSILON: f32 = 1.19209290e-07_f32;`
    pub const ARC_ANGULAR_TRAVEL_EPSILON: f32 = 5e-07_f32; // Float (radians)

    /// Number of arc generation iterations by small angle approximation before exact arc trajectory
    /// correction with expensive sin() and cos() calcualtions. This parameter maybe decreased if there
    /// are issues with the accuracy of the arc generations, or increased if arc execution is getting
    /// bogged down by too many trig calculations.
    pub const ARC_DEFAULT_N_CORRECTION: i32 = 12; // Integer (1-255)

    pub const ARC_DEFAULT_TOLERANCE: f32 = 0.0002_f32; //mm
}

/// store information needed by when generating next point in arc
#[allow(dead_code)]
struct CalcInfo {
    // in radius mode, it is None
    //"[Arc definition error] > 0.5mm", "[Arc definition error] > 0.005mm AND 0.1% radius"
    /// ccw angle between vec(from center to current) and vec(from center to target)
    pub angular_travel: f32,
    /// how many segments.
    pub segments: u32,
    /// angle of each segment
    pub theta_per_segment: f32,
    /// value of linear axis
    pub linear_per_segment: f32,
    /// help to calculate next point in arc
    pub sin_t: f32,
    /// help to calculate next point in arc
    pub cos_t: f32,
    /// store how many sengment has output
    cnt_segments: u32,
    /// help to sotre completed
    completed: bool,
    /// store temporary sub-level
    count: i32,
}

impl CalcInfo {
    pub fn new(
        angular_travel: f32,
        segments: u32,
        theta_per_segment: f32,
        linear_per_segment: f32,
        sin_t: f32,
        cos_t: f32,
    ) -> Self {
        Self {
            angular_travel,
            segments,
            theta_per_segment,
            linear_per_segment,
            sin_t,
            cos_t,

            cnt_segments: 1,
            count: 0,
            completed: false,
        }
    }
}

/// Arc satisfy rs274ngc arc requirements.
///
/// ARC_TOLERANCE unis is um, "1000um=1mm", radius and position fields is in mm unit
pub struct Arc {
    /// orig current position
    orig_c: Vec3<f32>,
    /// T - Target position
    t: Vec3<f32>,
    /// center- circle center postion, the circle that pass through both C and T
    center: Vec2<f32>,
    /// r - designated radius, it store in positive
    r: f32,
    /// true means counter clockwise direction, otherwise is clockwise direction
    turn_ccw: bool,

    /// C - last Current position, it will modify when generate next point
    c: Vec3<f32>,
    info: Option<CalcInfo>,
}

impl Arc {
    /// Arc Radius Mode
    /// current positon and target positon represent with plane and linear
    /// r is radius,
    /// trurn_ccw is true means trun in counter clockwise direction, otherwise means clockwise
    ///
    #[allow(dead_code)]
    pub fn new_radius_mode(
        current: Vec3<f32>,
        target: Vec3<f32>,
        radius: f32,
        turn_ccw: bool,
    ) -> Self {
        // vec from current to target
        let t = target.plane() - current.plane();
        // Calculate the change in position along each selected axis
        let x = t.0; // Delta axis0 between current position and target
        let y = t.1; // Delta axis1 between current position and target
        let r = num_traits::Float::abs(radius);

        /*
            d^2 == x^2 + y^2
            h^2 == r^2 - (d/2)^2
            i == x/2 - y/d*h
            j == y/2 + x/d*h

                                                                 O <- [i,j]
                                                              -  |
                                                    r      -     |
                                                        -        |
                                                     -           | h
                                                  -              |
                                    [0,0] ->  C -----------------+--------------- T  <- [x,y]
                                              | <------ d/2 ---->|

            C - Current position
            T - Target position
            O - center of circle that pass through both C and T
            d - distance from C to T
            r - designated radius
            h - distance from center of CT to O

            Expanding the equations:

            d -> sqrt(x^2 + y^2)
            h -> sqrt(4 * r^2 - x^2 - y^2)/2
            i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2
            j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2

            Which can be written:

            i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2
            j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2

            Which we for size and speed reasons optimize to:

            h_x2_div_d = sqrt(4 * r^2 - x^2 - y^2)/sqrt(x^2 + y^2)
            i = (x - (y * h_x2_div_d))/2
            j = (y + (x * h_x2_div_d))/2

            notice, in cw modal, should be :
            i = (x + (y * h_x2_div_d))/2
            j = (y - (x * h_x2_div_d))/2
        */
        let h_x2_div_d = 4.0 * r * r - x * x - y * y;
        let mut h_x2_div_d = num_traits::Float::sqrt(h_x2_div_d) / num_traits::Float::hypot(x, y);
        // Invert the sign of h_x2_div_d if the circle is clockwise
        if turn_ccw == false {
            h_x2_div_d = -h_x2_div_d;
        }
        // Negative R is g-code-alese for "I want a circle with more than 180 degrees of travel" (go figure!),
        // even though it is advised against ever generating such circles in a single line of g-code. By
        // inverting the sign of h_x2_div_d the center of the circles is placed on the opposite side of the line of
        // travel and thus we get the unadvisably long arcs as prescribed.
        if radius.is_sign_negative() {
            h_x2_div_d = -h_x2_div_d;
        }
        // Complete the operation by calculating the actual center of the arc, there are offset to current position
        let o_i = 0.5 * (x - (y * h_x2_div_d));
        let o_j = 0.5 * (y + (x * h_x2_div_d));
        let o = Vec2::new(o_i, o_j);
        let center = current.plane() + o;

        Self {
            orig_c: current,
            c: current,
            t: target,
            center,
            r,
            turn_ccw,
            info: None,
        }
    }

    /// Arc center Mode
    /// current positon and target positon represent with  plane and linear
    /// center_offset is offset from the current position. represent with axis0 and axis 1.  
    /// the center of circle that pass through both C and T.
    /// trurn_ccw is true means trun in counterclockwise direction, otherwise means clockwise
    ///
    pub fn new_center_mode(
        current: Vec3<f32>,
        target: Vec3<f32>,
        center_offset: Vec2<f32>,
        turn_ccw: bool,
    ) -> Self {
        let center = current.plane() + center_offset;
        // rt vector from center to target
        // let t = target.plane() - center;
        // // // Arc radius from center to target
        // // let target_r = t.distance();
        // arc radius current to center.
        let r = center_offset.distance();

        Self {
            orig_c: current,
            t: target,
            center,
            r,
            turn_ccw,
            c: current,
            info: None,
        }
    }

    /// according current and center_offset, output diff_r to help checking whether target is valid or not
    ///
    /// in rs274ngc.pdf "3.5.3.2 Center Format Arc" defined:
    /// It is an error if: when the arc is projected on the selected plane, the distance from the current point to
    /// the center differs from the distance from the end point to the center by more than
    /// 0.0002 inch (if inches are being used) or 0.002 millimeter (if millimeters are being used).
    ///
    /// in grbl defined:
    /// " [Arc definition error] > 0.5mm", and "[Arc definition error] > 0.005mm AND 0.1% radius"
    #[allow(dead_code)]
    pub fn check_center_mode(
        current: Vec3<f32>,
        target: Vec3<f32>,
        center_offset: Vec2<f32>,
    ) -> f32 {
        let center = current.plane() + center_offset;
        // rt vector from center to target
        let t = target.plane() - center;
        // Arc radius from center to target
        let target_r = t.distance();
        // arc radius current to center.
        let r = center_offset.distance();

        // // Compute difference between current location and target radii for final error-checks.
        let delta_r = num_traits::Float::abs(target_r - r);

        delta_r
    }

    /// ccw angle between current-postion and target-position from circle center
    ///
    /// The included radian is given by ccw rule regardless of the actual rotation direction.
    /// for example, actual rotation is cw, then the output will be a negative value
    fn get_radian_travel_by_ccw(&self) -> f32 {
        // Radius vector from center to current location
        let r_v = self.c.plane() - self.center;
        // Radius vector from center to target location
        let rt_v = self.t.plane() - self.center;

        // // CCW angle between position and target from circle center. Only one atan2() trig computation required.
        // float angular_travel = atan2(r_axis0*rt_axis1-r_axis1*rt_axis0, r_axis0*rt_axis0+r_axis1*rt_axis1);
        let mut angular_travel = num_traits::Float::atan2(r_v.wedge(rt_v), r_v.dot(rt_v));

        use core::f32::consts::TAU;
        // Correct atan2 output per direction
        if self.turn_ccw == false {
            //如果圆弧顺时针移动，角度应该是负值，如果计算出的角度为正值，需要在计算出的角度基础上减去2*pi（pi为圆周率）
            if angular_travel >= -config::ARC_ANGULAR_TRAVEL_EPSILON {
                angular_travel -= TAU;
            }
        } else {
            //如果圆弧逆时针移动，角度应该是正值，如果计算出的角度为负值，需要在计算出的角度基础上加上2*pi（pi为圆周率）
            if angular_travel <= config::ARC_ANGULAR_TRAVEL_EPSILON {
                angular_travel += TAU;
            }
        }

        angular_travel
    }

    /// according to cicle's radius(unit mm), arc's angle angular_travel(unit radiam) and
    /// arc_tolerance(unit um) to calculate how many segments.
    ///
    /// the arc_tolerance which is defined to be the maximum normal distance from segment
    /// to the circle when the end points both lie on the circle. its unit is um
    ///
    /// result (segments, angular_travel)
    pub fn get_segments_and_angular_travel(&self) -> (u32, f32) {
        let angular_travel = self.get_radian_travel_by_ccw();
        let radius = self.r;
        /*
                                         O <- [i,j]
                                      -  |
                            r      -     |
                                -        |
                             -           | h
                          -              |
            [0,0] ->  C -----------------+--------------- T  <- [x,y]
                      | <------ k   ---->|
                      | <--------------- d   ------------>|

            C - Current position
            T - Target position
            O - center of circle that pass through both C and T
            d - distance from C to T, k is d/2
            r - designated radius
            h - distance from center of CT to O

            approximate rule: segments is (travel_arc_len / d), d used as the approximate arc_len per sengemt.
            clearly, the d(2k) is less than the real arc_len per sengemt,but the diff is enough little.
            notice it is a approximate rule, so when use it, the end part should be specially deal

            h = r - arc_tolerance
            k^2 == r^2 - (r - arc_tolerance)^2== arc_tolerance(2*r - arc_tolerance)
            segments = (angular_travel * radius) / (2 * k)
        */
        let k = num_traits::Float::sqrt(
            config::ARC_DEFAULT_TOLERANCE * (2.0 * radius - config::ARC_DEFAULT_TOLERANCE),
        );

        let t = num_traits::Float::abs(angular_travel * radius) / k;
        let t = num_traits::Float::floor(0.5 * t) as u32;

        (t, angular_travel)
    }
}

/// get next point in circle
/// circle formular is: (r*cosθ，r*sinθ)
/// presume start point is:(p0,p1)=(r*cosθ，r*sinθ). if ccw δ,the new point will be: next= (r*cos(θ+δ)，r*sin(θ+δ))
/// according cos（α+β）=cosαcosβ-sinαsinβ, cos（α+β）=cosαcosβ-sinαsinβ
/// we can get iteration formula: next = ( p0*cosδ - p1*sinδ , p1*cosδ + p0*sinδ)
#[inline]
fn circle_formular(start: &Vec2<f32>, cos: f32, sin: f32) -> (f32, f32) {
    (start.0 * cos - start.1 * sin, start.1 * cos + start.0 * sin)
}

impl core::iter::Iterator for Arc {
    type Item = Vec3<f32>;

    /// output internal interpolation, not include start point:
    ///
    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        let (segments, angular_travel) = self.get_segments_and_angular_travel();

        let info = self.info.get_or_insert_with(|| {
            let theta_per_segment = angular_travel / (segments as f32);
            let linear_per_segment = (self.t.linear() - self.c.linear()) / (segments as f32);

            // /**
            //  * according taylor :  cosδ= 1 - (δ^2)/2 ;  sinδ= δ - (δ^3)/6
            //  *  because "Δ=2*cosδ= 2 - (δ^2)", so "sinδ= δ - (δ^3)/6=(δ*(6-δ^2))/6 = (δ*(6-(2-Δ))))/6".
            //  *
            //  *  so we use: Δ=2-δ^2 ; cosδ=Δ/2;     sinδ==(δ*(4+Δ))/6
            //  */
            let cos_t_2 = 2.0 - theta_per_segment * theta_per_segment; //to simple, this calculate is double of cos_t
            let sin_t = theta_per_segment * 0.16666667 * (cos_t_2 + 4.0);
            let cos_t = cos_t_2 * 0.5; //

            let value = CalcInfo::new(
                angular_travel,
                segments,
                theta_per_segment,
                linear_per_segment,
                sin_t,
                cos_t,
            );
            value
        });

        if info.completed == true {
            return None;
        } else if info.segments == 0 {
            // Ensure last segment arrives at target location.
            info.completed = true;
            return Some(self.t);
        }

        if info.cnt_segments >= info.segments {
            // Ensure last segment arrives at target location.
            info.completed = true;
            return Some(self.t);
        }

        // Radius vector from center to current location
        // let (mut r_axis0, mut r_axis1) = (0.0_f32, 0.0_f32);
        let r_axis0: f32;
        let r_axis1: f32;

        if info.count < config::ARC_DEFAULT_N_CORRECTION {
            let r_v = self.c.plane() - self.center;
            // use theta_per_segment do iteration to get next point.
            (r_axis0, r_axis1) = circle_formular(&r_v, info.cos_t, info.sin_t);
            info.count += 1;
        } else {
            // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments. ~375 usec
            // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
            let delta = info.cnt_segments as f32 * info.theta_per_segment;
            let cos_ti = num_traits::Float::cos(delta);
            let sin_ti = num_traits::Float::sin(delta);

            let r_v = self.orig_c.plane() - self.center;
            (r_axis0, r_axis1) = circle_formular(&r_v, cos_ti, sin_ti);

            info.count = 0;
        }
        let t = Vec3::new_from_plane(
            self.center + Vec2::new(r_axis0, r_axis1),
            self.c.linear() + info.linear_per_segment,
        );
        //update last current postion
        self.c = t;

        info.cnt_segments += 1;

        Some(t)
    }
}

#[cfg(test)]
mod tests {
    #[allow(unused_imports)]
    use super::{
        config::{ARC_DEFAULT_N_CORRECTION, ARC_DEFAULT_TOLERANCE},
        Arc, Vec2, Vec3,
    };
    // use rand::Rng;
    #[test]
    fn test_center_mode() {
        let start = Vec3::new(0., 0., 0.);
        let target = Vec3::new(4., 4., 4.);
        let center_offset = Vec2::new(0.0, 4.0);
        // let radius = 4.0_f32;
        let turn_ccw = true;
        let arc = Arc::new_center_mode(start, target, center_offset, turn_ccw);
        assert_eq!(arc.r, 4.0);
        let diff = Arc::check_center_mode(start, target, center_offset);
        assert_eq!(diff, 0.0);

        let _center = arc.center.clone();

        //check output include target
        let all: Vec<_> = arc.collect();
        let last = all.last().unwrap();
        assert_eq!(last, &target);

        // // check point in circle path has same distance to center
        // let mut random = rand::thread_rng();
        // let x1 = random.gen_range((0..all.len()));
        // let x2 = random.gen_range((0..all.len()));
        // let d1 = (all[x1].plane() - center).distance();
        // let d2 = (all[x2].plane() - center).distance();

        // let is_eq = |r: f32, l: f32| {
        //     let t = num_traits::Float::abs(r - l);
        //     if t < 2e-07_f32 {
        //         true
        //     } else {
        //         false
        //     }
        // };
        // assert_eq!(true, is_eq(d1, d2));
    }

    /// output image file to check
    #[test]
    fn test_radius_mode() {
        let start = Vec3::new(0., 0., 0.);
        let target = Vec3::new(4., 4., 0.);
        let radius = 4.0_f32;
        let turn_ccw = true;
        let arc = Arc::new_radius_mode(start, target, radius, turn_ccw);
        assert_eq!(arc.center, Vec2::new(0.0, 4.0));

        let start = Vec3::new(1., 1., 0.);
        let target = Vec3::new(5., 5., 0.);
        let radius = 4.0_f32;
        let turn_ccw = false;
        let arc = Arc::new_radius_mode(start, target, radius, turn_ccw);
        assert_eq!(arc.center, Vec2::new(5.0, 1.0));
    }
    // #[test]
    // fn test_line3d_example_2() {
    // }
}
