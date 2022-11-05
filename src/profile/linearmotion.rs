use crate::{vecx::Vec3, Line3D};
#[allow(unused_imports)]
use num_traits::{Float, Inv};

use super::{
    trapezoidal::{Conditions, Trapezoidal},
    LinearMotionProfile, SCurve, SCurveConstraints, SCurveStartConditions,
};

/// a implement of LinearMotionProfile, it use SCurve profile.
/// notes: it generate profile that not include direction information.
/// it means you should indepdent deal the direction by youself.
pub struct LinearMotionSCurve {
    line: Line3D,
    scurve: SCurve,

    index: f32,
    percent: f32,
}

impl LinearMotionSCurve {
    ///distance unit is S, velocity unit is S/T, acceleration unit is S/T^2.
    /// it dont care about direction. on other word, the algorithem dont care
    /// about sign of velocity, acceleration and distance.
    pub fn new(
        distance: f32,
        steps: &Vec3<i32>,
        max_acceleration: f32,
        max_velocity: f32,
        enter_velocity: f32,
        end_velocity: f32,
    ) -> Self {
        let (distance, steps, max_acceleration, max_velocity, enter_velocity, end_velocity) = (
            distance.abs(),
            steps.abs(),
            max_acceleration.abs(),
            max_velocity.abs(),
            enter_velocity.abs(),
            end_velocity.abs(),
        );

        //////////////////////////////////
        let line = Line3D::new(Vec3::<i32>::new(0, 0, 0), steps);
        #[cfg(test)]
        {
            println!(
                "LinearMotionSCurve input:{:?},max_acc:{},max_v:{},s_v:{},e_v:{}, line_len:{} steps",
                steps, max_acceleration, max_velocity, enter_velocity, end_velocity, line.len(),
            );
        }

        let scurve = {
            let constraints = SCurveConstraints {
                max_jerk: 3.0,
                max_acceleration,
                max_velocity,
            };
            let start_conditions = SCurveStartConditions {
                q0: 0.,
                //  for skipping enter point latelly
                q1: distance,
                v0: enter_velocity,
                v1: end_velocity,
            };
            SCurve::new(&constraints, &start_conditions)
        };
        let percent = scurve.params.time_intervals.total_duration() / (line.len() + 1) as f32;

        Self {
            line,
            scurve,
            //  for skipping enter point latelly, because we need delay for each step, so we need skip the enter point
            index: 1.,
            percent,
        }
    }
}

impl LinearMotionProfile for LinearMotionSCurve {
    fn next_profile(&mut self) -> Option<(Vec3<i32>, f32 /*velocity*/)> {
        if let Some(p) = self.line.next() {
            let velocity = self.scurve.params.eval_velocity(self.index * self.percent);
            self.index += 1.;

            return Some((p, velocity as f32));
        }
        None
    }
}

pub struct LinearMotionTrapezoidal {
    line: Line3D,
    trap: Trapezoidal,
}

impl LinearMotionTrapezoidal {
    #[allow(dead_code)]
    pub fn new(
        steps: &Vec3<i32>,
        target_accel: f32,
        max_velocity: f32,
        enter_velocity: f32,
        end_velocity: f32,
    ) -> Self {
        //////////////////////////////////
        let line = Line3D::new(Vec3::<i32>::new(0, 0, 0), steps.abs());

        let conditions = Conditions {
            enter_velocity,
            end_velocity,
            target_accel,
        };

        let trap = Trapezoidal::new(Some(conditions), Some(max_velocity), line.len() as u32);
        Self { line, trap }
    }
}

impl LinearMotionProfile for LinearMotionTrapezoidal {
    fn next_profile(&mut self) -> Option<(Vec3<i32>, f32 /*velocity*/)> {
        if let Some(delay) = self.trap.next_delay() {
            if let Some(p) = self.line.next() {
                return Some((p, delay.inv()));
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {

    use super::{LinearMotionProfile, LinearMotionSCurve, LinearMotionTrapezoidal, Vec3};
    use num_traits::Inv;

    #[test]
    fn stack_new_scurve() {
        let enter_velocity: f32 = 48.0;
        let end_velocity: f32 = 0.0;
        let max_acceleration: f32 = 360000.;
        let max_velocity: f32 = 48.0;
        let steps: Vec3<i32> = Vec3::new(200, 700, 0);

        let mut linear = LinearMotionSCurve::new(
            700.,
            &steps,
            max_acceleration,
            max_velocity,
            enter_velocity,
            end_velocity,
        );

        let total_duration = linear.scurve.params.time_intervals.total_duration();
        println!("total duration:{duration}", duration = total_duration);

        let mut all = Vec::<f32>::new();
        let mut all_inv = Vec::<f32>::new();
        let mut sum: f32 = 0.;
        while let Some(prf) = linear.next_profile() {
            all.push(prf.1);

            sum += prf.1.inv();
            if sum <= total_duration {
                all_inv.push(prf.1.inv());
            }
            // println!("{:?}, v:{}", prf.0, prf.1);
        }
        println!(
            "velocity sequence:{v:?},size:{size}",
            v = &all[0..all_inv.len()],
            size = all_inv.len()
        );
        println!(
            "/r/n left velocity sequence:{v:?},size:{size}",
            v = &all[all_inv.len()..all.len()],
            size = all.len() - all_inv.len()
        );
        // println!(
        //     "delay sequence:{v:?},size:{size}",
        //     v = all_inv,
        //     size = all_inv.len()
        // );
    }

    // #[test]
    // fn stack_new_Trapezoidal() {
    //     let enter_velocity: f32 = 20.0;
    //     let end_velocity: f32 = 0.0;
    //     let target_accel: f32 = 21.5;

    //     // Vec3(1, 1, 1), v:20.00594
    //     // Vec3(2, 2, 2), v:18.908373
    //     // Vec3(3, 3, 3), v:17.745014
    //     // Vec3(4, 4, 4), v:16.50282
    //     // Vec3(5, 5, 5), v:15.16395
    //     // Vec3(6, 6, 6), v:13.703016
    //     // Vec3(7, 7, 7), v:12.082024
    //     // Vec3(8, 8, 8), v:10.240526
    //     // Vec3(9, 9, 9), v:8.075691
    //     // Vec3(10, 10, 10), v:5.4101486

    //     let max_velocity: f32 = 20.0;
    //     let steps: Vec3<i32> = Vec3::new(10, 10, 10);

    //     let mut linear = LinearMotionTrapezoidal::new(
    //         &steps,
    //         target_accel,
    //         max_velocity,
    //         enter_velocity,
    //         end_velocity,
    //     );

    //     while let Some(_prf) = linear.next_profile() {
    //         // println!("{:?}, v:{}", prf.0, prf.1.inv());
    //     }
    // }
}
