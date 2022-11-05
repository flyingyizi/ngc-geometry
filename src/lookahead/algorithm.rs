///! look 2 ahead algorithm
use crate::{
    profile::{LinearMotionProfile, LinearMotionSCurve /* LinearMotionTrapezoidal*/},
    CanonPlane, Vec3,
};

use super::{
    const_cfg::{
        mm_pos_to_step_pos, mm_to_steps, step_pos_to_mm_pos, MaxByAxisTrait,
        DEFAULT_JUNCTION_DEVIATION, DEFAULT_RAPID_OVERRIDE, MINIMUM_FEED_RATE,
        MINIMUM_JUNCTION_SPEED,
    },
    Direction, PlanCondition, PlanLineData,
};

use alloc::collections::vec_deque::VecDeque;
#[allow(unused_imports)]
use num_traits::Float;

/// stores a linear movement. notes, all vectors in it representing data using CanonPlane::CanonPlaneXY plane
#[allow(dead_code)]
#[derive(Copy, Clone, Debug, Default)]
pub struct PlanBlock {
    /// distance that mm as unit. according plan_buffer_line logic, zero length item will not exist
    pub millimeters: f32,

    /// Step count along each axis, flag(+/-) represent direction
    pub steps: Vec3<i32>,

    /// Block condition data to ensure correct execution depending on states and overrides.
    /// Block bitflag variable defining block run conditions. Copied from pl_line_data.
    pub condition: PlanCondition,

    /// Fields used by the motion planner to manage acceleration.
    pub entry_speed_sqr: f32,

    /// Maximum allowable entry speed based on the minimum of junction limit and
    max_entry_speed_sqr: f32,

    /// Axis-limit adjusted line acceleration. Does not change.
    pub acceleration: f32,

    /// Stored rate limiting data used by planner when changes occur.
    max_junction_speed_sqr: f32,

    /// Axis-limit adjusted maximum rate for this block direction in
    rapid_rate: f32,
    pub nominal_speed: f32,

    /// Programmed rate of this block .
    programmed_rate: f32,

    /// Stored spindle speed data used by spindle overrides and resuming methods.
    /// Block spindle speed. Copied from pl_line_data.
    spindle_speed: f32,

    is_sys_motion: bool,
}

impl PlanBlock {
    /// convert to step profile. exist_speed_sqr unit is same as Self's veclocity
    pub fn to_step_profile(
        &self,
        exist_speed_sqr: f32,
    ) -> Option<(impl LinearMotionProfile, (Direction, Direction, Direction))> {
        if self.millimeters == 0. {
            return None;
        }

        #[rustfmt::skip]
        let dirs = (
            if self.steps.0 >= 0 {Direction::Forward}    else {Direction::Backward},
            if self.steps.1 >= 0 {Direction::Forward}    else {Direction::Backward},
            if self.steps.2 >= 0 {Direction::Forward}    else {Direction::Backward},
        );
        // use values that with step unit as input of scurve
        let unit_vec = step_pos_to_mm_pos(&self.steps, &CanonPlane::CanonPlaneXY).as_unit_vec();

        let max_acceleration: f32 =
            mm_to_steps(&unit_vec, &CanonPlane::CanonPlaneXY, self.acceleration);
        let max_velocity: f32 =
            mm_to_steps(&unit_vec, &CanonPlane::CanonPlaneXY, self.nominal_speed);
        let enter_velocity = mm_to_steps(
            &unit_vec,
            &CanonPlane::CanonPlaneXY,
            self.entry_speed_sqr.sqrt(),
        );
        let end_velocity =
            mm_to_steps(&unit_vec, &CanonPlane::CanonPlaneXY, exist_speed_sqr.sqrt());
        let distance = mm_to_steps(&unit_vec, &CanonPlane::CanonPlaneXY, self.millimeters);

        let linear_motion = LinearMotionSCurve::new(
            distance,
            &self.steps,
            max_acceleration,
            max_velocity,
            enter_velocity,
            end_velocity,
        );

        Some((linear_motion, dirs))
    }

    /// Computes and returns block nominal speed based on running condition and override values.
    /// NOTE: All system motion commands, such as homing/parking, are not subject to overrides.
    fn get_nominal_speed(&self) -> f32 {
        let mut nominal_speed = self.programmed_rate;
        if self
            .condition
            .contains(PlanCondition::PL_COND_FLAG_RAPID_MOTION)
        {
            nominal_speed *= DEFAULT_RAPID_OVERRIDE;
        } else {
            if false
                == self
                    .condition
                    .contains(PlanCondition::PL_COND_FLAG_NO_FEED_OVERRIDE)
            {
                nominal_speed *= DEFAULT_RAPID_OVERRIDE;
            }

            nominal_speed = nominal_speed.min(self.rapid_rate);
        }

        nominal_speed.max(MINIMUM_FEED_RATE)
    }
}

/// for planer variable, comparing to current, store the previous plan item information.
/// notes: if the plane item is a sys-motion, it will not affect it.
struct PreviousVar {
    /// The planner position of the tool in absolute steps. Kept separate
    /// from g-code position for movements requiring multiple line motions,
    /// i.e. arcs, canned cycles, and backlash compensation.
    pub steps: Vec3<i32>,
    /// Unit vector of previous path line segment
    pub pl_previous_unit_vec: Vec3<f32>,
    /// Nominal speed of previous path line segment
    pub pl_previous_nominal_speed: f32,
}
impl PreviousVar {
    pub fn zero() -> Self {
        Self {
            steps: Vec3::<i32>::zero(),
            pl_previous_unit_vec: Vec3::<f32>::zero(),
            pl_previous_nominal_speed: 0.,
        }
    }
    pub fn update(&mut self, nominal_speed: f32, unit_vec: &Vec3<f32>, steps: &Vec3<i32>) {
        self.pl_previous_nominal_speed = nominal_speed;
        self.pl_previous_unit_vec = unit_vec.clone();
        self.steps = steps.clone();
    }
}

pub struct Planer {
    block_buffer: VecDeque<PlanBlock>,

    //var
    prevar: PreviousVar,

    /// Points to the first buffer block after the last optimally planned block for normal
    ///  streaming operating conditions.
    block_buffer_planned: Option<usize>,
    // const ,
}

impl Planer {
    #[allow(dead_code)]
    pub fn new() -> Self {
        Self {
            block_buffer: VecDeque::new(),
            prevar: PreviousVar::zero(),
            block_buffer_planned: None,
        }
    }

    pub fn get_previous_steps(&self) -> &Vec3<i32> {
        &self.prevar.steps
    }

    /// Called when the current block(the first element) is no longer needed. Discards the block and makes the memory
    /// availible for new blocks.
    pub fn discard_current_block(&mut self) {
        if let Some(_) = self.block_buffer.pop_front() {
            if let Some(planned) = self.block_buffer_planned {
                if planned == 0 {
                    self.block_buffer_planned = None;
                } else {
                    self.block_buffer_planned = Some(planned - 1);
                }
            }
        }
    }

    /// Returns address of first planner block, and its exist speed sqr.
    pub fn get_current_block(&self) -> Option<(&PlanBlock, f32)> {
        if let Some(v) = self.block_buffer.get(0) {
            let exist_speed_sqr: f32;
            if let Some(v) = self.block_buffer.get(1) {
                exist_speed_sqr = v.entry_speed_sqr;
            } else {
                exist_speed_sqr = 0.;
            }
            Some((v, exist_speed_sqr))
        } else {
            None
        }
    }

    #[allow(dead_code)]
    pub fn len(&self) -> usize {
        self.block_buffer.len()
    }

    pub fn push_normal_motion(
        &mut self,
        target: &Vec3<f32>,
        pl_data: &PlanLineData,
    ) -> Result<(), ()> {
        self.plan_buffer_line(target, pl_data, None)
    }
    pub fn push_sys_motion(
        &mut self,
        target: &Vec3<f32>,
        pl_data: &PlanLineData,
        previsous_steps: &Vec3<i32>,
    ) -> Result<(), ()> {
        self.plan_buffer_line(target, pl_data, Some(previsous_steps))
    }
    /// Add a new linear movement to the planner. target[N_AXIS] is the signed, absolute target position
    /// in millimeters. Feed rate specifies the speed of the motion. If feed rate is inverted, the feed
    /// rate is taken to mean "frequency" and would complete the operation in 1/feed_rate minutes.
    ///
    /// All position data passed to the planner must be in terms of machine position to keep the planner
    /// independent of any coordinate system changes and offsets, which are handled by the g-code parser.
    ///
    /// previsous_steps: if it is none, use the planner himself stored preivious segment's steps as previous steps,
    /// otherwise, use input paramenter.  only SYSTEM_MOTION, need it
    ///
    /// err means the input plan is empty,
    fn plan_buffer_line(
        &mut self,
        target: &Vec3<f32>,
        pl_data: &PlanLineData,
        previsous_steps: Option<&Vec3<i32>>,
    ) -> Result<(), ()> {
        let is_sys_motion = previsous_steps.is_some();

        let target_steps = mm_pos_to_step_pos(target, &CanonPlane::CanonPlaneXY);

        // Prepare and initialize new block. Copy relevant pl_data for block execution.
        let steps = if let Some(st) = previsous_steps {
            target_steps - *st
        } else {
            target_steps - self.prevar.steps
        };

        let (unit_vec, distance) = {
            let vec_millim = step_pos_to_mm_pos(&steps, &CanonPlane::CanonPlaneXY);
            (vec_millim.as_unit_vec(), vec_millim.distance())
        };
        // Bail if this is a zero-length block. Highly unlikely to occur.
        if distance == 0. {
            return Err(());
        }

        let (acceleration, rapid_rate) =
            ((&unit_vec).get_max_acc(), (&unit_vec).get_max_velocity());

        let programmed_rate = if pl_data
            .condition
            .contains(PlanCondition::PL_COND_FLAG_RAPID_MOTION)
        {
            rapid_rate
        } else {
            let mut rate = pl_data.feed_rate;
            if pl_data
                .condition
                .contains(PlanCondition::PL_COND_FLAG_INVERSE_TIME)
            {
                // If feed rate is inverted, the feed rate is taken to mean "frequency" and would
                //complete the operation in 1/feed_rate minutes.
                rate *= distance
            }
            rate
        };

        let mut block = PlanBlock {
            condition: pl_data.condition,
            spindle_speed: pl_data.spindle_speed,
            steps,
            // step_event_count: steps_abs.max_element() as usize,
            millimeters: distance,
            acceleration,
            rapid_rate,
            programmed_rate,

            entry_speed_sqr: 0.,
            max_entry_speed_sqr: 0.,
            max_junction_speed_sqr: 0.,
            nominal_speed: 0.,

            is_sys_motion,
        };

        if true == is_sys_motion {
            self.block_buffer.push_back(block);
            return Ok(());
        } else {
            // TODO: Need to check this method handling zero junction speeds when starting from rest.
            let max_junction_speed_sqr = if self.block_buffer.len() > 0 {
                calc_max_junction_speed_sqr(&self.prevar.pl_previous_unit_vec, &unit_vec)
            } else {
                0.
            };
            let nominal_speed = block.get_nominal_speed();
            let max_entry_speed_sqr = Self::compute_profile_max_entry_speed_sqr(
                max_junction_speed_sqr,
                nominal_speed,
                self.prevar.pl_previous_nominal_speed,
            );

            block.max_junction_speed_sqr = max_junction_speed_sqr;
            block.max_entry_speed_sqr = max_entry_speed_sqr;
            block.nominal_speed = nominal_speed;

            // Update previous path unit_vector and planner position.
            self.prevar.update(nominal_speed, &unit_vec, &target_steps);

            {
                //if previous is a sys motion
                if let Some(p) = self.block_buffer.back() {
                    if p.is_sys_motion == true {
                        block.max_entry_speed_sqr = 0.;
                    }
                }
                // New block is all set. Update buffer.
                self.block_buffer.push_back(block);
            }

            // Finish up by recalculating the plan with the new block.
            self.recalculate();
        }

        Ok(())
    }

    /// Computes the max entry speed (sqr) of the block, based on the minimum of the junction's
    /// previous and current nominal speeds and max junction speed.
    fn compute_profile_max_entry_speed_sqr(
        max_junction_speed_sqr: f32,
        nominal_speed: f32,
        prev_nominal_speed: f32,
    ) -> f32 {
        // Compute the junction maximum entry based on the minimum of the junction speed and neighboring nominal speeds.
        let max_entry_speed_sqr = if nominal_speed > prev_nominal_speed {
            prev_nominal_speed * prev_nominal_speed
        } else {
            nominal_speed * nominal_speed
        };

        max_entry_speed_sqr.min(max_junction_speed_sqr)
    }

    /// to calculate each block's entry_speed
    ///
    /// ```text
    ///                        PLANNER SPEED DEFINITION
    ///                                +--------+   <- current->nominal_speed
    ///                               /          \
    ///    current->entry_speed ->   +            \
    ///                              |             + <- next->entry_speed (aka exit speed)
    ///                              +-------------+
    ///                                  time -->
    /// ```
    fn recalculate(&mut self) {
        let buf_len = self.block_buffer.len();
        if buf_len == 0 {
            return; // empty
        }
        //init
        let planned = self.block_buffer_planned.get_or_insert(0).clone();

        let last_index = buf_len - 1;

        // Bail. Can't do anything with one only one plan-able block.
        if last_index == planned {
            return;
        }

        let get_end_speed_sqr = |v0_sqr: f32, accel: f32, s: f32| -> f32 {
            // 2as = v^2_1 -v^2_0
            2. * accel * s + v0_sqr
        };

        let rrange = ((planned + 1)..last_index).rev();
        //to calculate each block's entry_speed, use two steps:
        // first step: using reverse pass, the last block's exist speed(is zero) is pre-condition, reverse each block's
        // entry speed. on other word, look the full sequence as a deceleration process, calc each item's entry speed
        {
            // update last item's entry speed,its exist_speed must be zero
            if let Some(item) = self.block_buffer.get_mut(last_index) {
                let x = get_end_speed_sqr(0., item.acceleration, item.millimeters);
                item.entry_speed_sqr = item.max_entry_speed_sqr.min(x);
            }

            for index in rrange {
                let next_entry_speed_sqr = {
                    let n = &self.block_buffer.get(index + 1).unwrap();
                    n.entry_speed_sqr
                };
                let c = self.block_buffer.get_mut(index).unwrap();

                if c.entry_speed_sqr != c.max_entry_speed_sqr {
                    let x = get_end_speed_sqr(next_entry_speed_sqr, c.acceleration, c.millimeters);
                    c.entry_speed_sqr = c.max_entry_speed_sqr.min(x);
                }
            }
        }
        // second step: using forward pass, the planned block(the last optimal planned) is pre-condition
        let range = planned..last_index;
        {
            // Forward Pass: Forward plan the acceleration curve from the planned pointer onward.
            // Also scans for optimal plan breakpoints and appropriately updates the planned pointer.
            for index in range {
                let (cur_entry_speed_sqr, cur_acc, cur_mills) = {
                    let cur = self.block_buffer.get(index).unwrap();
                    (cur.entry_speed_sqr, cur.acceleration, cur.millimeters)
                };

                let next = self.block_buffer.get_mut(index + 1).unwrap();

                // Any acceleration detected in the forward pass automatically moves the optimal planned
                // pointer forward, since everything before this is all optimal. In other words, nothing
                // can improve the plan from the buffer tail to the planned pointer by logic.
                if cur_entry_speed_sqr < next.entry_speed_sqr {
                    let sqr = get_end_speed_sqr(cur_entry_speed_sqr, cur_acc, cur_mills);
                    // If true, current block is full-acceleration and we can move the planned pointer forward.
                    if sqr < next.entry_speed_sqr {
                        // Always <= max_entry_speed_sqr. Backward pass sets this.
                        next.entry_speed_sqr = sqr;
                        // Set optimal plan pointer.
                        self.block_buffer_planned = Some(index + 1);
                    }
                }
                if next.entry_speed_sqr == next.max_entry_speed_sqr {
                    self.block_buffer_planned = Some(index + 1);
                }
            }
        }
    }

    #[cfg(test)]
    pub fn dump_planers(&self) -> Vec<&PlanBlock> {
        self.block_buffer.iter().collect()
    }
}

/// Compute maximum allowable entry speed at junction by centripetal acceleration approximation.
fn calc_max_junction_speed_sqr(previous_unit_vec: &Vec3<f32>, unit_vec: &Vec3<f32>) -> f32 {
    let junction_cos_theta = -previous_unit_vec.dot(*unit_vec);
    // NOTE: Computed without any expensive trig, sin() or acos(), by trig half angle identity of cos(theta).
    if junction_cos_theta > 0.999999 {
        //  For a 0 degree acute junction, just set minimum junction speed.
        return MINIMUM_JUNCTION_SPEED * MINIMUM_JUNCTION_SPEED;
    } else if junction_cos_theta < -0.999999 {
        // Junction is a straight line or 180 degrees. Junction speed is infinite.
        return f32::MAX; // SOME_LARGE_VALUE;
    }

    // 法向量
    let junction_unit_vec = (*unit_vec - *previous_unit_vec).as_unit_vec();

    let junction_acceleration = (&junction_unit_vec).get_max_acc();

    let sin_theta_d2 = (0.5 * (1.0 - junction_cos_theta)).sqrt(); // Trig half angle identity. Always positive.

    let min_junction_speed_sqr = MINIMUM_JUNCTION_SPEED.powi(2);
    let vecolicity_sqr =
        (junction_acceleration * DEFAULT_JUNCTION_DEVIATION * sin_theta_d2) / (1.0 - sin_theta_d2);

    let max_junction_speed_sqr = min_junction_speed_sqr.max(vecolicity_sqr);

    max_junction_speed_sqr
}

#[cfg(test)]
mod tests {
    use super::{PlanCondition, PlanLineData, Planer, Vec3};

    #[test]
    fn plan_buffer_push() {
        let mut planer = Planer::new();

        let pl_data = PlanLineData {
            feed_rate: 0.,
            spindle_speed: 0.,
            condition: PlanCondition::default(),
        };
        let _exist_speed_sqr: f32 = 0.;
        let sys_position: Vec3<i32> = Vec3::new(5, 5, 5);
        let target: Vec3<f32> = Vec3::new(10., 10., 10.);

        let _ = planer.push_sys_motion(&target, &pl_data, &sys_position);
        let _ = planer.push_normal_motion(&target, &pl_data);

        let collects = planer.dump_planers();
        let (sys_m, normal_m) = (&collects[0], &collects[1]);
        //check sys motion
        assert_eq!(sys_m.steps, Vec3::<i32>::new(95, 95, 95));
        assert_eq!(normal_m.steps, Vec3::<i32>::new(100, 100, 100));
        assert_eq!(normal_m.entry_speed_sqr, 0.);
    }

    /// check
    /// 1. push sys motion that exist speed-sqr must be zero
    /// 2. push normal motion
    #[test]
    fn push2() {
        let mut planer = Planer::new();

        let pl_data = PlanLineData {
            feed_rate: 0.,
            spindle_speed: 0.,
            condition: PlanCondition::default(),
        };

        let target: Vec3<f32> = Vec3::new(10., 10., 10.);
        let _ = planer.push_normal_motion(&target, &pl_data);
        assert_eq!(planer.len(), 1);
        //zero mills will not accept
        let _ = planer.push_normal_motion(&target, &pl_data);
        assert_eq!(planer.len(), 1);

        let previsous_steps: Vec3<i32> = Vec3::new(0, 0, 0);
        let target: Vec3<f32> = Vec3::new(10., 10., 10.);
        let _ = planer.push_sys_motion(&target, &pl_data, &previsous_steps);
        assert_eq!(planer.len(), 2);

        let target: Vec3<f32> = Vec3::new(11., 10., 10.);
        let _ = planer.push_normal_motion(&target, &pl_data);
        assert_eq!(planer.len(), 3);
        let target: Vec3<f32> = Vec3::new(11., 11., 10.);
        let _ = planer.push_normal_motion(&target, &pl_data);
        assert_eq!(planer.len(), 4);

        let collects = planer.dump_planers();
        assert_eq!(collects[0].entry_speed_sqr, 0.);
        assert_eq!(collects[1].entry_speed_sqr, 0.);
        assert_eq!(collects[2].entry_speed_sqr, 0.);
        assert_eq!(collects[3].entry_speed_sqr, 1.0);
    }
}
