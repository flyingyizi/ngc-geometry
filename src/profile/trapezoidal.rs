///! The Trapezoidal acceleration (ramping),  it composing the geometric path with the motion law.
///
///! Trapezoidal motion profile
// use core::ops;

// use core::cmp::max;
#[allow(unused_imports)]
use num_traits::{clamp_min, Float, Inv /*clamp_max,One, Zero*/};

/// Represents the Start and End Positions of the S_Curve
#[derive(Clone, Debug)]
pub struct Conditions {
    ///start velocity: unit is steps per unit-time
    pub enter_velocity: f32,
    ///end velocity: unit is steps per unit-time
    pub end_velocity: f32,
    ///: unit is steps per (unit-time)^2
    pub target_accel: f32,
}

impl Default for Conditions {
    fn default() -> Self {
        Conditions {
            enter_velocity: 0.0,
            end_velocity: 0.0,
            target_accel: 1.,
        }
    }
}

/// Trapezoidal motion profile
///
/// Generates an approximation of a trapezoidal ramp, following the algorithm
/// laid out here:
/// [http://hwml.com/LeibRamp.htm](http://hwml.com/LeibRamp.htm)
///
/// A PDF version of that page is available:
/// [http://hwml.com/LeibRamp.pdf](http://hwml.com/LeibRamp.pdf)
///
/// # Acceleration Ramp
///
/// This struct will generate a trapezoidal acceleration ramp with the following
/// attributes:
/// - The velocity will always be equal to or less than the maximum velocity
///   passed to the constructor.
/// - While ramping up or down, the acceleration will be an approximation
///   of the target acceleration passed to the constructor.
///
/// # Unit of Time
///
/// This code is agnostic on which unit of time is used. If you provide the
/// target acceleration and maximum velocity in steps per second, the unit of
/// the delay returned will be seconds.
///
/// This allows you to pass the parameters in steps per number of timer counts
/// for the timer you're using, completely eliminating any conversion overhead
/// for the delay.
///
#[allow(dead_code)]
pub struct Trapezoidal {
    /**
     * Trapezoidal acceleration：
     *    │
     *    xxxxxxxxxx
     *    │
     * ───┼─────────xxxxxxxxxx──────────────
     *    │
     *    │                   xxxxxxxxxxx
     *    │
     */
    /// it is \cfrac{1}{max-velocity}
    delay_min: f32,
    delay_initial: f32,
    delay_prev: f32,

    target_accel: f32,
    steps_left: u32,

    end_velocity: f32,
}

impl Trapezoidal {
    /// Create a new instance of `Trapezoidal`. the implement do FORWARD-only.
    /// all of paramenters MUST BE POSITIVE.
    ///
    /// acceleration in steps per (unit-time)^2. It must not be zero.
    ///
    /// velocity in steps per unit-time.
    ///
    /// The number of steps given here is always relative to the current
    /// position, as implementations of this algorithm are not expected to keep
    /// track of an absolute position.
    ///
    /// # Panics
    ///
    /// Panics, if `target_accel` is zero.
    #[allow(dead_code)]
    pub fn new(cond: Option<Conditions>, max_velocity: Option<f32>, num_steps: u32) -> Self {
        let conditions = if let Some(v) = cond {
            v
        } else {
            Conditions::default()
        };

        let max_velocity = Trapezoidal::get_max_velocity(&conditions, max_velocity, num_steps);
        let sv = &conditions.enter_velocity;
        let ev = &conditions.end_velocity;
        let a = &conditions.target_accel;

        let delay_min = max_velocity.inv();

        //[2]: calculate time delay initial value
        // intital: p_1 =  \cfrac{1}{\sqrt{v^2_0 +2a}}
        let initial_delay = 1.0 / (sv * sv + 2.0 * a).sqrt();

        Self {
            delay_min,
            delay_initial: initial_delay,
            delay_prev: initial_delay,

            target_accel: *a,
            steps_left: num_steps,

            end_velocity: *ev,
        }
    }

    // if input max_velocity is invalid, re arrange it
    fn get_max_velocity(cond: &Conditions, max_velocity: Option<f32>, num_steps: u32) -> f32 {
        let sv = &cond.enter_velocity;
        let ev = &cond.end_velocity;
        let a = &cond.target_accel;

        let t_max = sv.max(*ev);
        if max_velocity.is_none() {
            return t_max;
        }

        let max_velocity = max_velocity.unwrap();
        // input max velocity is less enter/end velocity
        // use the less one as the max velocity
        if max_velocity < t_max {
            return t_max;
        }

        //check whether max_velocity is too large

        // S = \cfrac{v^2 -v^2_0}{2*a},
        // compare num_steps and sum:distance between[v_0, v_max] plus distance between[v_0, v_max]
        let temp = 0.5 * (sv * sv + ev * ev) / a;
        let dis = max_velocity * max_velocity / a - temp;
        let dis = dis.ceil() as u32;

        // input max velocity is too large to complete steps with required end_velocity in limited distance
        let max = if dis > num_steps {
            // minimum requirements
            let t = ((num_steps as f32 + temp) * a).sqrt();
            t
        } else {
            max_velocity
        };

        max
    }

    pub fn next_delay(&mut self) -> Option<f32> {
        let mode = RampMode::compute(self);

        // Compute the delay for the next step. [22]
        // p_i=p_{i-1} (1+m*p^2_{i-1} + \cfrac{3}{2}(m*p^2_{i-1})^2)
        let q = self.target_accel * self.delay_prev * self.delay_prev;
        let addend = 1.5_f32 * q * q;
        let delay_next = match mode {
            RampMode::Idle => {
                return None;
            }
            RampMode::RampUp => {
                let delay_next = self.delay_prev * (1.0_f32 - q + addend);
                //confirm all veclocity is lee than the max veclocity
                clamp_min(delay_next, self.delay_min)
            }
            RampMode::Plateau => self.delay_prev,
            RampMode::RampDown => {
                let delay_next = self.delay_prev * (1.0_f32 + q + addend);
                delay_next
            }
        };

        self.delay_prev = delay_next;
        self.steps_left = self.steps_left.saturating_sub(1);

        Some(delay_next)
    }

    #[allow(dead_code)]
    pub fn next_velocity(&mut self) -> Option<f32> {
        self.next_delay().map(|delay| delay.inv())
    }
}

enum RampMode {
    Idle,
    RampUp,
    Plateau,
    RampDown,
}

impl RampMode {
    fn compute(profile: &Trapezoidal) -> Self {
        if profile.steps_left == 0
        /*no_steps_left*/
        {
            return Self::Idle;
        }

        // S = \cfrac{v^2 -v^2_0}{2*a}
        // Compute the number of steps needed to come to a stop. We'll compare
        // that to the number of steps left to the target step below, to
        // determine whether we need to decelerate.
        let (sv, ev) = (profile.delay_prev.inv(), profile.end_velocity);
        let steps_to_stop = (sv * sv - ev * ev) / (2.0 * profile.target_accel);
        let steps_to_stop = steps_to_stop.ceil().abs() as u32;

        if profile.steps_left <= steps_to_stop {
            return Self::RampDown;
        }

        let above_max_velocity = profile.delay_prev < profile.delay_min;
        let reached_max_velocity = profile.delay_prev == profile.delay_min;

        if above_max_velocity {
            Self::RampDown
        } else if reached_max_velocity {
            Self::Plateau
        } else {
            Self::RampUp
        }
    }
}
