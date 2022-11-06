//! [形象解释PID算法](https://www.cnblogs.com/shangdawei/p/4825259.html)

#[allow(unused_imports)]
use num_traits::Float;

///
/// Structure that holds PID all the PID controller data, multiple instances are
/// posible using different structures for each controller
///
pub struct PID {
    /// Controller Setpoint
    setpoint: f32,
    // Tuning parameters
    /// Stores the gain for the Proportional term
    kp: f32,
    /// Stores the gain for the Integral term
    ki: f32,
    /// Stores the gain for the Derivative term
    kd: f32,

    e_prev: f32,
    e_last: f32,

    lasto: Option<f32>,
}

impl PID {
    pub fn new(set: f32, kp: f32, ki: f32, kd: f32) -> Self {
        let s = Self {
            setpoint: set,

            lasto: None,

            kp,
            ki,
            kd,

            e_last: 0.,
            e_prev: 0.,
        };

        // return pid;
        s
    }

    pub fn set_point(&mut self, set: f32) {
        self.setpoint = set;
    }
    /// Current Process Value
    pub fn compute(&mut self, input: f32) -> f32 /*output*/ {
        let delta = self.pidctrl_increa(input);

        //todo err_max, err_min
        let delta = delta.max(-200.).min(200.);

        let o = self.lasto.unwrap_or(input) + delta;
        self.lasto = Some(o);

        o
    }

    /// # 增量计算公式：
    /// # delta=Kp*[e(t) - e(t-1)] + Ki*e(t) + Kd*[e(t) - 2*e(t-1) +e(t-2)]
    fn pidctrl_increa(
        &mut self,
        real_output: f32, /*采样时刻被控对象实际输出*/
    ) -> f32 {
        let err = self.setpoint - real_output;

        let delta = self.kp * (err - self.e_last)
            + self.ki * err
            + self.kd * (err - 2. * self.e_last + self.e_prev);

        self.e_prev = self.e_last;
        self.e_last = err;
        return delta;
    }

    ///
    /// @brief Sets new PID tuning parameters
    ///
    /// Sets the gain for the Proportional (Kp), Integral (Ki) and Derivative (Kd)
    /// terms.
    ///
    pub fn tune(&mut self, kp: f32, ki: f32, _kd: f32) {
        // Check for validity
        if kp < 0. || ki < 0. || _kd < 0. {
            return;
        }

        self.kp = kp;
        self.ki = ki;
        self.kd = _kd;
    }
}
