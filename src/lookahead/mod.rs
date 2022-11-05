mod algorithm;
mod const_cfg;

pub use algorithm::{PlanBlock, Planer};
/// rotate motor forward or backward
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Direction {
    /// Rotate the motor forward
    ///
    /// This corresponds to whatever direction the motor rotates in when the
    /// driver's DIR signal is set HIGH.
    Forward = 1,

    /// Rotate the motor backward
    ///
    /// This corresponds to whatever direction the motor rotates in when the
    /// driver's DIR signal set is LOW.
    Backward = -1,
}

use bitflags::bitflags;

bitflags! {
    #[repr(C)]
    #[derive(Default)]
    /// Define planner data condition flags. Used to denote running conditions of a block.
    pub struct PlanCondition:u32 {
        const PL_COND_FLAG_RAPID_MOTION = 1 << 0;

        /// Motion does not honor feed override.
        const PL_COND_FLAG_NO_FEED_OVERRIDE = 1 << 2;
        /// Interprets feed rate value as inverse time when set.
        const PL_COND_FLAG_INVERSE_TIME = 1 << 3;
        const PL_COND_FLAG_SPINDLE_CW = 1 << 4;
        const PL_COND_FLAG_SPINDLE_CCW = 1 << 5;
        const PL_COND_FLAG_COOLANT_FLOOD = 1 << 6;
        const PL_COND_FLAG_COOLANT_MIST = 1 << 7;

    }
}

#[derive(Copy, Clone, Debug, Default)]
pub struct PlanLineData {
    /// Desired feed rate for line motion. Value is ignored, if rapid motion.
    pub feed_rate: f32,
    /// Desired spindle speed through line motion.
    pub spindle_speed: f32,
    /// condition: BitFlags<PlanCondition>, // Bitflag variable to indicate planner conditions. See [PlanCondition] defines above.
    pub condition: PlanCondition,
}
