//! Robotics trajectory benchmark module for GRAHF.
//!
//! Implements 6-DOF waypoint-trajectory optimization using:
//! - JSON instances generated from `Pos_pnts.mat`
//! - adaptive `nr_changes = 1..5` island waypoint encodings
//! - forward-kinematics path evaluation
//! - heterogeneous migration across `nr_changes = 1..5`

pub mod evaluator;
pub mod output;
pub mod problem;

pub use evaluator::RoboEvaluator;
pub use problem::{RoboInstance, RoboProblem, GAMMA, JOINTS, TARGET_POINTS_PER_SEGMENT};
