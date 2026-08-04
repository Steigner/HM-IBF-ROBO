//! Robotics trajectory optimization experiments using the GRAHF framework.
//!
//! The crate provides the problem definition, island builders, migration builders and the
//! GRAHF genetic algorithm wired together to optimize 6-DOF waypoint trajectories, plus the
//! training and evaluation stages driven by the `hm-ibf-robo` binary.

pub mod cli;
pub mod config;
pub mod evaluation;
pub mod heuristic;
pub mod islands;
pub mod migrations;
pub mod problems;
pub mod robo;
pub mod training;
