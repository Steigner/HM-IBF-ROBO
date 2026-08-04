//! Dimension-agnostic robotics trajectory optimization problem.
//!
//! Solutions are vectors of joint-angle waypoints for a 6-DOF manipulator. The solution
//! length determines how many waypoints are used, so fitness is comparable across all
//! allowed island dimensions.

use std::{f64::consts::PI, fs, ops::Range, path::Path};

use eyre::{bail, WrapErr};
use mahf::{
    problems::{KnownOptimumProblem, LimitedVectorProblem, VectorProblem},
    Problem, SingleObjective,
};
use serde::{Deserialize, Serialize};

use crate::problems::DimensionAwareDomain;

/// Number of joint angles per waypoint.
pub const JOINTS: usize = 6;

/// Number of interpolated poses evaluated per waypoint segment.
pub const TARGET_POINTS_PER_SEGMENT: usize = 100;

/// Weight of the target-reaching term relative to the path-length term.
pub const GAMMA: f64 = 100.0;

/// File name inside the instances directory that holds the aggregated summary, not an instance.
const SUMMARY_FILE: &str = "summary.json";

/// JSON-serializable instance descriptor generated from `Pos_pnts.mat`.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RoboInstance {
    /// Instance name, e.g. `3_pnts_inst01`.
    pub name: String,
    /// Number of Cartesian target points the end effector must reach.
    pub nr_points: usize,
    /// Seed of the MATLAB RNG that selected the points.
    pub source_seed: u64,
    /// Row indices of the selected points inside `Pos_pnts.mat`.
    pub source_indices: Vec<usize>,
    /// The Cartesian target points.
    pub points: Vec<[f64; 3]>,
}

/// Robotics trajectory optimization problem.
///
/// A solution is a flattened `Vec<f64>` holding `JOINTS * nr_changes` joint angles. The
/// number of waypoints `nr_changes` is chosen per island by IRACE, so the evaluator uses
/// `solution.len()` rather than [`VectorProblem::dimension`]; all island dimensions
/// therefore produce directly comparable fitness values.
#[derive(Clone, Debug)]
pub struct RoboProblem {
    /// The instance being solved.
    pub instance: RoboInstance,
    /// The manipulator's home pose, used as the start of every trajectory.
    pub initial_angles: [f64; JOINTS],
    /// Allowed island dimensions in decision variables, strictly increasing.
    ///
    /// See `crate::config::TrainingParams::dimensions_allowed`.
    dimensions_allowed: Vec<u32>,
}

impl RoboProblem {
    /// Creates a problem for the given instance, starting from the zero pose.
    ///
    /// # Arguments
    ///
    /// * `instance` - The instance descriptor.
    /// * `dimensions_allowed` - Allowed island dimensions in decision variables, strictly
    ///   increasing; see `crate::config::TrainingParams::dimensions_allowed`.
    ///
    /// # Returns
    ///
    /// The problem.
    pub fn new(instance: RoboInstance, dimensions_allowed: Vec<u32>) -> Self {
        Self {
            instance,
            initial_angles: [0.0; JOINTS],
            dimensions_allowed,
        }
    }

    /// Loads every instance in `dir` as a problem, sorted by file name.
    ///
    /// # Arguments
    ///
    /// * `dir` - Directory holding the instance JSON files.
    /// * `dimensions_allowed` - Allowed island dimensions in decision variables, forwarded
    ///   to every loaded problem; see `crate::config::TrainingParams::dimensions_allowed`.
    ///
    /// # Returns
    ///
    /// One problem per instance file.
    ///
    /// # Errors
    ///
    /// Returns an error if the directory cannot be read, contains no instances, or holds a
    /// file that is not a valid instance descriptor.
    pub fn load_instances(
        dir: impl AsRef<Path>,
        dimensions_allowed: &[u32],
    ) -> eyre::Result<Vec<Self>> {
        Ok(Self::load_instance_descriptors(dir)?
            .into_iter()
            .map(|instance| Self::new(instance, dimensions_allowed.to_vec()))
            .collect())
    }

    /// Loads every instance descriptor in `dir`, sorted by file name.
    ///
    /// The aggregated `summary.json` is skipped.
    ///
    /// # Arguments
    ///
    /// * `dir` - Directory holding the instance JSON files.
    ///
    /// # Returns
    ///
    /// The instance descriptors.
    ///
    /// # Errors
    ///
    /// Returns an error if the directory cannot be read, contains no instances, or holds a
    /// file that is not a valid instance descriptor.
    pub fn load_instance_descriptors(dir: impl AsRef<Path>) -> eyre::Result<Vec<RoboInstance>> {
        let dir = dir.as_ref();
        let mut paths: Vec<_> = fs::read_dir(dir)
            .wrap_err_with(|| format!("failed to read instances from {}", dir.display()))?
            .filter_map(|entry| entry.ok().map(|entry| entry.path()))
            .filter(|path| {
                path.extension().and_then(|ext| ext.to_str()) == Some("json")
                    && path.file_name().and_then(|name| name.to_str()) != Some(SUMMARY_FILE)
            })
            .collect();
        paths.sort();

        if paths.is_empty() {
            bail!(
                "no robotics instances found in {}. Run hm-ibf-robo/preprocessing/prepare_instances.py first",
                dir.display()
            );
        }

        paths
            .into_iter()
            .map(|path| {
                let content = fs::read_to_string(&path)
                    .wrap_err_with(|| format!("failed to read {}", path.display()))?;
                serde_json::from_str::<RoboInstance>(&content)
                    .wrap_err_with(|| format!("failed to parse {}", path.display()))
            })
            .collect()
    }

    /// Returns the number of waypoints encoded by a flattened solution.
    ///
    /// # Arguments
    ///
    /// * `solution` - The flattened joint angles.
    ///
    /// # Returns
    ///
    /// `solution.len() / JOINTS`, or `None` if the length is zero or not a whole number of
    /// waypoints.
    pub fn solution_nr_changes(solution: &[f64]) -> Option<usize> {
        (!solution.is_empty() && solution.len().is_multiple_of(JOINTS))
            .then_some(solution.len() / JOINTS)
    }

    /// Returns the Cartesian end-effector position for a single pose.
    ///
    /// # Arguments
    ///
    /// * `angles` - The joint angles of the pose.
    ///
    /// # Returns
    ///
    /// The `[x, y, z]` position of the tool centre point.
    pub fn end_effector_position(angles: &[f64; JOINTS]) -> [f64; 3] {
        forward_kinematics(angles)
    }

    /// Evaluates a flattened trajectory.
    ///
    /// The objective is `GAMMA * max_target_miss + path_length`, where `max_target_miss` is
    /// the largest distance from any target to its closest point on the trajectory. Lower is
    /// better; the known optimum is `0`.
    ///
    /// # Arguments
    ///
    /// * `solution` - The flattened joint angles.
    ///
    /// # Returns
    ///
    /// The objective value, or [`f64::INFINITY`] if the solution is not a valid waypoint
    /// encoding.
    pub fn evaluate_solution(&self, solution: &[f64]) -> f64 {
        let Some(nr_changes) = Self::solution_nr_changes(solution) else {
            return f64::INFINITY;
        };

        let mut angles = self.initial_angles;
        let mut positions = Vec::with_capacity(nr_changes * TARGET_POINTS_PER_SEGMENT);

        for change in 0..nr_changes {
            let start = change * JOINTS;
            let mut destination = [0.0; JOINTS];
            destination.copy_from_slice(&solution[start..start + JOINTS]);

            for step in 0..TARGET_POINTS_PER_SEGMENT {
                let t = if TARGET_POINTS_PER_SEGMENT <= 1 {
                    1.0
                } else {
                    step as f64 / (TARGET_POINTS_PER_SEGMENT - 1) as f64
                };

                let mut interpolated = [0.0; JOINTS];
                for joint in 0..JOINTS {
                    interpolated[joint] = angles[joint] + t * (destination[joint] - angles[joint]);
                }
                positions.push(forward_kinematics(&interpolated));
            }

            angles = destination;
        }

        if positions.is_empty() {
            return f64::INFINITY;
        }

        let path_len = positions
            .windows(2)
            .map(|pair| euclidean_distance(pair[0], pair[1]))
            .sum::<f64>();

        let max_distance = self
            .instance
            .points
            .iter()
            .map(|target| {
                positions
                    .iter()
                    .map(|&position| euclidean_distance(position, *target))
                    .fold(f64::INFINITY, f64::min)
            })
            .fold(0.0, f64::max);

        GAMMA * max_distance + path_len
    }
}

impl Problem for RoboProblem {
    type Encoding = Vec<f64>;
    type Objective = SingleObjective;

    fn name(&self) -> &str {
        &self.instance.name
    }
}

impl VectorProblem for RoboProblem {
    type Element = f64;

    /// Returns the largest allowed island dimension.
    ///
    /// Islands running at a smaller dimension read their own size from
    /// [`crate::islands::IslandDimension`] instead.
    ///
    /// # Panics
    ///
    /// Panics if `dimensions_allowed` is empty; `TrainingParams::load` rejects that before
    /// any `RoboProblem` is constructed.
    fn dimension(&self) -> usize {
        *self
            .dimensions_allowed
            .last()
            .expect("at least one island dimension must be allowed") as usize
    }
}

impl LimitedVectorProblem for RoboProblem {
    fn domain(&self) -> Vec<Range<Self::Element>> {
        vec![(-2.0 * PI)..(2.0 * PI); self.dimension()]
    }
}

impl KnownOptimumProblem for RoboProblem {
    fn known_optimum(&self) -> SingleObjective {
        SingleObjective::try_from(0.0).expect("zero is a valid objective value")
    }
}

impl DimensionAwareDomain for RoboProblem {}

/// Returns the Euclidean distance between two Cartesian points.
fn euclidean_distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

/// Returns the tool centre point of a UR3-style 6-DOF arm for the given joint angles.
///
/// The Denavit-Hartenberg parameters match `robo-evo-apps`.
fn forward_kinematics(angles: &[f64; JOINTS]) -> [f64; 3] {
    let transforms = [
        dh_transform(angles[0], PI / 2.0, 0.1519, 0.0),
        dh_transform(angles[1], 0.0, 0.0, -0.24365),
        dh_transform(angles[2], 0.0, 0.0, -0.21325),
        dh_transform(angles[3], PI / 2.0, 0.11235, 0.0),
        dh_transform(angles[4], -PI / 2.0, 0.08535, 0.0),
        dh_transform(angles[5], 0.0, 0.0819, 0.0),
    ];

    let mut acc = identity_matrix();
    for transform in transforms {
        acc = matrix_mul(acc, transform);
    }

    [acc[0][3], acc[1][3], acc[2][3]]
}

/// Returns the homogeneous Denavit-Hartenberg transform for one joint.
fn dh_transform(theta: f64, alpha: f64, d: f64, a: f64) -> [[f64; 4]; 4] {
    [
        [
            theta.cos(),
            -theta.sin() * alpha.cos(),
            theta.sin() * alpha.sin(),
            a * theta.cos(),
        ],
        [
            theta.sin(),
            theta.cos() * alpha.cos(),
            -theta.cos() * alpha.sin(),
            a * theta.sin(),
        ],
        [0.0, alpha.sin(), alpha.cos(), d],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

/// Returns the 4x4 identity matrix.
fn identity_matrix() -> [[f64; 4]; 4] {
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

/// Multiplies two 4x4 matrices.
fn matrix_mul(a: [[f64; 4]; 4], b: [[f64; 4]; 4]) -> [[f64; 4]; 4] {
    let mut out = [[0.0; 4]; 4];
    for (row, out_row) in out.iter_mut().enumerate() {
        for (col, out_value) in out_row.iter_mut().enumerate() {
            *out_value = (0..4).map(|k| a[row][k] * b[k][col]).sum();
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-9;

    /// Allowed island dimensions used by tests in this module.
    ///
    /// The real invariants are enforced at load time by
    /// `config::validate_dimensions_allowed`; these tests only need *some* valid list.
    const TEST_DIMENSIONS: [u32; 5] = [6, 12, 18, 24, 30];

    /// Builds a small instance whose single target sits at the home position.
    fn problem_with_target(target: [f64; 3]) -> RoboProblem {
        RoboProblem::new(
            RoboInstance {
                name: "1_pnts_test".to_string(),
                nr_points: 1,
                source_seed: 1,
                source_indices: vec![0],
                points: vec![target],
            },
            TEST_DIMENSIONS.to_vec(),
        )
    }

    #[test]
    fn solution_nr_changes_accepts_whole_waypoints_only() {
        assert_eq!(RoboProblem::solution_nr_changes(&[0.0; 18]), Some(3));
        assert_eq!(RoboProblem::solution_nr_changes(&[0.0; 6]), Some(1));
        assert_eq!(RoboProblem::solution_nr_changes(&[0.0; 7]), None);
        assert_eq!(RoboProblem::solution_nr_changes(&[]), None);
    }

    #[test]
    fn every_allowed_dimension_is_a_valid_encoding() {
        for dimension in TEST_DIMENSIONS {
            let solution = vec![0.0; dimension as usize];
            assert!(
                RoboProblem::solution_nr_changes(&solution).is_some(),
                "{dimension}"
            );
        }
    }

    #[test]
    fn the_home_pose_maps_to_a_reachable_point() {
        let position = RoboProblem::end_effector_position(&[0.0; JOINTS]);

        assert!(position.iter().all(|v| v.is_finite()), "{position:?}");
        // The UR3 link lengths bound the reachable workspace to well under a metre.
        let reach = (position[0].powi(2) + position[1].powi(2) + position[2].powi(2)).sqrt();
        assert!(reach < 1.0, "reach {reach} exceeds the workspace");
    }

    #[test]
    fn forward_kinematics_is_periodic_in_every_joint() {
        let base = RoboProblem::end_effector_position(&[0.3, -0.2, 0.5, 1.0, -1.0, 0.7]);

        for joint in 0..JOINTS {
            let mut shifted = [0.3, -0.2, 0.5, 1.0, -1.0, 0.7];
            shifted[joint] += 2.0 * PI;
            let position = RoboProblem::end_effector_position(&shifted);

            for axis in 0..3 {
                assert!(
                    (position[axis] - base[axis]).abs() < 1e-9,
                    "joint {joint} axis {axis}"
                );
            }
        }
    }

    #[test]
    fn an_invalid_encoding_evaluates_to_infinity() {
        let problem = problem_with_target([0.0, 0.0, 0.0]);

        assert_eq!(problem.evaluate_solution(&[]), f64::INFINITY);
        assert_eq!(problem.evaluate_solution(&[0.0; 5]), f64::INFINITY);
    }

    #[test]
    fn staying_at_the_home_pose_costs_only_the_target_miss() {
        let home = RoboProblem::end_effector_position(&[0.0; JOINTS]);
        let problem = problem_with_target(home);

        // A trajectory that never moves has zero path length and hits the target exactly.
        let value = problem.evaluate_solution(&[0.0; JOINTS]);

        assert!(value.abs() < EPS, "expected 0, got {value}");
    }

    #[test]
    fn missing_a_target_is_penalised_by_gamma() {
        let home = RoboProblem::end_effector_position(&[0.0; JOINTS]);
        let offset = [home[0], home[1], home[2] + 0.5];
        let problem = problem_with_target(offset);

        let value = problem.evaluate_solution(&[0.0; JOINTS]);

        assert!((value - GAMMA * 0.5).abs() < 1e-6, "got {value}");
    }

    #[test]
    fn the_objective_is_finite_for_every_allowed_dimension() {
        let problem = problem_with_target([0.1, 0.1, 0.2]);

        for dimension in TEST_DIMENSIONS {
            let solution: Vec<f64> = (0..dimension as usize)
                .map(|i| (i as f64 * 0.31).sin())
                .collect();
            let value = problem.evaluate_solution(&solution);

            assert!(value.is_finite(), "dimension {dimension} gave {value}");
            assert!(value >= 0.0, "the objective must be non-negative");
        }
    }

    #[test]
    fn the_domain_covers_the_maximum_island_dimension() {
        let problem = problem_with_target([0.0, 0.0, 0.0]);
        let domain = problem.domain();

        assert_eq!(domain.len(), *TEST_DIMENSIONS.last().unwrap() as usize);
        for range in domain {
            assert!((range.start + 2.0 * PI).abs() < EPS);
            assert!((range.end - 2.0 * PI).abs() < EPS);
        }
    }

    #[test]
    fn domain_for_dimension_matches_the_requested_length() {
        let problem = problem_with_target([0.0, 0.0, 0.0]);

        for dimension in TEST_DIMENSIONS {
            assert_eq!(
                problem.domain_for_dimension(dimension as usize).len(),
                dimension as usize
            );
        }
    }

    #[test]
    fn the_known_optimum_is_zero() {
        assert_eq!(problem_with_target([0.0; 3]).known_optimum().value(), 0.0);
    }

    #[test]
    fn loading_from_a_missing_directory_reports_the_path() {
        let error =
            RoboProblem::load_instances("definitely/not/here", &TEST_DIMENSIONS).unwrap_err();
        assert!(error.to_string().contains("definitely/not/here"), "{error}");
    }
}
