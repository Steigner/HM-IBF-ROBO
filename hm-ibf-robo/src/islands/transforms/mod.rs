//! Heterogeneous migration transforms for the robotics benchmark.
//!
//! This implementation mirrors the selected methods from `robo-evo-apps/transforms.py`.
//! The contract from the Python reference is:
//!
//! 1. if the signal already has the target length, apply the method directly,
//! 2. otherwise resample it to a target-length proxy first,
//! 3. apply the method to that proxy using the migration complexity budget.
//!
//! A solution has shape `(nr_changes, JOINTS)` flattened row-major. Each joint is
//! transformed independently as a 1D signal.
//!
//! Submodules:
//!
//! * [`angles`] - phase unwrapping and joint-limit bounding,
//! * [`backbone`] - arc-length parameterisation along the instance's target route,
//! * [`interpolation`] - sampling grids and spline slope estimation,
//! * [`linalg`] - dense and tridiagonal solvers,
//! * [`methods`] - the transformation methods themselves.

use grahf::components::transform::{SolutionTransformer, TransformRequest};
use mahf::Random;

use crate::robo::{RoboProblem, JOINTS};

mod angles;
mod backbone;
mod interpolation;
mod kernels;
mod linalg;
mod methods;

pub use methods::TransformMethod;

use angles::{bound_unwrapped_angle_signal, unwrap_angles};
use backbone::{source_backbone_positions, TargetRouteBackbone};
use interpolation::resample_from_points;
use methods::{apply_method_same_len, transform_signal_to_length};

/// The method used when IRACE supplies an unknown transform name.
const FALLBACK_METHOD: TransformMethod = TransformMethod::Pchip;

/// Resizes migrants along a deterministic Cartesian route through the instance's targets.
#[derive(Clone, Copy, Debug, Default)]
pub struct TargetRouteTransformer;

impl TargetRouteTransformer {
    /// Creates a new `TargetRouteTransformer`.
    ///
    /// # Returns
    ///
    /// The transformer.
    pub fn new() -> Self {
        Self
    }
}

impl SolutionTransformer<RoboProblem> for TargetRouteTransformer {
    fn transform(
        &self,
        problem: &RoboProblem,
        solution: &Vec<f64>,
        request: TransformRequest<'_>,
        _rng: &mut Random,
    ) -> Vec<f64> {
        if request.is_identity() {
            return solution.clone();
        }

        transform_along_target_route(
            problem,
            solution,
            request.target_dim as usize,
            request.method,
        )
    }
}

/// Resizes `solution` to `target_dim` using the instance's target route as the parameter.
///
/// Falls back to [`transform_uniformly`] when the route is degenerate or either dimension is
/// not a whole number of waypoints.
///
/// # Arguments
///
/// * `problem` - The instance supplying the target route and the initial pose.
/// * `solution` - The flattened joint angles to resize.
/// * `target_dim` - The requested output length.
/// * `method_name` - The transformation method name; unknown names fall back to PCHIP.
///
/// # Returns
///
/// The resized solution of length `target_dim`, or an empty vector for degenerate input.
pub fn transform_along_target_route(
    problem: &RoboProblem,
    solution: &[f64],
    target_dim: usize,
    method_name: &str,
) -> Vec<f64> {
    let source_dim = solution.len();
    if source_dim == 0 || target_dim == 0 {
        return Vec::new();
    }

    if !source_dim.is_multiple_of(JOINTS) || !target_dim.is_multiple_of(JOINTS) {
        return transform_uniformly(solution, target_dim, method_name);
    }

    let source_changes = source_dim / JOINTS;
    let target_changes = target_dim / JOINTS;
    let Some(backbone) = TargetRouteBackbone::from_problem(problem) else {
        return transform_uniformly(solution, target_dim, method_name);
    };

    let source_t = source_backbone_positions(solution, source_changes, &backbone);
    let target_t: Vec<_> = (1..=target_changes)
        .map(|index| index as f64 / target_changes as f64)
        .collect();
    let method = parse_method(method_name);

    let mut out = vec![0.0; target_dim];
    for joint in 0..JOINTS {
        // The initial pose anchors the signal at t = 0 so the first segment is well defined.
        let mut samples = Vec::with_capacity(source_changes + 1);
        samples.push((0.0, problem.initial_angles[joint]));
        for change in 0..source_changes {
            samples.push((source_t[change], solution[change * JOINTS + joint]));
        }

        let (x, signal) = sorted_unwrapped_signal(samples);
        let proxy = resample_from_points(&x, &signal, &target_t);
        let transformed =
            apply_method_same_len(&proxy, (source_changes + 1).min(target_changes), method);

        for (change, value) in bound_unwrapped_angle_signal(&transformed)
            .into_iter()
            .enumerate()
        {
            out[change * JOINTS + joint] = value;
        }
    }

    out
}

/// Resizes `solution` to `target_dim` on a uniform waypoint grid, ignoring problem geometry.
///
/// # Arguments
///
/// * `solution` - The flattened joint angles to resize.
/// * `target_dim` - The requested output length.
/// * `method_name` - The transformation method name; unknown names fall back to PCHIP.
///
/// # Returns
///
/// The resized solution of length `target_dim`, or an empty vector for degenerate input.
pub fn transform_uniformly(solution: &[f64], target_dim: usize, method_name: &str) -> Vec<f64> {
    let source_dim = solution.len();
    if source_dim == 0 || target_dim == 0 {
        return Vec::new();
    }

    // A length that is not a whole number of waypoints cannot be split per joint, so the
    // flattened vector is resampled as a single signal instead.
    if !source_dim.is_multiple_of(JOINTS) || !target_dim.is_multiple_of(JOINTS) {
        let unwrapped = unwrap_angles(solution);
        let transformed = if source_dim == target_dim {
            unwrapped
        } else {
            interpolation::resample_signal(&unwrapped, target_dim)
        };
        return bound_unwrapped_angle_signal(&transformed);
    }

    let source_changes = source_dim / JOINTS;
    let target_changes = target_dim / JOINTS;
    let method = parse_method(method_name);

    let mut out = vec![0.0; target_dim];
    for joint in 0..JOINTS {
        let raw_signal: Vec<f64> = (0..source_changes)
            .map(|change| solution[change * JOINTS + joint])
            .collect();
        let signal = unwrap_angles(&raw_signal);

        let transformed =
            transform_signal_to_length(&signal, source_changes, target_changes, method);

        for (change, value) in bound_unwrapped_angle_signal(&transformed)
            .into_iter()
            .enumerate()
        {
            out[change * JOINTS + joint] = value;
        }
    }

    out
}

/// Parses a transform method name, falling back to PCHIP for unknown names.
fn parse_method(name: &str) -> TransformMethod {
    TransformMethod::from_name(name).unwrap_or(FALLBACK_METHOD)
}

/// Sorts `(position, angle)` samples, merges near-duplicate positions and unwraps the result.
///
/// # Arguments
///
/// * `samples` - The unsorted `(position, angle)` pairs; non-finite entries are dropped.
///
/// # Returns
///
/// The sorted positions clamped to `[0, 1]` and the matching unwrapped angle signal.
fn sorted_unwrapped_signal(mut samples: Vec<(f64, f64)>) -> (Vec<f64>, Vec<f64>) {
    samples.retain(|(x, y)| x.is_finite() && y.is_finite());
    samples.sort_by(|a, b| a.0.total_cmp(&b.0));

    let mut x: Vec<f64> = Vec::with_capacity(samples.len());
    let mut y: Vec<f64> = Vec::with_capacity(samples.len());
    for (sample_x, sample_y) in samples {
        // Duplicate positions would create a zero-width interpolation interval, so
        // coincident waypoints are averaged into a single sample.
        if let Some(last_x) = x.last().copied() {
            if (sample_x - last_x).abs() <= 1e-9 {
                let last = y.len() - 1;
                y[last] = 0.5 * (y[last] + sample_y);
                continue;
            }
        }
        x.push(sample_x.clamp(0.0, 1.0));
        y.push(sample_y);
    }

    (x, unwrap_angles(&y))
}

#[cfg(test)]
mod tests {
    use grahf::components::transform::SolutionTransformer;
    use mahf::Random;

    use super::*;
    use crate::robo::RoboInstance;

    /// Allowed island dimensions used by tests in this module.
    ///
    /// The real invariants are enforced at load time by
    /// `config::validate_dimensions_allowed`; these tests only need *some* valid list.
    const TEST_DIMENSIONS: [u32; 5] = [6, 12, 18, 24, 30];

    /// Builds a small instance with three well-separated targets.
    fn problem() -> RoboProblem {
        RoboProblem::new(
            RoboInstance {
                name: "3_pnts_test".to_string(),
                nr_points: 3,
                source_seed: 1,
                source_indices: vec![0, 1, 2],
                points: vec![[0.2, 0.1, 0.3], [-0.1, 0.25, 0.15], [0.05, -0.2, 0.4]],
            },
            TEST_DIMENSIONS.to_vec(),
        )
    }

    /// Builds a deterministic solution of `changes` waypoints.
    fn solution(changes: usize) -> Vec<f64> {
        (0..changes * JOINTS)
            .map(|i| ((i as f64) * 0.37).sin())
            .collect()
    }

    #[test]
    fn the_transform_hits_the_requested_dimension() {
        let problem = problem();

        for &source in &TEST_DIMENSIONS {
            for &target in &TEST_DIMENSIONS {
                let input = solution(source as usize / JOINTS);
                let out = transform_along_target_route(&problem, &input, target as usize, "PCHIP");
                assert_eq!(out.len(), target as usize, "{source} -> {target}");
            }
        }
    }

    #[test]
    fn every_method_keeps_angles_inside_the_joint_limits() {
        let problem = problem();
        let input = solution(3);

        for name in TransformMethod::all_names() {
            let out = transform_along_target_route(&problem, &input, 54, name);
            for value in out {
                assert!(value.is_finite(), "{name} produced {value}");
                assert!(value.abs() <= angles::ANGLE_LIMIT + 1e-9, "{name}: {value}");
            }
        }
    }

    #[test]
    fn an_unknown_method_falls_back_to_pchip() {
        let problem = problem();
        let input = solution(3);

        let unknown = transform_along_target_route(&problem, &input, 24, "NotAMethod");
        let pchip = transform_along_target_route(&problem, &input, 24, "PCHIP");

        assert_eq!(unknown, pchip);
    }

    #[test]
    fn degenerate_dimensions_produce_an_empty_solution() {
        let problem = problem();

        assert!(transform_along_target_route(&problem, &[], 24, "PCHIP").is_empty());
        assert!(transform_along_target_route(&problem, &solution(3), 0, "PCHIP").is_empty());
        assert!(transform_uniformly(&[], 24, "PCHIP").is_empty());
        assert!(transform_uniformly(&solution(3), 0, "PCHIP").is_empty());
    }

    #[test]
    fn dimensions_that_are_not_whole_waypoints_still_produce_the_requested_length() {
        let out = transform_uniformly(&solution(3), 19, "PCHIP");
        assert_eq!(out.len(), 19);
        assert!(out.iter().all(|v| v.is_finite()));
    }

    #[test]
    fn the_uniform_transform_preserves_a_constant_solution() {
        let input = vec![0.5; 18];

        let out = transform_uniformly(&input, 30, "PCHIP");

        assert_eq!(out.len(), 30);
        for value in out {
            assert!((value - 0.5).abs() < 1e-6, "{value}");
        }
    }

    #[test]
    fn the_transformer_short_circuits_equal_dimensions() {
        let problem = problem();
        let input = solution(4);
        let mut rng = Random::new(0);

        let out = TargetRouteTransformer::new().transform(
            &problem,
            &input,
            TransformRequest::new(24, 24, "TVDenoise"),
            &mut rng,
        );

        assert_eq!(out, input);
    }

    #[test]
    fn the_transformer_resizes_between_allowed_dimensions() {
        let problem = problem();
        let input = solution(3);
        let mut rng = Random::new(0);

        let out = TargetRouteTransformer::new().transform(
            &problem,
            &input,
            TransformRequest::new(18, 54, "Akima"),
            &mut rng,
        );

        assert_eq!(out.len(), 54);
    }

    #[test]
    fn the_transform_is_deterministic() {
        let problem = problem();
        let input = solution(3);

        let first = transform_along_target_route(&problem, &input, 48, "CT_Spline");
        let second = transform_along_target_route(&problem, &input, 48, "CT_Spline");

        assert_eq!(first, second);
    }

    #[test]
    fn coincident_sample_positions_are_merged() {
        let (x, y) = sorted_unwrapped_signal(vec![(0.0, 1.0), (0.0, 3.0), (1.0, 0.0)]);

        assert_eq!(x, vec![0.0, 1.0]);
        assert_eq!(y[0], 2.0, "coincident samples are averaged");
    }

    #[test]
    fn non_finite_samples_are_dropped() {
        let (x, _) = sorted_unwrapped_signal(vec![(0.0, 1.0), (f64::NAN, 2.0), (1.0, f64::NAN)]);

        assert_eq!(x, vec![0.0]);
    }
}
