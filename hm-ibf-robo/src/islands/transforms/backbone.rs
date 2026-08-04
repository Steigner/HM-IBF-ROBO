//! Arc-length parameterisation of a trajectory along the instance's target route.
//!
//! Migrating between islands with different waypoint counts needs a common parameter for
//! "how far along the trajectory" a waypoint sits. Interpolating on the waypoint index
//! would be wrong because islands place different numbers of waypoints on the same path.
//! Instead every waypoint is projected onto the shortest polyline through the instance's
//! target points, which yields a resolution-independent position in `[0, 1]`.

use crate::robo::RoboProblem;

/// Minimum route length below which the backbone carries no usable parameterisation.
const BACKBONE_EPS: f64 = 1e-12;

/// The polyline through the start position and the instance's target points.
pub(super) struct TargetRouteBackbone {
    points: Vec<[f64; 3]>,
    cumulative: Vec<f64>,
    total_len: f64,
}

impl TargetRouteBackbone {
    /// Builds the backbone from a problem instance's targets.
    ///
    /// The targets are ordered by an exact shortest open tour starting at the end-effector's
    /// home position.
    ///
    /// # Arguments
    ///
    /// * `problem` - The instance supplying the initial pose and the target points.
    ///
    /// # Returns
    ///
    /// The backbone, or `None` if the route is degenerate (fewer than two distinct points).
    pub(super) fn from_problem(problem: &RoboProblem) -> Option<Self> {
        let start = RoboProblem::end_effector_position(&problem.initial_angles);
        let order = exact_shortest_target_order(start, &problem.instance.points);

        let mut route = Vec::with_capacity(problem.instance.points.len() + 1);
        route.push(start);
        route.extend(
            order
                .into_iter()
                .map(|index| problem.instance.points[index]),
        );

        Self::new(route)
    }

    /// Builds the backbone from an explicit polyline.
    ///
    /// # Arguments
    ///
    /// * `points` - The polyline vertices in traversal order.
    ///
    /// # Returns
    ///
    /// The backbone, or `None` if it has fewer than two vertices or zero length.
    pub(super) fn new(points: Vec<[f64; 3]>) -> Option<Self> {
        if points.len() < 2 {
            return None;
        }

        let mut cumulative = Vec::with_capacity(points.len());
        cumulative.push(0.0);
        for segment in points.windows(2) {
            let length = squared_distance(segment[0], segment[1]).sqrt();
            cumulative.push(cumulative.last().copied().unwrap_or(0.0) + length);
        }

        let total_len = cumulative.last().copied().unwrap_or(0.0);
        (total_len > BACKBONE_EPS).then_some(Self {
            points,
            cumulative,
            total_len,
        })
    }

    /// Projects a Cartesian point onto the backbone.
    ///
    /// # Arguments
    ///
    /// * `point` - The end-effector position to project.
    ///
    /// # Returns
    ///
    /// The normalized arc-length position of the closest point on the backbone, in `[0, 1]`.
    pub(super) fn project_t(&self, point: [f64; 3]) -> f64 {
        let mut best_distance = f64::INFINITY;
        let mut best_s = 0.0;

        for index in 0..self.points.len() - 1 {
            let a = self.points[index];
            let b = self.points[index + 1];
            let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
            let ap = [point[0] - a[0], point[1] - a[1], point[2] - a[2]];
            // `denom` is the squared segment length, so its root is the segment length.
            let denom = dot3(ab, ab);
            let local_t = if denom <= BACKBONE_EPS {
                0.0
            } else {
                (dot3(ap, ab) / denom).clamp(0.0, 1.0)
            };
            let projection = [
                a[0] + local_t * ab[0],
                a[1] + local_t * ab[1],
                a[2] + local_t * ab[2],
            ];
            let distance = squared_distance(point, projection);

            if distance < best_distance {
                best_distance = distance;
                best_s = self.cumulative[index] + local_t * denom.sqrt();
            }
        }

        (best_s / self.total_len).clamp(0.0, 1.0)
    }
}

/// Returns the backbone position of every waypoint of a flattened solution.
///
/// # Arguments
///
/// * `solution` - The flattened joint angles, row-major with `JOINTS` values per waypoint.
/// * `source_changes` - The number of waypoints in `solution`.
/// * `backbone` - The backbone to project onto.
///
/// # Returns
///
/// One normalized position per waypoint.
pub(super) fn source_backbone_positions(
    solution: &[f64],
    source_changes: usize,
    backbone: &TargetRouteBackbone,
) -> Vec<f64> {
    const JOINTS: usize = crate::robo::JOINTS;

    (0..source_changes)
        .map(|change| {
            let start = change * JOINTS;
            let mut angles = [0.0; JOINTS];
            angles.copy_from_slice(&solution[start..start + JOINTS]);
            backbone.project_t(RoboProblem::end_effector_position(&angles))
        })
        .collect()
}

/// Returns the target order of a shortest open tour starting at `start`.
///
/// Instances hold at most six targets, so the branch-and-bound search is exhaustive and
/// negligible next to a single objective evaluation.
///
/// # Arguments
///
/// * `start` - The tour's starting position.
/// * `targets` - The points to visit.
///
/// # Returns
///
/// Indices into `targets` in visiting order; empty if `targets` is empty.
pub(super) fn exact_shortest_target_order(start: [f64; 3], targets: &[[f64; 3]]) -> Vec<usize> {
    let mut used = vec![false; targets.len()];
    let mut current = Vec::with_capacity(targets.len());
    let mut best = Vec::new();
    let mut best_len = f64::INFINITY;

    search_target_order(
        start,
        targets,
        &mut used,
        &mut current,
        0.0,
        &mut best_len,
        &mut best,
    );

    best
}

/// Depth-first branch-and-bound step of [`exact_shortest_target_order`].
#[allow(clippy::too_many_arguments)]
fn search_target_order(
    current_point: [f64; 3],
    targets: &[[f64; 3]],
    used: &mut [bool],
    current: &mut Vec<usize>,
    current_len: f64,
    best_len: &mut f64,
    best: &mut Vec<usize>,
) {
    if current.len() == targets.len() {
        if current_len < *best_len {
            *best_len = current_len;
            *best = current.clone();
        }
        return;
    }

    // Prune: the tour length only grows, so a prefix at or above the incumbent cannot win.
    if current_len >= *best_len {
        return;
    }

    for index in 0..targets.len() {
        if used[index] {
            continue;
        }

        used[index] = true;
        current.push(index);
        search_target_order(
            targets[index],
            targets,
            used,
            current,
            current_len + squared_distance(current_point, targets[index]).sqrt(),
            best_len,
            best,
        );
        current.pop();
        used[index] = false;
    }
}

/// Returns the dot product of two 3D vectors.
fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// Returns the squared Euclidean distance between two 3D points.
fn squared_distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    (a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-9;

    /// A straight backbone of total length 2 along the x axis.
    fn straight_backbone() -> TargetRouteBackbone {
        TargetRouteBackbone::new(vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]).unwrap()
    }

    #[test]
    fn a_backbone_needs_at_least_two_distinct_points() {
        assert!(TargetRouteBackbone::new(vec![]).is_none());
        assert!(TargetRouteBackbone::new(vec![[0.0; 3]]).is_none());
        assert!(
            TargetRouteBackbone::new(vec![[1.0; 3], [1.0; 3]]).is_none(),
            "a zero-length route carries no parameterisation"
        );
    }

    #[test]
    fn projection_maps_the_endpoints_to_zero_and_one() {
        let backbone = straight_backbone();

        assert!(backbone.project_t([0.0, 0.0, 0.0]).abs() < EPS);
        assert!((backbone.project_t([2.0, 0.0, 0.0]) - 1.0).abs() < EPS);
    }

    #[test]
    fn projection_measures_arc_length_not_the_vertex_index() {
        let backbone = straight_backbone();
        assert!((backbone.project_t([0.5, 0.0, 0.0]) - 0.25).abs() < EPS);
        assert!((backbone.project_t([1.5, 0.0, 0.0]) - 0.75).abs() < EPS);
    }

    #[test]
    fn projection_clamps_points_beyond_the_route() {
        let backbone = straight_backbone();

        assert!(backbone.project_t([-5.0, 0.0, 0.0]).abs() < EPS);
        assert!((backbone.project_t([9.0, 0.0, 0.0]) - 1.0).abs() < EPS);
    }

    #[test]
    fn projection_ignores_the_offset_perpendicular_to_the_route() {
        let backbone = straight_backbone();
        assert!((backbone.project_t([1.0, 3.0, -4.0]) - 0.5).abs() < EPS);
    }

    #[test]
    fn the_shortest_order_visits_the_nearest_chain_first() {
        let targets = [[3.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]];

        let order = exact_shortest_target_order([0.0, 0.0, 0.0], &targets);

        assert_eq!(order, vec![1, 2, 0]);
    }

    #[test]
    fn the_shortest_order_of_no_targets_is_empty() {
        assert!(exact_shortest_target_order([0.0; 3], &[]).is_empty());
    }

    #[test]
    fn the_shortest_order_is_a_permutation_of_all_targets() {
        let targets = [
            [1.0, 2.0, 3.0],
            [-4.0, 0.5, 1.0],
            [0.0, 0.0, 9.0],
            [2.0, -2.0, 0.0],
            [5.0, 5.0, 5.0],
        ];

        let mut order = exact_shortest_target_order([0.0; 3], &targets);
        order.sort_unstable();

        assert_eq!(order, vec![0, 1, 2, 3, 4]);
    }
}
