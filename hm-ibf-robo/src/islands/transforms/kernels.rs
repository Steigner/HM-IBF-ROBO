//! The seven signal transformation kernels selectable by IRACE.
//!
//! Each kernel keeps the signal length and limits it to `n_coeffs` degrees of freedom.
//! `Akima`, `ClampedCubic`, `CT_Spline`, `DouglasPeucker` and `PCHIP` interpolate their
//! knots, so a full budget reproduces the signal; `TVDenoise` and `VSpline` are
//! regularisers whose penalty stays positive even at a full budget.

use super::{
    interpolation::{
        akima_slopes, clamped_cubic_second_derivatives, cubic_hermite_eval, cubic_spline_eval,
        linear_interp, linspace, pchip_monotone_slopes, resample_at,
    },
    linalg::{
        add_scaled_dt, build_tv_matrix, build_whittaker_matrix, forward_diff, soft_threshold,
        solve_dense,
    },
};

/// Maximum ADMM iterations for the total-variation transform.
const TV_MAX_ITERATIONS: usize = 120;
/// ADMM penalty parameter for the total-variation transform.
const TV_RHO: f64 = 5.0;
/// Primal and dual residual tolerance for the total-variation transform.
const TV_TOLERANCE: f64 = 1e-6;
/// Bisection steps used to find the Douglas-Peucker epsilon matching `n_coeffs`.
const DOUGLAS_PEUCKER_BISECTIONS: usize = 20;

/// Resamples `signal` through an Akima spline over `n_coeffs` knots.
pub(super) fn transform_akima(signal: &[f64], n_coeffs: usize) -> Vec<f64> {
    let n = signal.len();
    // The Akima construction needs five knots; shorter signals are already minimal.
    if n < 5 {
        return signal.to_vec();
    }

    let t_orig = linspace(0.0, 1.0, n);
    let t_sample = linspace(0.0, 1.0, n_coeffs.max(5));
    let values = resample_at(signal, &t_sample);
    let derivatives = akima_slopes(&t_sample, &values);

    t_orig
        .into_iter()
        .map(|t| cubic_hermite_eval(&t_sample, &values, &derivatives, t))
        .collect()
}

/// Resamples `signal` through a monotone PCHIP spline over `n_coeffs` knots.
pub(super) fn transform_pchip(signal: &[f64], n_coeffs: usize) -> Vec<f64> {
    let t_orig = linspace(0.0, 1.0, signal.len());
    let t_sample = linspace(0.0, 1.0, n_coeffs.max(2));
    let values = resample_at(signal, &t_sample);
    let derivatives = pchip_monotone_slopes(&t_sample, &values);

    t_orig
        .into_iter()
        .map(|t| cubic_hermite_eval(&t_sample, &values, &derivatives, t))
        .collect()
}

/// Resamples `signal` through a clamped cubic spline over `n_coeffs` knots.
pub(super) fn transform_clamped_cubic(signal: &[f64], n_coeffs: usize) -> Vec<f64> {
    let n = signal.len();
    if n == 2 {
        return signal.to_vec();
    }

    let t_orig = linspace(0.0, 1.0, n);
    let sample_count = n_coeffs.max(2);
    let t_knots = linspace(0.0, 1.0, sample_count);
    let y_knots = resample_at(signal, &t_knots);

    if sample_count == 2 {
        return t_orig
            .into_iter()
            .map(|t| linear_interp(&t_knots, &y_knots, t))
            .collect();
    }

    let second_derivatives = clamped_cubic_second_derivatives(&t_knots, &y_knots);
    let h: Vec<f64> = t_knots.windows(2).map(|w| w[1] - w[0]).collect();

    t_orig
        .into_iter()
        .map(|t| cubic_spline_eval(&t_knots, &y_knots, &second_derivatives, &h, t))
        .collect()
}

/// Blends `signal` with a Whittaker-smoothed clamped cubic spline, weighted by the
/// compression ratio implied by `n_coeffs`.
pub(super) fn transform_ct_spline(signal: &[f64], n_coeffs: usize) -> Vec<f64> {
    let n = signal.len();
    if n <= 2 {
        return signal.to_vec();
    }

    let compression = 1.0 - (n_coeffs.min(n) as f64 / n as f64);
    if compression <= 0.0 {
        return signal.to_vec();
    }

    let spline = transform_clamped_cubic(signal, n_coeffs);
    let lambda = (compression * n as f64).powi(2).max(1e-3);
    let smoothed = solve_dense(build_whittaker_matrix(n, lambda), spline);

    signal
        .iter()
        .zip(smoothed)
        .map(|(&original, smooth)| original * (1.0 - compression) + smooth * compression)
        .collect()
}

/// Simplifies `signal` to at most `n_coeffs` vertices and re-interpolates it linearly.
pub(super) fn transform_douglas_peucker(signal: &[f64], n_coeffs: usize) -> Vec<f64> {
    let n = signal.len();
    let t_orig = linspace(0.0, 1.0, n);
    let points: Vec<[f64; 2]> = t_orig
        .iter()
        .copied()
        .zip(signal.iter().copied())
        .map(|(t, value)| [t, value])
        .collect();

    // Bisect on the simplification tolerance until the vertex budget is met.
    let (mut low, mut high) = (0.0, signal_range(signal));
    for _ in 0..DOUGLAS_PEUCKER_BISECTIONS {
        let mid = 0.5 * (low + high);
        if douglas_peucker(&points, mid).len() > n_coeffs {
            low = mid;
        } else {
            high = mid;
        }
    }

    let simplified = douglas_peucker(&points, high);
    let x: Vec<f64> = simplified.iter().map(|point| point[0]).collect();
    let y: Vec<f64> = simplified.iter().map(|point| point[1]).collect();

    t_orig
        .into_iter()
        .map(|t| linear_interp(&x, &y, t))
        .collect()
}

/// Smooths `signal` with a Whittaker penalty derived from the compression ratio.
pub(super) fn transform_vspline(signal: &[f64], n_coeffs: usize) -> Vec<f64> {
    let n = signal.len();
    if n <= 2 {
        return signal.to_vec();
    }

    let lambda = ((n as f64) / (n_coeffs.max(1) as f64)).powi(4).max(0.001);
    solve_dense(build_whittaker_matrix(n, lambda), signal.to_vec())
}

/// Denoises `signal` with total-variation regularisation solved by ADMM.
pub(super) fn transform_tv_denoise(signal: &[f64], n_coeffs: usize) -> Vec<f64> {
    let n = signal.len();
    if n <= 1 {
        return signal.to_vec();
    }

    let lambda = (0.1 * n as f64 / n_coeffs.max(1) as f64).max(1e-8);
    let matrix = build_tv_matrix(n, TV_RHO);

    let mut u = signal.to_vec();
    let mut z = forward_diff(&u);
    let mut q = vec![0.0; n - 1];

    for _ in 0..TV_MAX_ITERATIONS {
        let mut rhs = signal.to_vec();
        let z_minus_q: Vec<f64> = z.iter().zip(&q).map(|(&z_i, &q_i)| z_i - q_i).collect();
        add_scaled_dt(&mut rhs, &z_minus_q, TV_RHO);

        u = solve_dense(matrix.clone(), rhs);

        let du = forward_diff(&u);
        let z_old = z;
        z = du
            .iter()
            .zip(&q)
            .map(|(&du_i, &q_i)| soft_threshold(du_i + q_i, lambda / TV_RHO))
            .collect();

        for ((q_i, &du_i), &z_i) in q.iter_mut().zip(&du).zip(&z) {
            *q_i += du_i - z_i;
        }

        let primal = l2_norm(du.iter().zip(&z).map(|(&du_i, &z_i)| du_i - z_i));
        let dual = l2_norm(
            z.iter()
                .zip(&z_old)
                .map(|(&z_i, &old_i)| TV_RHO * (z_i - old_i)),
        );

        if primal <= TV_TOLERANCE && dual <= TV_TOLERANCE {
            break;
        }
    }

    u
}

/// Returns the Euclidean norm of an iterator of residuals.
fn l2_norm(values: impl Iterator<Item = f64>) -> f64 {
    values.map(|value| value * value).sum::<f64>().sqrt()
}

/// Returns the peak-to-peak range of `signal`, used as the upper Douglas-Peucker tolerance.
fn signal_range(signal: &[f64]) -> f64 {
    let (lo, hi) = signal
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), &value| {
            (lo.min(value), hi.max(value))
        });
    (hi - lo).abs()
}

/// Simplifies a polyline with the Douglas-Peucker algorithm.
///
/// # Arguments
///
/// * `points` - The polyline vertices.
/// * `epsilon` - The maximum allowed perpendicular deviation.
///
/// # Returns
///
/// The retained vertices, always including the first and last one.
fn douglas_peucker(points: &[[f64; 2]], epsilon: f64) -> Vec<[f64; 2]> {
    if points.len() <= 2 {
        return points.to_vec();
    }

    let first = points[0];
    let last = points[points.len() - 1];

    // Split at the interior vertex furthest from the chord between the endpoints.
    let mut d_max = -1.0;
    let mut index = 0usize;
    for (offset, &point) in points[1..points.len() - 1].iter().enumerate() {
        let d = point_line_distance(first, last, point);
        if d > d_max {
            index = offset + 1;
            d_max = d;
        }
    }

    if d_max > epsilon {
        let mut left = douglas_peucker(&points[..=index], epsilon);
        let right = douglas_peucker(&points[index..], epsilon);
        left.pop();
        left.extend(right);
        left
    } else {
        vec![first, last]
    }
}

/// Returns the perpendicular distance from `p` to the line through `p0` and `p1`.
fn point_line_distance(p0: [f64; 2], p1: [f64; 2], p: [f64; 2]) -> f64 {
    let v = [p1[0] - p0[0], p1[1] - p0[1]];
    let norm = (v[0] * v[0] + v[1] * v[1]).sqrt();
    if norm == 0.0 {
        return ((p[0] - p0[0]).powi(2) + (p[1] - p0[1]).powi(2)).sqrt();
    }
    let w = [p[0] - p0[0], p[1] - p0[1]];
    (v[0] * w[1] - v[1] * w[0]).abs() / norm
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn douglas_peucker_drops_collinear_interior_points() {
        let points = [[0.0, 0.0], [0.5, 0.5], [1.0, 1.0]];

        assert_eq!(douglas_peucker(&points, 1e-9).len(), 2);
    }

    #[test]
    fn douglas_peucker_keeps_a_significant_corner() {
        let points = [[0.0, 0.0], [0.5, 5.0], [1.0, 0.0]];

        assert_eq!(douglas_peucker(&points, 0.1).len(), 3);
    }

    #[test]
    fn douglas_peucker_keeps_short_polylines_verbatim() {
        let points = [[0.0, 0.0], [1.0, 1.0]];

        assert_eq!(douglas_peucker(&points, 0.0), points.to_vec());
    }

    #[test]
    fn point_line_distance_falls_back_to_the_point_distance_for_degenerate_lines() {
        let d = point_line_distance([1.0, 1.0], [1.0, 1.0], [1.0, 4.0]);
        assert!((d - 3.0).abs() < 1e-9);
    }

    #[test]
    fn point_line_distance_measures_the_perpendicular_offset() {
        let d = point_line_distance([0.0, 0.0], [1.0, 0.0], [0.5, 2.0]);
        assert!((d - 2.0).abs() < 1e-9);
    }

    #[test]
    fn the_signal_range_is_the_peak_to_peak_amplitude() {
        assert!((signal_range(&[-1.0, 0.0, 3.0]) - 4.0).abs() < 1e-9);
        assert_eq!(signal_range(&[2.0, 2.0]), 0.0);
    }

    #[test]
    fn the_l2_norm_matches_the_euclidean_length() {
        assert!((l2_norm([3.0, 4.0].into_iter()) - 5.0).abs() < 1e-9);
        assert_eq!(l2_norm(std::iter::empty()), 0.0);
    }
}
