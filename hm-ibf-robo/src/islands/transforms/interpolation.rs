//! Sampling grids, linear interpolation and cubic spline slope estimation.

/// Returns `n` evenly spaced values covering `[start, end]`.
///
/// # Arguments
///
/// * `start` - The first value.
/// * `end` - The last value.
/// * `n` - The number of samples.
///
/// # Returns
///
/// The grid; empty for `n == 0` and `[start]` for `n == 1`.
pub(super) fn linspace(start: f64, end: f64, n: usize) -> Vec<f64> {
    match n {
        0 => Vec::new(),
        1 => vec![start],
        _ => (0..n)
            .map(|i| start + (end - start) * i as f64 / (n - 1) as f64)
            .collect(),
    }
}

/// Interpolates `y` over the grid `x` at position `t`, extrapolating linearly beyond the ends.
///
/// Extrapolating rather than clamping prevents duplicate waypoints when the source grid does
/// not span the full range of the target grid.
///
/// # Arguments
///
/// * `x` - The strictly increasing sample positions; must be non-empty and as long as `y`.
/// * `y` - The sample values.
/// * `t` - The position to evaluate at.
///
/// # Returns
///
/// The interpolated value.
///
/// # Panics
///
/// Panics if `x` is empty or shorter than `y`.
pub(super) fn linear_interp(x: &[f64], y: &[f64], t: f64) -> f64 {
    let n = x.len();
    if n == 1 {
        return y[0];
    }

    if t <= x[0] {
        let span = (x[1] - x[0]).max(1e-12);
        let slope = (y[1] - y[0]) / span;
        return y[0] + slope * (t - x[0]);
    }
    if t >= x[n - 1] {
        let span = (x[n - 1] - x[n - 2]).max(1e-12);
        let slope = (y[n - 1] - y[n - 2]) / span;
        return y[n - 1] + slope * (t - x[n - 1]);
    }

    let mut hi = 1usize;
    while hi < n && x[hi] < t {
        hi += 1;
    }
    let lo = hi.saturating_sub(1);
    let span = (x[hi] - x[lo]).max(1e-12);
    let frac = (t - x[lo]) / span;
    y[lo] * (1.0 - frac) + y[hi] * frac
}

/// Resamples `signal` (assumed on a uniform `[0, 1]` grid) at the given positions.
///
/// # Arguments
///
/// * `signal` - The source samples.
/// * `points` - The positions to evaluate at.
///
/// # Returns
///
/// One value per entry of `points`.
pub(super) fn resample_at(signal: &[f64], points: &[f64]) -> Vec<f64> {
    let x = linspace(0.0, 1.0, signal.len());
    points
        .iter()
        .map(|&t| linear_interp(&x, signal, t))
        .collect()
}

/// Resamples `signal` (assumed on a uniform `[0, 1]` grid) to `target_len` samples.
///
/// # Arguments
///
/// * `signal` - The source samples.
/// * `target_len` - The requested number of samples.
///
/// # Returns
///
/// The resampled signal, or an empty vector if either input is empty.
pub(super) fn resample_signal(signal: &[f64], target_len: usize) -> Vec<f64> {
    if signal.is_empty() || target_len == 0 {
        return Vec::new();
    }
    let x = linspace(0.0, 1.0, signal.len());
    linspace(0.0, 1.0, target_len)
        .into_iter()
        .map(|t| linear_interp(&x, signal, t))
        .collect()
}

/// Resamples a signal given on the explicit grid `x` at the positions `target_t`.
///
/// # Arguments
///
/// * `x` - The source sample positions.
/// * `signal` - The source sample values.
/// * `target_t` - The positions to evaluate at.
///
/// # Returns
///
/// One value per entry of `target_t`, or an empty vector if `signal` is empty.
pub(super) fn resample_from_points(x: &[f64], signal: &[f64], target_t: &[f64]) -> Vec<f64> {
    if signal.is_empty() {
        return Vec::new();
    }
    target_t
        .iter()
        .map(|&t| linear_interp(x, signal, t))
        .collect()
}

/// Returns the index of the interval of `x` that contains `t`, clamped to the valid range.
///
/// # Arguments
///
/// * `x` - The strictly increasing grid; must contain at least two entries.
/// * `t` - The position to locate.
///
/// # Returns
///
/// The index `i` with `x[i] <= t <= x[i + 1]` after clamping.
///
/// # Panics
///
/// Panics if `x` contains fewer than two entries.
pub(super) fn find_interval(x: &[f64], t: f64) -> usize {
    if t <= x[0] {
        return 0;
    }
    if t >= x[x.len() - 1] {
        return x.len() - 2;
    }

    let mut hi = 1usize;
    while hi < x.len() && x[hi] < t {
        hi += 1;
    }
    hi.saturating_sub(1).min(x.len() - 2)
}

/// Evaluates a cubic Hermite spline defined by values `y` and slopes `d` on the grid `x`.
///
/// # Arguments
///
/// * `x` - The knot positions; at least two entries.
/// * `y` - The knot values.
/// * `d` - The knot slopes.
/// * `t` - The position to evaluate at.
///
/// # Returns
///
/// The spline value at `t`.
pub(super) fn cubic_hermite_eval(x: &[f64], y: &[f64], d: &[f64], t: f64) -> f64 {
    let idx = find_interval(x, t);
    let h = (x[idx + 1] - x[idx]).max(1e-12);
    let s = ((t - x[idx]) / h).clamp(0.0, 1.0);

    let h00 = 2.0 * s.powi(3) - 3.0 * s.powi(2) + 1.0;
    let h10 = s.powi(3) - 2.0 * s.powi(2) + s;
    let h01 = -2.0 * s.powi(3) + 3.0 * s.powi(2);
    let h11 = s.powi(3) - s.powi(2);

    h00 * y[idx] + h10 * h * d[idx] + h01 * y[idx + 1] + h11 * h * d[idx + 1]
}

/// Evaluates a natural-form cubic spline from its second derivatives.
///
/// # Arguments
///
/// * `x` - The knot positions; at least two entries.
/// * `y` - The knot values.
/// * `m` - The second derivatives at the knots.
/// * `h` - The knot spacings, i.e. `x[i + 1] - x[i]`.
/// * `t` - The position to evaluate at.
///
/// # Returns
///
/// The spline value at `t`.
pub(super) fn cubic_spline_eval(x: &[f64], y: &[f64], m: &[f64], h: &[f64], t: f64) -> f64 {
    let idx = find_interval(x, t);
    let span = h[idx].max(1e-12);
    let a = (x[idx + 1] - t) / span;
    let b = (t - x[idx]) / span;

    a * y[idx]
        + b * y[idx + 1]
        + ((a.powi(3) - a) * m[idx] + (b.powi(3) - b) * m[idx + 1]) * span.powi(2) / 6.0
}

/// Estimates Akima slopes, falling back to PCHIP for fewer than five knots.
///
/// # Arguments
///
/// * `x` - The knot positions.
/// * `y` - The knot values.
///
/// # Returns
///
/// One slope per knot.
pub(super) fn akima_slopes(x: &[f64], y: &[f64]) -> Vec<f64> {
    let n = x.len();
    if n < 5 {
        return pchip_monotone_slopes(x, y);
    }

    let slopes: Vec<f64> = (0..n - 1)
        .map(|i| (y[i + 1] - y[i]) / (x[i + 1] - x[i]).max(1e-12))
        .collect();
    let mut derivatives = vec![0.0; n];

    derivatives[0] = slopes[0];
    derivatives[1] = 0.5 * (slopes[0] + slopes[1]);
    derivatives[n - 2] = 0.5 * (slopes[n - 3] + slopes[n - 2]);
    derivatives[n - 1] = slopes[n - 2];

    for i in 2..n - 2 {
        let w_left = (slopes[i + 1] - slopes[i]).abs();
        let w_right = (slopes[i - 1] - slopes[i - 2]).abs();
        let denom = w_left + w_right;

        derivatives[i] = if denom <= 1e-12 {
            0.5 * (slopes[i - 1] + slopes[i])
        } else {
            (w_left * slopes[i - 1] + w_right * slopes[i]) / denom
        };
    }

    derivatives
}

/// Estimates monotonicity-preserving PCHIP slopes.
///
/// # Arguments
///
/// * `x` - The knot positions; at least two entries.
/// * `y` - The knot values.
///
/// # Returns
///
/// One slope per knot.
///
/// # Panics
///
/// Panics if `x` contains fewer than two entries.
pub(super) fn pchip_monotone_slopes(x: &[f64], y: &[f64]) -> Vec<f64> {
    let n = x.len();
    if n == 2 {
        let slope = (y[1] - y[0]) / (x[1] - x[0]).max(1e-12);
        return vec![slope; 2];
    }

    let h: Vec<f64> = x.windows(2).map(|w| w[1] - w[0]).collect();
    let delta: Vec<f64> = (0..n - 1)
        .map(|i| (y[i + 1] - y[i]) / h[i].max(1e-12))
        .collect();
    let mut d = vec![0.0; n];

    for k in 1..n - 1 {
        if delta[k - 1] == 0.0 || delta[k] == 0.0 || delta[k - 1].signum() != delta[k].signum() {
            d[k] = 0.0;
        } else {
            let w1 = 2.0 * h[k] + h[k - 1];
            let w2 = h[k] + 2.0 * h[k - 1];
            d[k] = (w1 + w2) / (w1 / delta[k - 1] + w2 / delta[k]);
        }
    }

    let d0 = ((2.0 * h[0] + h[1]) * delta[0] - h[0] * delta[1]) / (h[0] + h[1]).max(1e-12);
    d[0] = clamp_endpoint_slope(d0, delta[0], delta[1]);

    let dn = ((2.0 * h[n - 2] + h[n - 3]) * delta[n - 2] - h[n - 2] * delta[n - 3])
        / (h[n - 2] + h[n - 3]).max(1e-12);
    d[n - 1] = clamp_endpoint_slope(dn, delta[n - 2], delta[n - 3]);

    d
}

/// Limits an endpoint slope so the spline stays monotone near the boundary.
///
/// # Arguments
///
/// * `candidate` - The unconstrained endpoint slope.
/// * `delta_edge` - The secant slope of the boundary interval.
/// * `delta_neighbor` - The secant slope of the adjacent interval.
///
/// # Returns
///
/// The limited slope.
fn clamp_endpoint_slope(candidate: f64, delta_edge: f64, delta_neighbor: f64) -> f64 {
    if candidate.signum() != delta_edge.signum() {
        0.0
    } else if delta_edge.signum() != delta_neighbor.signum()
        && candidate.abs() > 3.0 * delta_edge.abs()
    {
        3.0 * delta_edge
    } else {
        candidate
    }
}

/// Solves for the second derivatives of a clamped cubic spline.
///
/// # Arguments
///
/// * `x` - The knot positions.
/// * `y` - The knot values.
///
/// # Returns
///
/// One second derivative per knot; all zero for fewer than three knots.
pub(super) fn clamped_cubic_second_derivatives(x: &[f64], y: &[f64]) -> Vec<f64> {
    let n = x.len();
    if n <= 2 {
        return vec![0.0; n];
    }

    let h: Vec<f64> = x.windows(2).map(|w| w[1] - w[0]).collect();
    let delta: Vec<f64> = (0..n - 1)
        .map(|i| (y[i + 1] - y[i]) / h[i].max(1e-12))
        .collect();

    let mut diag = vec![0.0; n];
    let mut upper = vec![0.0; n - 1];
    let mut lower = vec![0.0; n - 1];
    let mut rhs = vec![0.0; n];

    diag[0] = 2.0 * h[0];
    upper[0] = h[0];
    rhs[0] = 6.0 * delta[0];

    for i in 1..n - 1 {
        lower[i - 1] = h[i - 1];
        diag[i] = 2.0 * (h[i - 1] + h[i]);
        upper[i] = h[i];
        rhs[i] = 6.0 * (delta[i] - delta[i - 1]);
    }

    lower[n - 2] = h[n - 2];
    diag[n - 1] = 2.0 * h[n - 2];
    rhs[n - 1] = -6.0 * delta[n - 2];

    super::linalg::tridiagonal_thomas(&diag, &upper, &lower, &rhs)
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-9;

    #[test]
    fn linspace_covers_the_requested_range() {
        assert_eq!(linspace(0.0, 1.0, 0), Vec::<f64>::new());
        assert_eq!(linspace(2.0, 5.0, 1), vec![2.0]);
        assert_eq!(linspace(0.0, 1.0, 3), vec![0.0, 0.5, 1.0]);
    }

    #[test]
    fn linear_interp_reproduces_the_knots() {
        let x = [0.0, 0.5, 1.0];
        let y = [0.0, 4.0, 2.0];

        for (&xi, &yi) in x.iter().zip(&y) {
            assert!((linear_interp(&x, &y, xi) - yi).abs() < EPS);
        }
    }

    #[test]
    fn linear_interp_extrapolates_beyond_both_ends() {
        let x = [0.0, 1.0];
        let y = [0.0, 2.0];

        assert!((linear_interp(&x, &y, -1.0) - -2.0).abs() < EPS);
        assert!((linear_interp(&x, &y, 2.0) - 4.0).abs() < EPS);
    }

    #[test]
    fn linear_interp_handles_a_single_sample() {
        assert_eq!(linear_interp(&[0.0], &[7.0], 42.0), 7.0);
    }

    #[test]
    fn resample_signal_preserves_a_linear_ramp() {
        let signal: Vec<f64> = (0..5).map(|i| i as f64).collect();

        let resampled = resample_signal(&signal, 9);

        assert_eq!(resampled.len(), 9);
        for (index, value) in resampled.iter().enumerate() {
            assert!((value - index as f64 / 2.0).abs() < EPS, "index {index}");
        }
    }

    #[test]
    fn resample_signal_returns_empty_for_degenerate_input() {
        assert!(resample_signal(&[], 4).is_empty());
        assert!(resample_signal(&[1.0, 2.0], 0).is_empty());
    }

    #[test]
    fn find_interval_clamps_outside_the_grid() {
        let x = [0.0, 1.0, 2.0];
        assert_eq!(find_interval(&x, -5.0), 0);
        assert_eq!(find_interval(&x, 0.5), 0);
        assert_eq!(find_interval(&x, 1.5), 1);
        assert_eq!(find_interval(&x, 5.0), 1);
    }

    #[test]
    fn cubic_hermite_eval_interpolates_the_knots_exactly() {
        let x = [0.0, 1.0];
        let y = [1.0, 3.0];
        let d = [2.0, 2.0];

        assert!((cubic_hermite_eval(&x, &y, &d, 0.0) - 1.0).abs() < EPS);
        assert!((cubic_hermite_eval(&x, &y, &d, 1.0) - 3.0).abs() < EPS);
        // With matching slopes the segment is exactly the straight line through the knots.
        assert!((cubic_hermite_eval(&x, &y, &d, 0.5) - 2.0).abs() < EPS);
    }

    #[test]
    fn pchip_slopes_are_flat_at_local_extrema() {
        let x = [0.0, 1.0, 2.0];
        let y = [0.0, 1.0, 0.0];

        let slopes = pchip_monotone_slopes(&x, &y);

        assert_eq!(slopes[1], 0.0, "no overshoot at the peak");
    }

    #[test]
    fn pchip_slopes_match_the_secant_for_a_straight_line() {
        let x = [0.0, 1.0, 2.0, 3.0];
        let y = [0.0, 2.0, 4.0, 6.0];

        for slope in pchip_monotone_slopes(&x, &y) {
            assert!((slope - 2.0).abs() < EPS);
        }
    }

    #[test]
    fn akima_slopes_delegate_to_pchip_for_short_signals() {
        let x = [0.0, 1.0, 2.0];
        let y = [0.0, 1.0, 0.0];

        assert_eq!(akima_slopes(&x, &y), pchip_monotone_slopes(&x, &y));
    }

    #[test]
    fn akima_slopes_are_linear_for_a_straight_line() {
        let x: Vec<f64> = (0..6).map(|i| i as f64).collect();
        let y: Vec<f64> = x.iter().map(|v| 3.0 * v).collect();

        for slope in akima_slopes(&x, &y) {
            assert!((slope - 3.0).abs() < 1e-9);
        }
    }

    #[test]
    fn the_clamped_cubic_spline_interpolates_its_knots() {
        // The boundary rows impose zero end slopes, so the second derivatives do not vanish
        // even for a straight line; what must hold is that the spline passes through the
        // knots exactly.
        let x = [0.0, 1.0, 2.0, 3.0];
        let y = [0.0, 1.0, 4.0, 9.0];

        let m = clamped_cubic_second_derivatives(&x, &y);
        let h: Vec<f64> = x.windows(2).map(|w| w[1] - w[0]).collect();

        for (&xi, &yi) in x.iter().zip(&y) {
            let value = cubic_spline_eval(&x, &y, &m, &h, xi);
            assert!((value - yi).abs() < 1e-9, "at {xi}: {value} != {yi}");
        }
    }

    #[test]
    fn clamped_cubic_second_derivatives_are_trivial_for_short_grids() {
        assert_eq!(
            clamped_cubic_second_derivatives(&[0.0, 1.0], &[0.0, 1.0]),
            vec![0.0, 0.0]
        );
    }
}
