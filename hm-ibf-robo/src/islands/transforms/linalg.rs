//! Small dense linear-algebra helpers used by the smoothing transforms.

/// Solves a tridiagonal system with the Thomas algorithm.
///
/// # Arguments
///
/// * `diag` - The main diagonal, of length `n`.
/// * `upper` - The superdiagonal, of length `n - 1`.
/// * `lower` - The subdiagonal, of length `n - 1`.
/// * `rhs` - The right-hand side, of length `n`.
///
/// # Returns
///
/// The solution vector, or an empty vector for `n == 0`.
pub(super) fn tridiagonal_thomas(
    diag: &[f64],
    upper: &[f64],
    lower: &[f64],
    rhs: &[f64],
) -> Vec<f64> {
    let n = diag.len();
    if n == 0 {
        return Vec::new();
    }
    if n == 1 {
        return vec![rhs[0] / diag[0]];
    }

    let mut c = vec![0.0; n - 1];
    let mut d = vec![0.0; n];

    c[0] = upper[0] / diag[0];
    d[0] = rhs[0] / diag[0];

    for i in 1..n {
        let denom = diag[i] - lower[i - 1] * c.get(i - 1).copied().unwrap_or(0.0);
        if i < n - 1 {
            c[i] = upper[i] / denom;
        }
        d[i] = (rhs[i] - lower[i - 1] * d[i - 1]) / denom;
    }

    let mut x = vec![0.0; n];
    x[n - 1] = d[n - 1];
    for i in (0..n - 1).rev() {
        x[i] = d[i] - c[i] * x[i + 1];
    }
    x
}

/// Solves a dense linear system with Gaussian elimination and partial pivoting.
///
/// Rows whose pivot is numerically zero contribute a zero to the solution instead of an
/// infinity, keeping the smoothing transforms finite for singular systems.
///
/// # Arguments
///
/// * `matrix` - The square coefficient matrix, consumed during elimination.
/// * `rhs` - The right-hand side, of the same length as `matrix`.
///
/// # Returns
///
/// The solution vector.
pub(super) fn solve_dense(mut matrix: Vec<Vec<f64>>, mut rhs: Vec<f64>) -> Vec<f64> {
    let n = rhs.len();

    for pivot in 0..n {
        // `total_cmp` keeps the pivot search total even if a NaN appears in the matrix;
        // `partial_cmp().unwrap()` would panic instead.
        let best_row = (pivot..n)
            .max_by(|&a, &b| matrix[a][pivot].abs().total_cmp(&matrix[b][pivot].abs()))
            .unwrap_or(pivot);
        matrix.swap(pivot, best_row);
        rhs.swap(pivot, best_row);

        let pivot_value = matrix[pivot][pivot];
        if pivot_value.abs() < 1e-12 {
            continue;
        }

        for row in pivot + 1..n {
            let factor = matrix[row][pivot] / pivot_value;
            if factor.abs() < 1e-12 {
                continue;
            }
            for col in pivot..n {
                matrix[row][col] -= factor * matrix[pivot][col];
            }
            rhs[row] -= factor * rhs[pivot];
        }
    }

    let mut out = vec![0.0; n];
    for row in (0..n).rev() {
        let sum = ((row + 1)..n)
            .map(|col| matrix[row][col] * out[col])
            .sum::<f64>();
        let denom = matrix[row][row];
        out[row] = if denom.abs() < 1e-12 {
            0.0
        } else {
            (rhs[row] - sum) / denom
        };
    }

    out
}

/// Builds the Whittaker smoothing matrix `I + lambda * D2' * D2`.
///
/// # Arguments
///
/// * `n` - The signal length; must be at least two.
/// * `lambda` - The smoothing strength.
///
/// # Returns
///
/// The `n x n` smoothing matrix.
pub(super) fn build_whittaker_matrix(n: usize, lambda: f64) -> Vec<Vec<f64>> {
    let mut matrix = vec![vec![0.0; n]; n];
    for (i, row) in matrix.iter_mut().enumerate() {
        row[i] = 1.0;
    }

    const STENCIL: [f64; 3] = [1.0, -2.0, 1.0];
    for row in 0..n.saturating_sub(2) {
        for (a, &stencil_a) in STENCIL.iter().enumerate() {
            for (b, &stencil_b) in STENCIL.iter().enumerate() {
                matrix[row + a][row + b] += lambda * stencil_a * stencil_b;
            }
        }
    }

    matrix
}

/// Builds the total-variation ADMM system matrix `I + rho * D' * D`.
///
/// # Arguments
///
/// * `n` - The signal length; must be at least one.
/// * `rho` - The ADMM penalty parameter.
///
/// # Returns
///
/// The `n x n` system matrix.
pub(super) fn build_tv_matrix(n: usize, rho: f64) -> Vec<Vec<f64>> {
    let mut matrix = vec![vec![0.0; n]; n];
    for (i, row) in matrix.iter_mut().enumerate() {
        row[i] = 1.0;
    }

    for row in 0..n.saturating_sub(1) {
        matrix[row][row] += rho;
        matrix[row][row + 1] -= rho;
        matrix[row + 1][row] -= rho;
        matrix[row + 1][row + 1] += rho;
    }

    matrix
}

/// Returns the first forward difference of `signal`.
///
/// # Arguments
///
/// * `signal` - The input samples.
///
/// # Returns
///
/// A vector of length `signal.len() - 1`, or empty for signals shorter than two.
pub(super) fn forward_diff(signal: &[f64]) -> Vec<f64> {
    signal.windows(2).map(|w| w[1] - w[0]).collect()
}

/// Adds `scale * D' * diff_values` to `target` in place.
///
/// # Arguments
///
/// * `target` - The vector to update, of length `diff_values.len() + 1`.
/// * `diff_values` - The difference-domain vector.
/// * `scale` - The scaling factor.
pub(super) fn add_scaled_dt(target: &mut [f64], diff_values: &[f64], scale: f64) {
    if diff_values.is_empty() || target.len() < 2 {
        return;
    }

    target[0] -= scale * diff_values[0];
    for i in 1..target.len() - 1 {
        target[i] += scale * (diff_values[i - 1] - diff_values[i]);
    }
    let last = target.len() - 1;
    target[last] += scale * diff_values[last - 1];
}

/// Applies the soft-thresholding (shrinkage) operator.
///
/// # Arguments
///
/// * `value` - The value to shrink.
/// * `threshold` - The non-negative shrinkage amount.
///
/// # Returns
///
/// `sign(value) * max(|value| - threshold, 0)`.
pub(super) fn soft_threshold(value: f64, threshold: f64) -> f64 {
    value.signum() * (value.abs() - threshold).max(0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-9;

    #[test]
    fn thomas_solves_a_small_tridiagonal_system() {
        // [[2, 1, 0], [1, 2, 1], [0, 1, 2]] * x = [3, 4, 3] has the solution [1, 1, 1].
        let x = tridiagonal_thomas(&[2.0, 2.0, 2.0], &[1.0, 1.0], &[1.0, 1.0], &[3.0, 4.0, 3.0]);

        for value in x {
            assert!((value - 1.0).abs() < EPS);
        }
    }

    #[test]
    fn thomas_handles_degenerate_sizes() {
        assert!(tridiagonal_thomas(&[], &[], &[], &[]).is_empty());
        assert_eq!(tridiagonal_thomas(&[2.0], &[], &[], &[6.0]), vec![3.0]);
    }

    #[test]
    fn solve_dense_solves_a_permuted_system() {
        // Row order forces a pivot swap.
        let matrix = vec![vec![0.0, 1.0], vec![1.0, 0.0]];
        let x = solve_dense(matrix, vec![2.0, 3.0]);

        assert!((x[0] - 3.0).abs() < EPS);
        assert!((x[1] - 2.0).abs() < EPS);
    }

    #[test]
    fn solve_dense_stays_finite_for_a_singular_system() {
        let matrix = vec![vec![1.0, 1.0], vec![1.0, 1.0]];
        let x = solve_dense(matrix, vec![1.0, 1.0]);

        assert!(x.iter().all(|v| v.is_finite()), "got {x:?}");
    }

    #[test]
    fn solve_dense_does_not_panic_on_nan_entries() {
        // Regression: the pivot search used `partial_cmp().unwrap()`, which panicked here.
        let matrix = vec![vec![f64::NAN, 1.0], vec![1.0, 1.0]];
        let x = solve_dense(matrix, vec![1.0, 1.0]);

        assert_eq!(x.len(), 2);
    }

    #[test]
    fn whittaker_matrix_is_the_identity_for_zero_smoothing() {
        let matrix = build_whittaker_matrix(4, 0.0);

        for (i, row) in matrix.iter().enumerate() {
            for (j, &value) in row.iter().enumerate() {
                assert_eq!(value, if i == j { 1.0 } else { 0.0 });
            }
        }
    }

    #[test]
    fn whittaker_matrix_is_symmetric() {
        let matrix = build_whittaker_matrix(5, 2.5);

        for i in 0..5 {
            for j in 0..5 {
                assert!((matrix[i][j] - matrix[j][i]).abs() < EPS);
            }
        }
    }

    #[test]
    fn tv_matrix_rows_of_the_laplacian_part_sum_to_one() {
        let matrix = build_tv_matrix(4, 3.0);

        for row in &matrix {
            assert!((row.iter().sum::<f64>() - 1.0).abs() < EPS);
        }
    }

    #[test]
    fn forward_diff_returns_consecutive_differences() {
        assert_eq!(forward_diff(&[1.0, 4.0, 9.0]), vec![3.0, 5.0]);
        assert!(forward_diff(&[1.0]).is_empty());
    }

    #[test]
    fn add_scaled_dt_is_the_adjoint_of_forward_diff() {
        // <D u, v> must equal <u, D' v> for the adjoint to be correct.
        let u = [1.0, 4.0, 9.0, 16.0];
        let v = [0.5, -1.5, 2.0];

        let lhs: f64 = forward_diff(&u).iter().zip(&v).map(|(a, b)| a * b).sum();

        let mut dt_v = vec![0.0; u.len()];
        add_scaled_dt(&mut dt_v, &v, 1.0);
        let rhs: f64 = u.iter().zip(&dt_v).map(|(a, b)| a * b).sum();

        assert!((lhs - rhs).abs() < EPS, "lhs {lhs} rhs {rhs}");
    }

    #[test]
    fn add_scaled_dt_ignores_degenerate_input() {
        let mut target = vec![1.0];
        add_scaled_dt(&mut target, &[], 1.0);
        assert_eq!(target, vec![1.0]);
    }

    #[test]
    fn soft_threshold_shrinks_towards_zero() {
        assert!((soft_threshold(3.0, 1.0) - 2.0).abs() < EPS);
        assert!((soft_threshold(-3.0, 1.0) + 2.0).abs() < EPS);
        assert_eq!(soft_threshold(0.5, 1.0), 0.0);
    }
}
