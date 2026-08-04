//! The signal transformation methods available to GRAHF migrations.
//!
//! Each method compresses a joint-angle signal to `n_coeffs` degrees of freedom while
//! keeping its length, mirroring the reference implementation in `robo-evo-apps`.

use serde::{Deserialize, Serialize};

use super::{
    interpolation::resample_signal,
    kernels::{
        transform_akima, transform_clamped_cubic, transform_ct_spline, transform_douglas_peucker,
        transform_pchip, transform_tv_denoise, transform_vspline,
    },
};

/// A signal transformation method selectable by IRACE for a migration edge.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum TransformMethod {
    /// Akima spline resampling.
    Akima,
    /// Clamped cubic spline resampling.
    ClampedCubic,
    /// Clamped cubic spline blended with Whittaker smoothing.
    CtSpline,
    /// Douglas-Peucker polyline simplification.
    DouglasPeucker,
    /// Monotone piecewise cubic Hermite resampling.
    Pchip,
    /// Total-variation denoising solved with ADMM.
    TvDenoise,
    /// Whittaker (variational) spline smoothing.
    VSpline,
}

impl TransformMethod {
    /// Returns the IRACE-facing names of all methods, in declaration order.
    ///
    /// # Returns
    ///
    /// The categorical parameter values for `transform_method`.
    pub fn all_names() -> Vec<&'static str> {
        vec![
            "Akima",
            "ClampedCubic",
            "CT_Spline",
            "DouglasPeucker",
            "PCHIP",
            "TVDenoise",
            "VSpline",
        ]
    }

    /// Parses a method from its IRACE-facing name.
    ///
    /// # Arguments
    ///
    /// * `name` - One of the values returned by [`TransformMethod::all_names`].
    ///
    /// # Returns
    ///
    /// The method, or `None` for an unknown name.
    pub fn from_name(name: &str) -> Option<Self> {
        match name {
            "Akima" => Some(Self::Akima),
            "ClampedCubic" => Some(Self::ClampedCubic),
            "CT_Spline" => Some(Self::CtSpline),
            "DouglasPeucker" => Some(Self::DouglasPeucker),
            "PCHIP" => Some(Self::Pchip),
            "TVDenoise" => Some(Self::TvDenoise),
            "VSpline" => Some(Self::VSpline),
            _ => None,
        }
    }

    /// Returns the IRACE-facing name of this method.
    pub fn name(self) -> &'static str {
        match self {
            Self::Akima => "Akima",
            Self::ClampedCubic => "ClampedCubic",
            Self::CtSpline => "CT_Spline",
            Self::DouglasPeucker => "DouglasPeucker",
            Self::Pchip => "PCHIP",
            Self::TvDenoise => "TVDenoise",
            Self::VSpline => "VSpline",
        }
    }
}

/// Resamples `signal` to `target_len` and applies `method` with `n_coeffs` degrees of freedom.
///
/// # Arguments
///
/// * `signal` - The source samples.
/// * `n_coeffs` - The complexity budget of the transform.
/// * `target_len` - The requested output length.
/// * `method` - The transformation method.
///
/// # Returns
///
/// The transformed signal of length `target_len`, or an empty vector for degenerate input.
pub(super) fn transform_signal_to_length(
    signal: &[f64],
    n_coeffs: usize,
    target_len: usize,
    method: TransformMethod,
) -> Vec<f64> {
    if target_len == 0 || signal.is_empty() {
        return Vec::new();
    }

    if signal.len() == target_len {
        return apply_method_same_len(signal, n_coeffs, method);
    }

    let proxy = resample_signal(signal, target_len);
    apply_method_same_len(&proxy, n_coeffs.min(target_len), method)
}

/// Applies `method` to `signal`, keeping its length and limiting it to `n_coeffs` degrees of
/// freedom.
///
/// # Arguments
///
/// * `signal` - The source samples.
/// * `n_coeffs` - The complexity budget, clamped into `1..=signal.len()`.
/// * `method` - The transformation method.
///
/// # Returns
///
/// The transformed signal, of the same length as `signal`.
pub(super) fn apply_method_same_len(
    signal: &[f64],
    n_coeffs: usize,
    method: TransformMethod,
) -> Vec<f64> {
    let n = signal.len();
    if n <= 1 {
        return signal.to_vec();
    }

    let n_coeffs = n_coeffs.clamp(1, n);
    match method {
        TransformMethod::Akima => transform_akima(signal, n_coeffs),
        TransformMethod::Pchip => transform_pchip(signal, n_coeffs),
        TransformMethod::ClampedCubic => transform_clamped_cubic(signal, n_coeffs),
        TransformMethod::CtSpline => transform_ct_spline(signal, n_coeffs),
        TransformMethod::DouglasPeucker => transform_douglas_peucker(signal, n_coeffs),
        TransformMethod::TvDenoise => transform_tv_denoise(signal, n_coeffs),
        TransformMethod::VSpline => transform_vspline(signal, n_coeffs),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every method, used to assert invariants that must hold for all of them.
    fn all_methods() -> Vec<TransformMethod> {
        TransformMethod::all_names()
            .into_iter()
            .map(|name| TransformMethod::from_name(name).unwrap())
            .collect()
    }

    #[test]
    fn every_name_round_trips_through_the_enum() {
        for name in TransformMethod::all_names() {
            let method = TransformMethod::from_name(name).expect(name);
            assert_eq!(method.name(), name);
        }
    }

    #[test]
    fn unknown_names_are_rejected() {
        assert_eq!(TransformMethod::from_name("Nope"), None);
        assert_eq!(TransformMethod::from_name(""), None);
    }

    #[test]
    fn all_names_covers_every_variant() {
        assert_eq!(all_methods().len(), 7);
    }

    #[test]
    fn every_method_preserves_the_signal_length() {
        let signal: Vec<f64> = (0..12).map(|i| (i as f64 * 0.4).sin()).collect();

        for method in all_methods() {
            let out = apply_method_same_len(&signal, 5, method);
            assert_eq!(out.len(), signal.len(), "{method:?}");
        }
    }

    #[test]
    fn every_method_produces_finite_values() {
        let signal: Vec<f64> = (0..16).map(|i| ((i % 4) as f64) - 1.5).collect();

        for method in all_methods() {
            for value in apply_method_same_len(&signal, 3, method) {
                assert!(value.is_finite(), "{method:?} produced {value}");
            }
        }
    }

    #[test]
    fn every_method_is_a_noop_for_signals_shorter_than_two() {
        for method in all_methods() {
            assert_eq!(apply_method_same_len(&[], 3, method), Vec::<f64>::new());
            assert_eq!(apply_method_same_len(&[1.5], 3, method), vec![1.5]);
        }
    }

    #[test]
    fn every_method_reproduces_a_constant_signal() {
        let signal = vec![0.75; 10];

        for method in all_methods() {
            for value in apply_method_same_len(&signal, 4, method) {
                assert!((value - 0.75).abs() < 1e-6, "{method:?} produced {value}");
            }
        }
    }

    #[test]
    fn a_full_budget_leaves_the_interpolating_methods_untouched() {
        // The interpolating methods place a knot on every sample at a full budget, so they
        // reproduce the signal exactly.
        let signal: Vec<f64> = (0..10).map(|i| i as f64 * 0.3).collect();
        let interpolating = [
            TransformMethod::Akima,
            TransformMethod::ClampedCubic,
            TransformMethod::CtSpline,
            TransformMethod::DouglasPeucker,
            TransformMethod::Pchip,
        ];

        for method in interpolating {
            let out = apply_method_same_len(&signal, signal.len(), method);
            for (before, after) in signal.iter().zip(&out) {
                assert!(
                    (before - after).abs() < 1e-3,
                    "{method:?}: {before} vs {after}"
                );
            }
        }
    }

    #[test]
    fn the_penalty_based_methods_smooth_even_at_a_full_budget() {
        // `TvDenoise` and `VSpline` are regularisers, not interpolants: their penalty stays
        // positive at a full budget, so they always pull the signal towards a smoother one.
        let mut signal = vec![0.0; 11];
        signal[5] = 1.0;

        for method in [TransformMethod::TvDenoise, TransformMethod::VSpline] {
            let out = apply_method_same_len(&signal, signal.len(), method);
            assert!(out[5] < 1.0, "{method:?} left the spike untouched: {out:?}");
            assert!(out.iter().all(|v| v.is_finite()), "{method:?}");
        }
    }

    #[test]
    fn transform_signal_to_length_resamples_when_lengths_differ() {
        let signal: Vec<f64> = (0..6).map(|i| i as f64).collect();

        let out = transform_signal_to_length(&signal, 3, 9, TransformMethod::Pchip);

        assert_eq!(out.len(), 9);
        assert!(out.iter().all(|v| v.is_finite()));
    }

    #[test]
    fn transform_signal_to_length_handles_degenerate_requests() {
        assert!(transform_signal_to_length(&[1.0, 2.0], 2, 0, TransformMethod::Pchip).is_empty());
        assert!(transform_signal_to_length(&[], 2, 4, TransformMethod::Pchip).is_empty());
    }

    #[test]
    fn tv_denoise_flattens_a_single_outlier() {
        let mut signal = vec![0.0; 11];
        signal[5] = 1.0;

        let out = apply_method_same_len(&signal, 1, TransformMethod::TvDenoise);

        assert!(out[5] < 1.0, "the spike must be attenuated: {out:?}");
        assert!(out.iter().all(|v| v.is_finite()));
    }
}
