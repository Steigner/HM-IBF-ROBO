//! Phase unwrapping and bounding of joint-angle signals.
//!
//! Joint angles live in `[-2*PI, 2*PI]`. Interpolating them directly would produce
//! artificial sweeps whenever a signal wraps around, so signals are unwrapped into a
//! continuous representation before smoothing and mapped back afterwards.

/// The inclusive joint-angle bound in radians.
pub(super) const ANGLE_LIMIT: f64 = 2.0 * std::f64::consts::PI;

/// The period after which a joint angle repeats.
pub(super) const ANGLE_PERIOD: f64 = 2.0 * std::f64::consts::PI;

/// Unwraps an angle signal into a continuous representation without `2*PI` jumps.
///
/// # Arguments
///
/// * `signal` - The wrapped angle samples.
///
/// # Returns
///
/// The unwrapped signal of the same length; empty for an empty input.
pub(super) fn unwrap_angles(signal: &[f64]) -> Vec<f64> {
    if signal.is_empty() {
        return Vec::new();
    }

    let mut out = Vec::with_capacity(signal.len());
    let mut offset = 0.0;
    let mut previous = clamp_angle(signal[0]);
    out.push(previous);

    for &value in &signal[1..] {
        let mut adjusted = clamp_angle(value) + offset;
        while adjusted - previous > std::f64::consts::PI {
            offset -= ANGLE_PERIOD;
            adjusted -= ANGLE_PERIOD;
        }
        while adjusted - previous < -std::f64::consts::PI {
            offset += ANGLE_PERIOD;
            adjusted += ANGLE_PERIOD;
        }

        out.push(adjusted);
        previous = adjusted;
    }

    out
}

/// Maps an unwrapped angle signal back into `[-ANGLE_LIMIT, ANGLE_LIMIT]`.
///
/// Each sample is shifted by whole periods towards its predecessor, so the result stays as
/// continuous as the bounds allow.
///
/// # Arguments
///
/// * `signal` - The unwrapped angle samples.
///
/// # Returns
///
/// The bounded signal of the same length; empty for an empty input.
pub(super) fn bound_unwrapped_angle_signal(signal: &[f64]) -> Vec<f64> {
    if signal.is_empty() {
        return Vec::new();
    }

    let mut out = Vec::with_capacity(signal.len());
    let mut previous = clamp_angle(signal[0]);
    out.push(previous);

    for &value in &signal[1..] {
        let bounded = bound_angle_near(value, previous);
        out.push(bounded);
        previous = bounded;
    }

    out
}

/// Returns the in-bounds representation of `value` closest to `reference`.
///
/// # Arguments
///
/// * `value` - The unwrapped angle.
/// * `reference` - The previous bounded angle to stay close to.
///
/// # Returns
///
/// An angle within `[-ANGLE_LIMIT, ANGLE_LIMIT]`, or `0.0` for non-finite input.
fn bound_angle_near(value: f64, reference: f64) -> f64 {
    if !value.is_finite() {
        return 0.0;
    }

    let mut best = clamp_angle(value);
    let mut best_distance = (best - reference).abs();

    for shift in -2..=2 {
        let candidate = value + f64::from(shift) * ANGLE_PERIOD;
        if (-ANGLE_LIMIT..=ANGLE_LIMIT).contains(&candidate) {
            let distance = (candidate - reference).abs();
            if distance < best_distance {
                best = candidate;
                best_distance = distance;
            }
        }
    }

    best
}

/// Clamps an angle into `[-ANGLE_LIMIT, ANGLE_LIMIT]`, mapping non-finite values to zero.
///
/// # Arguments
///
/// * `value` - The angle to clamp.
///
/// # Returns
///
/// The clamped angle.
pub(super) fn clamp_angle(value: f64) -> f64 {
    if !value.is_finite() {
        return 0.0;
    }

    value.clamp(-ANGLE_LIMIT, ANGLE_LIMIT)
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use super::*;

    const EPS: f64 = 1e-9;

    #[test]
    fn clamp_angle_maps_non_finite_values_to_zero() {
        assert_eq!(clamp_angle(f64::NAN), 0.0);
        assert_eq!(clamp_angle(f64::INFINITY), 0.0);
        assert_eq!(clamp_angle(f64::NEG_INFINITY), 0.0);
    }

    #[test]
    fn clamp_angle_limits_to_two_full_turns() {
        assert_eq!(clamp_angle(10.0), ANGLE_LIMIT);
        assert_eq!(clamp_angle(-10.0), -ANGLE_LIMIT);
        assert_eq!(clamp_angle(1.0), 1.0);
    }

    #[test]
    fn unwrap_removes_the_jump_across_the_period_boundary() {
        // A step of 2*PI - 0.2 is really a step of -0.2 in the unwrapped domain.
        let signal = [-PI + 0.1, PI - 0.1];

        let unwrapped = unwrap_angles(&signal);

        assert!(
            (unwrapped[1] - unwrapped[0] + 0.2).abs() < EPS,
            "{unwrapped:?}"
        );
    }

    #[test]
    fn unwrap_leaves_a_smooth_signal_unchanged() {
        let signal = [0.0, 0.5, 1.0, 1.5];
        let unwrapped = unwrap_angles(&signal);

        for (a, b) in signal.iter().zip(&unwrapped) {
            assert!((a - b).abs() < EPS);
        }
    }

    #[test]
    fn unwrap_handles_the_empty_signal() {
        assert!(unwrap_angles(&[]).is_empty());
        assert!(bound_unwrapped_angle_signal(&[]).is_empty());
    }

    #[test]
    fn bounding_keeps_every_sample_inside_the_joint_limits() {
        let unwrapped = [0.0, 7.0, 14.0, -20.0];

        let bounded = bound_unwrapped_angle_signal(&unwrapped);

        assert_eq!(bounded.len(), unwrapped.len());
        for value in bounded {
            assert!((-ANGLE_LIMIT..=ANGLE_LIMIT).contains(&value), "{value}");
        }
    }

    #[test]
    fn bounding_prefers_the_representation_closest_to_the_predecessor() {
        // 2.5*PI is out of bounds; shifting by one period gives 0.5*PI, which is closer to
        // the predecessor than the other in-bounds representative -1.5*PI.
        let bounded = bound_unwrapped_angle_signal(&[0.0, 2.5 * PI]);

        assert!((bounded[1] - 0.5 * PI).abs() < EPS, "{bounded:?}");
    }

    #[test]
    fn bounding_replaces_non_finite_samples_with_zero() {
        let bounded = bound_unwrapped_angle_signal(&[1.0, f64::NAN]);

        assert_eq!(bounded[1], 0.0);
    }

    #[test]
    fn unwrapping_then_bounding_is_stable_for_in_range_signals() {
        let signal = [0.5, 1.0, -1.0, 2.0];

        let round_trip = bound_unwrapped_angle_signal(&unwrap_angles(&signal));

        for (a, b) in signal.iter().zip(&round_trip) {
            assert!((a - b).abs() < EPS, "{signal:?} -> {round_trip:?}");
        }
    }
}
