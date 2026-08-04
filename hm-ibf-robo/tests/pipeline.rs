//! Integration tests covering the robotics benchmark end to end.
//!
//! These drive the crate through its public API: load the shipped instances, evaluate
//! trajectories, migrate solutions between island dimensions and export a run.

use std::{fs, path::PathBuf};

use grahf::components::transform::{SolutionTransformer, TransformRequest};
use grahf_robo::{
    config::{TrainingParams, DEFAULT_TRAINING_PARAMS},
    islands::{TargetRouteTransformer, TransformMethod},
    robo::{
        output::{write_run_results, RunMetadata, OUTPUT_TRANSFORM_NOT_APPLIED},
        RoboProblem, GAMMA, JOINTS,
    },
};
use mahf::{
    problems::{LimitedVectorProblem, VectorProblem},
    Problem, Random,
};

/// Returns the directory holding the instance JSON files shipped with the crate.
fn instances_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("instances")
}

/// Returns the shipped `params_training.conf`'s allowed island dimensions.
fn dimensions_allowed() -> Vec<u32> {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(DEFAULT_TRAINING_PARAMS);
    TrainingParams::load(&path)
        .expect("the shipped params_training.conf must load")
        .dimensions_allowed
}

/// Loads every shipped instance.
fn instances() -> Vec<RoboProblem> {
    RoboProblem::load_instances(instances_dir(), &dimensions_allowed())
        .expect("the shipped instances must load")
}

/// Builds a deterministic solution of `changes` waypoints.
fn solution(changes: usize) -> Vec<f64> {
    (0..changes * JOINTS)
        .map(|i| ((i as f64) * 0.37).sin())
        .collect()
}

#[test]
fn every_shipped_instance_loads_and_is_self_consistent() {
    let instances = instances();

    assert_eq!(instances.len(), 40, "4 point counts times 10 seeds");
    for instance in &instances {
        let descriptor = &instance.instance;
        assert_eq!(
            descriptor.points.len(),
            descriptor.nr_points,
            "{} declares {} points but holds {}",
            descriptor.name,
            descriptor.nr_points,
            descriptor.points.len()
        );
        assert_eq!(descriptor.source_indices.len(), descriptor.nr_points);
        assert!(
            (3..=6).contains(&descriptor.nr_points),
            "{}: unexpected point count {}",
            descriptor.name,
            descriptor.nr_points
        );
        assert!(
            descriptor
                .points
                .iter()
                .all(|point| point.iter().all(|v| v.is_finite())),
            "{} has a non-finite target",
            descriptor.name
        );
        assert_eq!(instance.name(), descriptor.name);
    }
}

#[test]
fn the_summary_file_is_not_loaded_as_an_instance() {
    assert!(instances_dir().join("summary.json").exists());
    assert!(instances().iter().all(|i| i.name() != "summary"));
}

#[test]
fn the_objective_is_finite_and_non_negative_for_every_allowed_dimension() {
    for instance in instances() {
        for &dimension in &dimensions_allowed() {
            let value = instance.evaluate_solution(&solution(dimension as usize / JOINTS));

            assert!(
                value.is_finite() && value >= 0.0,
                "{} at D={dimension} produced {value}",
                instance.name()
            );
        }
    }
}

#[test]
fn a_motionless_trajectory_costs_exactly_the_target_miss() {
    // An all-zero solution keeps the arm at its home pose, so the path length is zero and
    // the objective reduces to `GAMMA` times the distance to the furthest target.
    let instance = &instances()[0];
    let home = RoboProblem::end_effector_position(&[0.0; JOINTS]);

    let expected_miss = instance
        .instance
        .points
        .iter()
        .map(|target| {
            let d: f64 = (0..3).map(|axis| (home[axis] - target[axis]).powi(2)).sum();
            d.sqrt()
        })
        .fold(0.0_f64, f64::max);

    let value = instance.evaluate_solution(&[0.0; 18]);

    assert!(
        expected_miss > 0.0,
        "the targets must not sit at the home pose"
    );
    assert!(
        (value - GAMMA * expected_miss).abs() < 1e-9,
        "expected {} but got {value}",
        GAMMA * expected_miss
    );
}

#[test]
fn the_declared_domain_covers_the_largest_island_dimension() {
    let instance = &instances()[0];

    assert_eq!(
        instance.dimension(),
        *dimensions_allowed().last().unwrap() as usize
    );
    assert_eq!(instance.domain().len(), instance.dimension());
}

#[test]
fn migration_between_any_two_allowed_dimensions_yields_a_valid_solution() {
    let instance = &instances()[0];
    let transformer = TargetRouteTransformer::new();
    let mut rng = Random::new(0);

    let dimensions_allowed = dimensions_allowed();
    for &source in &dimensions_allowed {
        for &target in &dimensions_allowed {
            for method in TransformMethod::all_names() {
                let input = solution(source as usize / JOINTS);
                let request = TransformRequest::new(source, target, method);

                let output = transformer.transform(instance, &input, request, &mut rng);

                assert_eq!(
                    output.len(),
                    target as usize,
                    "{method}: {source} -> {target}"
                );
                assert!(
                    RoboProblem::solution_nr_changes(&output).is_some(),
                    "{method}: {source} -> {target} is not a waypoint encoding"
                );
                assert!(
                    instance.evaluate_solution(&output).is_finite(),
                    "{method}: {source} -> {target} is not evaluable"
                );
            }
        }
    }
}

#[test]
fn migrated_solutions_stay_inside_the_joint_limits() {
    let instance = &instances()[0];
    let transformer = TargetRouteTransformer::new();
    let mut rng = Random::new(1);
    let bounds = instance.domain();
    let (low, high) = (bounds[0].start, bounds[0].end);

    for method in TransformMethod::all_names() {
        let output = transformer.transform(
            instance,
            &solution(3),
            TransformRequest::new(18, 54, method),
            &mut rng,
        );

        for value in output {
            assert!(
                (low..=high).contains(&value),
                "{method} produced {value} outside [{low}, {high}]"
            );
        }
    }
}

#[test]
fn exporting_a_run_produces_a_readable_payload() {
    let dir = tempfile::tempdir().unwrap();
    let instance = &instances()[0];
    let angles = solution(3);
    let best_value = instance.evaluate_solution(&angles);

    let metadata = RunMetadata {
        problem: instance.name(),
        nr_points: instance.instance.nr_points,
        instance_id: "inst01",
        instance_seed: instance.instance.source_seed,
        solution_dim: angles.len(),
        nr_changes: angles.len() / JOINTS,
        output_transform_method: OUTPUT_TRANSFORM_NOT_APPLIED,
        solution: &angles,
    };

    let run_dir =
        write_run_results(dir.path(), &metadata, "GRAHF", 42, 100_000, best_value).unwrap();

    let payload: serde_json::Value =
        serde_json::from_str(&fs::read_to_string(run_dir.join("results.json")).unwrap()).unwrap();

    // The exported `x` must be the vector that produced the exported `best_value`.
    //
    // Bit equality is not asserted: `serde_json`'s float parser can be one ULP off when
    // reading back a value its own writer emitted, so the round trip is checked against a
    // tolerance far below anything that affects the objective.
    let exported: Vec<f64> = payload["x"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_f64().unwrap())
        .collect();

    assert_eq!(exported.len(), angles.len());
    for (before, after) in angles.iter().zip(&exported) {
        assert!(
            (before - after).abs() <= f64::EPSILON * before.abs().max(1.0),
            "{before} round-tripped to {after}"
        );
    }

    let reported = payload["best_value"].as_f64().unwrap();
    assert!(
        (instance.evaluate_solution(&exported) - reported).abs() < 1e-9,
        "recomputing f(x) from the export must reproduce the reported value"
    );
    assert_eq!(
        payload["solution_dim"].as_u64().unwrap() as usize,
        angles.len()
    );
    assert_eq!(
        payload["nr_changes"].as_u64().unwrap() as usize,
        angles.len() / JOINTS
    );
}
