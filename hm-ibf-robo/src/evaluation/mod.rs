//! Evaluation of a trained GRAHF island graph across a range of seeds.

use std::{
    collections::BTreeMap,
    fs::{self, File},
    io::{BufWriter, Write},
    path::Path,
    sync::Arc,
};

use eyre::{ensure, WrapErr};
use grahf::components::{island::MigrationTransformer, transform::SolutionTransformer};
use log::info;
use mahf::{
    conditions,
    lens::{common::BestObjectiveValueLens, ValueOf},
    logging::{extractor::EntryExtractor, log::Entry},
    prelude::*,
    problems::SingleObjectiveProblem,
    state::common as mcommon,
    ExecResult, Random, State,
};

use crate::{
    cli::EvaluateArgs,
    config::EvaluationParams,
    islands::TargetRouteTransformer,
    robo::{
        output::{
            select_global_best_solution, write_run_results, RunMetadata,
            OUTPUT_TRANSFORM_NOT_APPLIED,
        },
        RoboEvaluator, RoboProblem, JOINTS,
    },
};

pub mod elitist;
pub mod log_series;

pub use elitist::load_elitist;
pub use log_series::{parse_log_series, write_progress_csvs, ProgressSeries};

/// Logs the objective values of every island's current population.
#[derive(Clone)]
pub struct IslandObjectiveValuesEntry;

impl<P> EntryExtractor<P> for IslandObjectiveValuesEntry
where
    P: SingleObjectiveProblem + 'static,
{
    fn extract_entry(&self, _problem: &P, state: &State<P>) -> Entry {
        let mut values = Vec::new();

        if let Ok(states) = state.try_borrow::<grahf::components::island::IslandStates<P>>() {
            for island_state in states.iter() {
                if let Ok(populations) = island_state.try_borrow::<mcommon::Populations<P>>() {
                    if let Some(population) = populations.get_current() {
                        values.extend(
                            population
                                .iter()
                                .filter_map(|individual| individual.get_objective())
                                .map(|objective| objective.value()),
                        );
                    }
                }
            }
        }

        Entry {
            name: "ObjectiveValues",
            value: Box::new(values),
        }
    }
}

/// Evaluates the trained elitist on every instance for every requested seed.
///
/// One result folder is written per `(instance, seed)` pair and an aggregated summary CSV is
/// written afterwards, holding the mean and sample standard deviation of the per-seed means
/// grouped by the instance's number of target points.
///
/// # Arguments
///
/// * `args` - Seeds and output locations.
/// * `eval_params` - The evaluate stage's tuning parameters, loaded from
///   `params_evaluation.conf`.
/// * `dimensions_allowed` - Allowed island dimensions in decision variables; see
///   `crate::config::TrainingParams::dimensions_allowed`.
/// * `run_dir` - Directory holding the trained artefacts.
/// * `instances_dir` - Directory holding the instance JSON files.
/// * `max_evaluations` - Evaluation budget of a single run.
///
/// # Returns
///
/// Nothing; all results are written to disk.
///
/// # Errors
///
/// Returns an error if the artefacts or instances cannot be loaded, a run fails, the
/// exported best individual is inconsistent with the run's state, or a file cannot be
/// written.
pub fn run(
    args: &EvaluateArgs,
    eval_params: &EvaluationParams,
    dimensions_allowed: &[u32],
    run_dir: &Path,
    instances_dir: &Path,
    max_evaluations: u32,
) -> ExecResult<()> {
    fs::create_dir_all(&args.results_dir)
        .wrap_err_with(|| format!("failed to create {}", args.results_dir.display()))?;

    let (builder_graph, params) = load_elitist::<RoboProblem>(
        run_dir,
        &args.elitist,
        dimensions_allowed,
        eval_params.max_iterations,
        eval_params.max_population_size,
    )?;

    info!("Loading instances from {}", instances_dir.display());
    let instances = RoboProblem::load_instances(instances_dir, dimensions_allowed)?;
    info!("Loaded {} instances.", instances.len());
    info!("Allowed island dimensions: {dimensions_allowed:?}");

    let transformer: Arc<dyn SolutionTransformer<RoboProblem>> =
        Arc::new(TargetRouteTransformer::new());
    let builder = builder_graph.into_builder(conditions::LessThanN::evaluations(max_evaluations));
    let config = builder(params)?;

    // Per-seed means, grouped by the instance's number of target points.
    let mut seed_means_by_points: BTreeMap<usize, Vec<f64>> = BTreeMap::new();

    for seed in args.seeds() {
        info!("=== SEED {seed} ===");
        let mut values_by_points: BTreeMap<usize, Vec<f64>> = BTreeMap::new();

        for instance in &instances {
            let best_value = evaluate_instance(
                &config,
                instance,
                seed,
                &args.results_dir,
                transformer.clone(),
                eval_params.best_value_tolerance,
                dimensions_allowed,
            )?;
            values_by_points
                .entry(instance.instance.nr_points)
                .or_default()
                .push(best_value);
        }

        for (points, values) in values_by_points {
            let mean = values.iter().sum::<f64>() / values.len() as f64;
            seed_means_by_points.entry(points).or_default().push(mean);
        }
    }

    write_summary_csv(&args.summary_csv, &seed_means_by_points)?;
    info!("Wrote summary to {}", args.summary_csv.display());

    Ok(())
}

/// Runs the configured metaheuristic once and writes the run's artefacts.
///
/// # Arguments
///
/// * `config` - The metaheuristic built from the trained elitist.
/// * `instance` - The instance to solve.
/// * `seed` - The run's random seed.
/// * `results_dir` - Directory receiving the run folder.
/// * `transformer` - Resizes migrants between islands of different dimensions.
/// * `best_value_tolerance` - Tolerance when cross-checking the exported best value
///   against the run's state.
/// * `dimensions_allowed` - Allowed island dimensions in decision variables; see
///   `crate::config::TrainingParams::dimensions_allowed`.
///
/// # Returns
///
/// The best objective value reached by the run.
///
/// # Errors
///
/// Returns an error if the run fails, the exported best individual disagrees with the run's
/// state, its dimension is not an allowed island dimension, or an artefact cannot be written.
#[allow(clippy::too_many_arguments)]
fn evaluate_instance(
    config: &Configuration<RoboProblem>,
    instance: &RoboProblem,
    seed: u64,
    results_dir: &Path,
    transformer: Arc<dyn SolutionTransformer<RoboProblem>>,
    best_value_tolerance: f64,
    dimensions_allowed: &[u32],
) -> ExecResult<f64> {
    let instance_name = instance.name().to_string();
    info!("Starting run on {instance_name} (seed={seed})");

    let state = config.optimize_with(instance, |state| {
        state.insert_evaluator(RoboEvaluator);
        state.insert(Random::new(seed));
        state.insert(MigrationTransformer(transformer.clone()));
        state.configure_log(|cfg| {
            cfg.with(
                conditions::EveryN::iterations(1),
                BestObjectiveValueLens::entry(),
            );
            cfg.with(
                conditions::EveryN::iterations(1),
                ValueOf::<mcommon::Evaluations>::entry(),
            );
            cfg.with(
                conditions::EveryN::iterations(1),
                Box::new(IslandObjectiveValuesEntry),
            );
            Ok(())
        })?;
        Ok(())
    })?;

    let state_best_value = state
        .best_objective_value()
        .ok_or_else(|| eyre::eyre!("run on {instance_name} (seed={seed}) produced no best value"))?
        .value();

    let global_best = select_global_best_solution(&state).wrap_err_with(|| {
        format!("failed to select global best individual for {instance_name} (seed={seed})")
    })?;

    ensure!(
        (state_best_value - global_best.best_value).abs() <= best_value_tolerance,
        "state best value {state_best_value} does not match global best individual value {} \
         for {instance_name} (seed={seed})",
        global_best.best_value,
    );
    ensure!(
        dimensions_allowed.contains(&(global_best.solution_dim as u32)),
        "global best solution for {instance_name} (seed={seed}) has unsupported dimension {}; \
         allowed {:?}",
        global_best.solution_dim,
        dimensions_allowed,
    );

    let nr_changes = RoboProblem::solution_nr_changes(&global_best.solution).ok_or_else(|| {
        eyre::eyre!(
            "global best solution for {instance_name} (seed={seed}) has dim {}, \
             which is not a valid {JOINTS}-joint waypoint encoding",
            global_best.solution_dim,
        )
    })?;

    info!(
        "f(x) global_best={:.6} solution_dim={} nr_changes={nr_changes} instance_points={}",
        global_best.best_value, global_best.solution_dim, instance.instance.nr_points,
    );

    let metadata = RunMetadata {
        problem: &instance_name,
        nr_points: instance.instance.nr_points,
        instance_id: &format!("inst{:02}", instance.instance.source_seed),
        instance_seed: instance.instance.source_seed,
        solution_dim: global_best.solution_dim,
        nr_changes,
        output_transform_method: OUTPUT_TRANSFORM_NOT_APPLIED,
        solution: &global_best.solution,
    };

    let run_dir = write_run_results(
        results_dir,
        &metadata,
        "GRAHF",
        seed,
        u64::from(state.evaluations()),
        global_best.best_value,
    )?;

    // `Log::to_json` is the only public serialization path, so the log is round-tripped
    // through a file inside the run folder before the progress series is extracted.
    let log_path = run_dir.join("run.log");
    state.log().to_json(&log_path)?;
    let log_json: serde_json::Value = serde_json::from_str(
        &fs::read_to_string(&log_path)
            .wrap_err_with(|| format!("failed to read {}", log_path.display()))?,
    )
    .wrap_err_with(|| format!("failed to parse {}", log_path.display()))?;
    write_progress_csvs(&run_dir, &parse_log_series(&log_json)?)?;

    info!("Stored results in {}", run_dir.display());

    Ok(global_best.best_value)
}

/// Writes the aggregated evaluation summary.
///
/// # Arguments
///
/// * `path` - Destination CSV file.
/// * `seed_means_by_points` - Per-seed means grouped by the number of target points.
///
/// # Errors
///
/// Returns an error if the file cannot be created or written.
fn write_summary_csv(
    path: &Path,
    seed_means_by_points: &BTreeMap<usize, Vec<f64>>,
) -> ExecResult<()> {
    let file =
        File::create(path).wrap_err_with(|| format!("failed to create {}", path.display()))?;
    let mut writer = BufWriter::new(file);

    writeln!(writer, "Algorithm,P,NSeeds,Mean,Std")?;
    for (points, seed_means) in seed_means_by_points {
        let n = seed_means.len();
        let mean = seed_means.iter().sum::<f64>() / n as f64;
        // Sample standard deviation; a single seed carries no spread information.
        let std = if n >= 2 {
            let variance =
                seed_means.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / (n - 1) as f64;
            variance.sqrt()
        } else {
            0.0
        };
        writeln!(writer, "GRAHF,{points},{n},{mean:.6},{std:.6}")?;
    }
    writer.flush()?;

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_summary_holds_one_row_per_point_count() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("results_robo.csv");
        let means = BTreeMap::from([(3, vec![1.0, 3.0]), (4, vec![10.0])]);

        write_summary_csv(&path, &means).unwrap();

        let content = fs::read_to_string(&path).unwrap();
        let lines: Vec<_> = content.lines().collect();
        assert_eq!(lines[0], "Algorithm,P,NSeeds,Mean,Std");
        assert_eq!(lines[1], "GRAHF,3,2,2.000000,1.414214");
        assert_eq!(
            lines[2], "GRAHF,4,1,10.000000,0.000000",
            "n=1 has no spread"
        );
    }

    #[test]
    fn an_empty_summary_still_has_a_header() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("results_robo.csv");

        write_summary_csv(&path, &BTreeMap::new()).unwrap();

        assert_eq!(
            fs::read_to_string(&path).unwrap().trim(),
            "Algorithm,P,NSeeds,Mean,Std"
        );
    }

    #[test]
    fn writing_the_summary_to_a_missing_directory_reports_the_path() {
        let error =
            write_summary_csv(Path::new("no/such/dir/out.csv"), &BTreeMap::new()).unwrap_err();

        assert!(error.to_string().contains("no/such/dir/out.csv"), "{error}");
    }
}
