//! Binary entry point of the HM-IBF-ROBO experiment pipeline.
//!
//! See `hm-ibf-robo --help` for the available subcommands and flags.

use std::sync::Arc;

use clap::Parser;
use eyre::WrapErr;
use grahf::components::transform::SolutionTransformer;
use grahf_robo::{
    cli::{Cli, Command, TrainArgs},
    config::{EvaluationParams, TrainingParams},
    evaluation,
    islands::TargetRouteTransformer,
    robo::{RoboEvaluator, RoboProblem},
    training,
};
use log::info;

fn main() -> eyre::Result<()> {
    color_eyre::install()?;
    pretty_env_logger::init_timed();

    let cli = Cli::parse();
    let jobs = cli.effective_jobs();

    // The global pool is process-wide, so it must be sized before any parallel work starts.
    rayon::ThreadPoolBuilder::new()
        .num_threads(jobs)
        .build_global()
        .wrap_err("failed to configure the rayon thread pool")?;
    info!("Using {jobs} worker threads.");

    match &cli.command {
        Command::Train(args) => {
            let params = TrainingParams::load(&cli.training_params)?;
            train(args, &params, &cli, jobs)
        }
        Command::Evaluate(args) => {
            let training_params = TrainingParams::load(&cli.training_params)?;
            let evaluation_params = EvaluationParams::load(&cli.evaluation_params)?;
            evaluation::run(
                args,
                &evaluation_params,
                &training_params.dimensions_allowed,
                &cli.run_dir,
                &cli.instances_dir,
                training_params.max_evaluations,
            )
        }
        Command::Pipeline {
            train: train_args,
            evaluate,
        } => {
            let training_params = TrainingParams::load(&cli.training_params)?;
            let evaluation_params = EvaluationParams::load(&cli.evaluation_params)?;
            train(train_args, &training_params, &cli, jobs)?;
            evaluation::run(
                evaluate,
                &evaluation_params,
                &training_params.dimensions_allowed,
                &cli.run_dir,
                &cli.instances_dir,
                training_params.max_evaluations,
            )
        }
    }
}

/// Runs the GRAHF structure search and writes its summary.
///
/// # Arguments
///
/// * `args` - The training seed and output directory.
/// * `params` - The GRAHF search's tuning parameters, loaded from `params_training.conf`.
/// * `cli` - The global options, supplying the directories.
/// * `jobs` - Total number of worker threads available.
///
/// # Errors
///
/// Returns an error if the instances cannot be loaded or the search fails.
fn train(args: &TrainArgs, params: &TrainingParams, cli: &Cli, jobs: usize) -> eyre::Result<()> {
    let instances_dir = &cli.instances_dir;
    info!("Loading instances from {}", instances_dir.display());
    let instances = RoboProblem::load_instances(instances_dir, &params.dimensions_allowed)?;
    info!("Loaded {} instances.", instances.len());
    info!("Allowed island dimensions: {:?}", params.dimensions_allowed);

    let evaluators = vec![RoboEvaluator; instances.len()];
    let transformer: Arc<dyn SolutionTransformer<RoboProblem>> =
        Arc::new(TargetRouteTransformer::new());

    let (problem, mut state) = training::run(
        args,
        params,
        instances,
        evaluators,
        &cli.run_dir,
        transformer,
        jobs,
    )?;
    training::summarize(&problem, &mut state, &cli.run_dir)?;

    info!("Finished training successfully.");
    Ok(())
}
