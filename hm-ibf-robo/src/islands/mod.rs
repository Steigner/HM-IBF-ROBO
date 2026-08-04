//! Island builder collection for the robotics benchmark.
//!
//! Each builder produces a MAHF island (DE, ES, LS, SA, RS, or Archive) wired to work at
//! any IRACE-tuned dimension via [`RandomSpreadWithDimension`] initialization.

#![allow(clippy::new_ret_no_self)]

use better_any::{Tid, TidAble};
use eyre::bail;
use grahf::problems::algorithm_design::builder::MetaheuristicIslandBuilder;
use mahf::{params::Params, population::IntoIndividuals, prelude::*};
use serde::{Deserialize, Serialize};

use crate::problems::{DimensionAwareDomain, RealValuedProblem};

pub mod archive;
pub mod de;
pub mod es;
pub mod ls;
pub mod rs;
pub mod sa;
pub mod safe_boundary;
pub mod safe_de;
pub mod safe_diversity;
pub mod transforms;

pub use transforms::{TargetRouteTransformer, TransformMethod};

/// Returns all available island builders for GRAHF to choose from.
///
/// The returned order defines the node weight encoding: node weight `i` selects builder `i`.
///
/// # Arguments
///
/// * `dimensions_allowed` - Allowed island dimensions in decision variables; see
///   [`crate::config::TrainingParams::dimensions_allowed`].
/// * `max_iterations` - Upper bound IRACE may assign to an island's iteration count.
/// * `max_population_size` - Upper bound IRACE may assign to an island's population size.
///
/// # Returns
///
/// The builders, in node weight order.
pub fn island_builders<P: RealValuedProblem + DimensionAwareDomain>(
    dimensions_allowed: &[u32],
    max_iterations: u32,
    max_population_size: u32,
) -> Vec<Box<dyn MetaheuristicIslandBuilder<P>>> {
    vec![
        de::Builder::new(dimensions_allowed, max_iterations, max_population_size),
        es::Builder::new(dimensions_allowed, max_iterations, max_population_size),
        ls::Builder::new(dimensions_allowed, max_iterations, max_population_size),
        sa::Builder::new(dimensions_allowed, max_iterations),
        rs::Builder::new(dimensions_allowed, max_iterations, max_population_size),
        archive::Builder::new(dimensions_allowed, max_population_size),
    ]
}

/// Node weight of the differential evolution island in [`island_builders`].
pub const ISLAND_DE: u32 = 0;
/// Node weight of the evolution strategy island in [`island_builders`].
pub const ISLAND_ES: u32 = 1;
/// Node weight of the local search island in [`island_builders`].
pub const ISLAND_LS: u32 = 2;
/// Node weight of the simulated annealing island in [`island_builders`].
pub const ISLAND_SA: u32 = 3;
/// Node weight of the random search island in [`island_builders`].
pub const ISLAND_RS: u32 = 4;
/// Node weight of the passive archive island in [`island_builders`].
pub const ISLAND_ARCHIVE: u32 = 5;

/// Builds the default initialization component of an island.
///
/// # Arguments
///
/// * `params` - The island's IRACE parameters; `population_size` is required and
///   `dimension` is optional.
///
/// # Returns
///
/// A component that spreads a population across the island's working dimension and
/// evaluates it.
///
/// # Errors
///
/// Returns an error if `population_size` is missing or has the wrong type.
pub fn default_initialization<P: RealValuedProblem + DimensionAwareDomain>(
    mut params: Params,
) -> ExecResult<Box<dyn Component<P>>> {
    let population_size = params.try_extract::<u32>("population_size")?;
    let dimension = params.try_extract::<u32>("dimension").ok();

    Ok(Configuration::builder()
        .do_(RandomSpreadWithDimension::new(population_size, dimension))
        .evaluate()
        .update_best_individual()
        .build_component())
}

/// Builds a mutation component by name.
///
/// # Arguments
///
/// * `name` - Either `"normal"` or `"uniform"`.
/// * `strength` - The mutation strength passed to the operator.
///
/// # Returns
///
/// The boxed mutation component.
///
/// # Errors
///
/// Returns an error if `name` is not a known mutation method.
pub fn make_mutation<P: RealValuedProblem>(
    name: &str,
    strength: f64,
) -> ExecResult<Box<dyn Component<P>>> {
    let mutation = match name {
        "normal" => mutation::NormalMutation::new(strength, 1.0),
        "uniform" => mutation::UniformMutation::new(strength, 1.0),
        _ => bail!("invalid mutation method: {name}"),
    };
    Ok(mutation)
}

/// Resets the iteration counter so an island's inner loop starts from zero.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ResetIterations;

impl ResetIterations {
    /// Creates a new `ResetIterations`.
    pub fn from_params() -> Self {
        Self
    }

    /// Wraps `ResetIterations` in a boxed component.
    pub fn new<P: Problem>() -> Box<dyn Component<P>> {
        Box::new(Self::from_params())
    }
}

impl<P: Problem> Component<P> for ResetIterations {
    fn execute(&self, _problem: &P, state: &mut State<P>) -> ExecResult<()> {
        state.set_value::<common::Iterations>(0);
        Ok(())
    }
}

/// Initializes a population spread uniformly across the domain at a specific dimension.
///
/// When `dimension` is `None`, falls back to `problem.dimension()`. When set, each solution
/// has exactly that many elements regardless of the problem's declared dimension.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RandomSpreadWithDimension {
    /// Number of individuals in the initial population.
    pub population_size: u32,
    /// Working dimension for this island; `None` means use `problem.dimension()`.
    pub dimension: Option<u32>,
}

impl RandomSpreadWithDimension {
    /// Creates a new `RandomSpreadWithDimension`.
    ///
    /// # Arguments
    ///
    /// * `population_size` - Number of individuals to sample.
    /// * `dimension` - The island's working dimension, or `None` for the problem default.
    ///
    /// # Returns
    ///
    /// The component.
    pub fn from_params(population_size: u32, dimension: Option<u32>) -> Self {
        Self {
            population_size,
            dimension,
        }
    }

    /// Wraps `RandomSpreadWithDimension` in a boxed component.
    ///
    /// # Arguments
    ///
    /// * `population_size` - Number of individuals to sample.
    /// * `dimension` - The island's working dimension, or `None` for the problem default.
    ///
    /// # Returns
    ///
    /// The boxed component.
    pub fn new<P: RealValuedProblem + DimensionAwareDomain>(
        population_size: u32,
        dimension: Option<u32>,
    ) -> Box<dyn Component<P>> {
        Box::new(Self::from_params(population_size, dimension))
    }
}

impl<P: RealValuedProblem + DimensionAwareDomain> Component<P> for RandomSpreadWithDimension {
    fn init(&self, problem: &P, state: &mut State<P>) -> ExecResult<()> {
        let dim = self
            .dimension
            .map_or_else(|| problem.dimension(), |d| d as usize);

        state.insert(IslandDimension(dim));

        // `domain_for_dimension` gives each island the correct number of joint-angle bounds
        // for its working dimension; `domain()` would always return the maximum layout.
        let domain = problem.domain_for_dimension(dim);

        let mut rng = state.random_mut();
        let population: Vec<_> = (0..self.population_size)
            .map(|_| {
                domain
                    .iter()
                    .map(|range| {
                        use mahf::rand::Rng;
                        rng.gen_range(range.clone())
                    })
                    .collect::<Vec<f64>>()
            })
            .into_individuals();
        drop(rng);

        state.populations_mut().push(population);
        Ok(())
    }

    fn execute(&self, _problem: &P, _state: &mut State<P>) -> ExecResult<()> {
        Ok(())
    }
}

/// The actual working dimension of an island, stored in the island's state.
///
/// May differ from `problem.dimension()` when IRACE tunes per-island dimensions. Safe
/// components read this instead of calling `problem.dimension()`.
#[derive(Clone, Copy, Debug, Serialize, Deserialize, Tid)]
pub struct IslandDimension(pub usize);

impl mahf::CustomState<'_> for IslandDimension {}

impl std::ops::Deref for IslandDimension {
    type Target = usize;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Allowed island dimensions used by tests in this module.
    ///
    /// The real invariants (non-empty, strictly increasing, multiples of `JOINTS`) are
    /// enforced at load time by `config::validate_dimensions_allowed` and checked against
    /// the shipped `params_training.conf` there; these tests only need *some* valid list.
    const TEST_DIMENSIONS: [u32; 5] = [6, 12, 18, 24, 30];

    #[test]
    fn the_island_weight_constants_match_the_builder_order() {
        let builders = island_builders::<crate::robo::RoboProblem>(&TEST_DIMENSIONS, 10, 10);
        let ids: Vec<String> = builders.iter().map(|builder| builder.id()).collect();

        assert_eq!(ids[ISLAND_DE as usize], "de");
        assert_eq!(ids[ISLAND_ES as usize], "es");
        assert_eq!(ids[ISLAND_LS as usize], "ls");
        assert_eq!(ids[ISLAND_SA as usize], "sa");
        assert_eq!(ids[ISLAND_RS as usize], "rs");
        assert_eq!(ids[ISLAND_ARCHIVE as usize], "ar");
        assert_eq!(ids.len(), 6, "the encoding must cover every builder");
    }

    #[test]
    fn make_mutation_accepts_the_documented_names() {
        for name in ["normal", "uniform"] {
            assert!(
                make_mutation::<crate::robo::RoboProblem>(name, 0.1).is_ok(),
                "{name} must be a known mutation"
            );
        }
    }

    #[test]
    fn make_mutation_rejects_unknown_names() {
        let result = make_mutation::<crate::robo::RoboProblem>("cauchy", 0.1);

        match result {
            Ok(_) => panic!("an unknown mutation method must be rejected"),
            Err(error) => assert!(error.to_string().contains("cauchy"), "{error}"),
        }
    }

    #[test]
    fn island_dimension_derefs_to_its_value() {
        assert_eq!(*IslandDimension(24), 24);
    }
}
