//! Dimension-agnostic evaluator for the robotics trajectory benchmark.
//!
//! Solutions are evaluated directly at their working dimension — `evaluate_solution`
//! uses `solution.len()` so fitness is comparable across all island dimensions without
//! any intermediate transform. Evaluation export reads the final global best individual
//! directly, so the reported `x` is the exact vector that produced the reported `f(x)`.

use mahf::{problems::Evaluate, Individual, State};

use super::problem::RoboProblem;

#[derive(Clone, Debug, Default)]
pub struct RoboEvaluator;

impl Evaluate for RoboEvaluator {
    type Problem = RoboProblem;

    fn evaluate(
        &mut self,
        problem: &Self::Problem,
        _state: &mut State<Self::Problem>,
        individuals: &mut [Individual<Self::Problem>],
    ) {
        for individual in individuals {
            individual.evaluate_with(|solution: &Vec<f64>| {
                problem
                    .evaluate_solution(solution)
                    .try_into()
                    .unwrap_or_default()
            });
        }
    }
}
