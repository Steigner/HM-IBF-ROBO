"""Catalog of every place a retarget to a new optimization problem has to touch.

The pipeline is already generic over the problem type: `training::run`, `island_builders`,
`load_elitist` and every island operator are bound only by
`P: RealValuedProblem + DimensionAwareDomain`. Retargeting is therefore not a rewrite but a
finite, enumerable set of edits, listed here as pure data.

Three layers, ordered by how much judgement each needs:

* ``rewrite`` - the domain semantics. Nothing here transfers to another problem.
* ``bind`` - mechanical, but skipping one entry produces a run that fails late or lies.
* ``keep`` - already generic. Editing these is how a retarget stops being an HM-IBF.

:mod:`retarget` anchors each entry to a live line; extend this catalog, not the resolver.
"""

from __future__ import annotations

from model import Layer, Residue, Site

LAYERS: tuple[Layer, ...] = (
    Layer(
        key="rewrite",
        title="REWRITE - the domain lives here",
        note=(
            "Each entry encodes an assumption about 6-DOF manipulators. Replace the body, keep "
            "the signature's contract. The five that decide whether migration still means "
            "anything are the block size, the objective, the decoder, the backbone and the "
            "variable topology."
        ),
    ),
    Layer(
        key="bind",
        title="BIND - mechanical, but load-bearing",
        note=(
            "Concrete type bindings, validation and the export payload. No design decisions "
            "here, yet skipping one produces a run that fails on the last instance or writes a "
            "payload that no longer describes what was optimized."
        ),
    ),
    Layer(
        key="keep",
        title="KEEP - already generic, do not touch",
        note=(
            "These are bound by `P: RealValuedProblem + DimensionAwareDomain` and never mention "
            "robotics. Editing them to 'make the new domain fit' is the failure mode: it "
            "specialises the framework instead of the benchmark."
        ),
    ),
)

SITES: tuple[Site, ...] = (
    # ---------------------------------------------------------------- rewrite
    Site(
        key="block_size",
        layer="rewrite",
        path="hm-ibf-robo/src/robo/problem.rs",
        anchor=r"^pub const JOINTS: usize",
        title="Block size B - decision variables per indivisible unit",
        change=(
            "Replace with the new domain's block size: the group of variables that describes "
            "one repeatable element (a waypoint here; a residue, a reactor stage, a time step "
            "elsewhere). B = 1 is legal and means every variable stands alone."
        ),
        contract=(
            "A solution of length D encodes D/B blocks. Every allowed dimension must stay a "
            "positive multiple of B, and the flattening must stay block-major: block i owns "
            "indices [i*B, (i+1)*B)."
        ),
    ),
    Site(
        key="instance_schema",
        layer="rewrite",
        path="hm-ibf-robo/src/robo/problem.rs",
        anchor=r"^pub struct RoboInstance",
        title="Instance descriptor - what a benchmark case is",
        change=(
            "Replace the Cartesian target list with the new domain's case data. Keep `name` "
            "(it labels run folders and the summary) and keep whatever field the backbone "
            "needs to build its parameterisation."
        ),
        contract="Stays `serde`-round-trippable: instances are read from JSON on disk.",
    ),
    Site(
        key="objective",
        layer="rewrite",
        path="hm-ibf-robo/src/robo/problem.rs",
        anchor=r"pub fn evaluate_solution",
        title="Objective function f(x)",
        change=(
            "Replace the `GAMMA * max_target_miss + path_length` body with the new objective. "
            "Return `f64::INFINITY` for an encoding that is not a whole number of blocks."
        ),
        contract=(
            "THE load-bearing invariant of the whole framework: read `solution.len()`, never "
            "`problem.dimension()`. A 12-variable and a 30-variable solution are scored on the "
            "same scale, otherwise the outer search just selects for whichever dimension the "
            "objective happens to flatter, and heterogeneous migration becomes noise."
        ),
    ),
    Site(
        key="decoder",
        layer="rewrite",
        path="hm-ibf-robo/src/robo/problem.rs",
        anchor=r"^fn forward_kinematics",
        title="Decoder - decision variables to the observable space",
        change=(
            "Replace forward kinematics with the map from one block to the space the backbone "
            "is parameterised in (a concentration profile, a sequence-space embedding, a "
            "process state). Delete `dh_transform`, `matrix_mul` and `identity_matrix` with it."
        ),
        contract=(
            "Pure and deterministic: it is called once per interpolation step per evaluation "
            "and again for every migration, so it must be cheap and side-effect free."
        ),
    ),
    Site(
        key="bounds",
        layer="rewrite",
        path="hm-ibf-robo/src/robo/problem.rs",
        anchor=r"fn domain\(&self\)",
        title="Search-space bounds at the maximum dimension",
        change=(
            "Replace the uniform `[-2*PI, 2*PI]` range with the new domain's per-variable "
            "bounds, repeated block-wise up to the maximum allowed dimension."
        ),
        contract="Length must equal `self.dimension()`; islands slice it, never extend it.",
    ),
    Site(
        key="max_dimension",
        layer="rewrite",
        path="hm-ibf-robo/src/robo/problem.rs",
        anchor=r"fn dimension\(&self\)",
        title="Declared dimension - the maximum, not the working one",
        change=(
            "Usually unchanged: it returns the largest allowed island dimension. Read the doc "
            "comment before touching it - returning anything else silently breaks every island "
            "that runs below the maximum."
        ),
        contract=(
            "Islands read their own working dimension from `IslandDimension` in state, set by "
            "`RandomSpreadWithDimension::init`. This method is the ceiling, not the runtime "
            "size."
        ),
    ),
    Site(
        key="dimension_aware_domain",
        layer="rewrite",
        path="hm-ibf-robo/src/robo/problem.rs",
        anchor=r"^impl DimensionAwareDomain for",
        title="Per-dimension bounds",
        change=(
            "The empty impl takes the default body, which cycles `domain()` modulo its length. "
            "That is correct only when the bounds repeat with period B. Override "
            "`domain_for_dimension` if a variable's range depends on where in the solution it "
            "sits (a ramp, a monotone schedule, a resolution-dependent budget)."
        ),
        contract=(
            "Must return exactly `dim` ranges. Getting this wrong does not crash - it samples "
            "initial populations outside the feasible set and the error only shows up as bad "
            "fitness."
        ),
    ),
    Site(
        key="block_count",
        layer="rewrite",
        path="hm-ibf-robo/src/robo/problem.rs",
        anchor=r"pub fn solution_nr_changes",
        title="Block-count decoder and encoding validity check",
        change=(
            "Rename to the new domain's vocabulary and divide by the new block size. This is "
            "the single definition of 'is this vector a valid encoding', used by the "
            "objective, the export and the evaluation gate."
        ),
        contract=(
            "Returns `None` for an empty vector and for any length that is not a multiple of B."
        ),
    ),
    Site(
        key="penalty_weight",
        layer="rewrite",
        path="hm-ibf-robo/src/robo/problem.rs",
        anchor=r"^pub const GAMMA",
        title="Constraint-violation weight of the scalarised objective",
        change=(
            "Replace with the new domain's penalty weight, or delete it if the objective is "
            "unconstrained. Document the scale it puts the two terms on."
        ),
        contract=(
            "The framework is single-objective (`SingleObjective`). A genuinely multi-objective "
            "domain must be scalarised here, deliberately - there is no Pareto path through "
            "`AlgorithmDesignProblem`."
        ),
    ),
    Site(
        key="discretisation",
        layer="rewrite",
        path="hm-ibf-robo/src/robo/problem.rs",
        anchor=r"^pub const TARGET_POINTS_PER_SEGMENT",
        title="Per-block discretisation of the evaluation",
        change=(
            "Replace with the new domain's within-block resolution, or delete it if blocks are "
            "evaluated atomically."
        ),
        contract=(
            "Must be constant across dimensions. Making it depend on the block count gives "
            "high-dimension solutions a finer (or coarser) objective and destroys "
            "cross-dimension comparability."
        ),
    ),
    Site(
        key="backbone",
        layer="rewrite",
        path="hm-ibf-robo/src/islands/transforms/backbone.rs",
        anchor=r"fn from_problem",
        title="Backbone - the resolution-independent parameter t in [0, 1]",
        change=(
            "Replace the shortest Cartesian route through the target points with the new "
            "domain's shared axis, and `project_t` with the projection onto it. Return `None` "
            "when the axis is degenerate so the caller falls back to the uniform transform."
        ),
        contract=(
            "This is what makes the framework HM-IBF rather than an island model. It answers "
            "'how far along is this block' in a way that does not depend on how many blocks "
            "the island uses. Without a real one, tau degrades to index-to-index resampling "
            "and migration between different dimensions is meaningless. If the new domain has "
            "no such axis - variables are unordered, exchangeable, or purely categorical - say "
            "so and stop; the benchmark is not a fit."
        ),
    ),
    Site(
        key="variable_topology",
        layer="rewrite",
        path="hm-ibf-robo/src/islands/transforms/angles.rs",
        anchor=r"fn unwrap_angles",
        title="Variable topology - periodic vs. bounded-linear",
        change=(
            "Joint angles are periodic, so the module unwraps 2*PI jumps before interpolating. "
            "For a non-periodic domain - concentrations, temperatures, pH, rates, masses - this "
            "is WRONG: it shifts values by whole periods. Replace both functions with the "
            "identity plus a clamp, or with the topology the variables actually have."
        ),
        contract=(
            "The single most dangerous site in a retarget. Leaving the angle code in place "
            "does not fail loudly: it silently corrupts every migrant, and the run still "
            "produces plausible-looking numbers."
        ),
    ),
    Site(
        key="topology_bound",
        layer="rewrite",
        path="hm-ibf-robo/src/islands/transforms/angles.rs",
        anchor=r"fn bound_unwrapped_angle_signal",
        title="Post-transform bounding back into the feasible set",
        change=(
            "Replace the shift-by-whole-periods logic with the new domain's feasibility "
            "restoration: a clamp, a projection, or a renormalisation for sum-constrained "
            "variables."
        ),
        contract=(
            "Runs on every transformed block in both transform paths. A transform can leave "
            "the feasible set (splines overshoot); this is the only thing that puts it back."
        ),
    ),
    Site(
        key="tau",
        layer="rewrite",
        path="hm-ibf-robo/src/islands/transforms/mod.rs",
        anchor=r"pub fn transform_along_target_route",
        title="tau - the migration transform along the backbone",
        change=(
            "Rename to the new domain, and rewrite the per-block loop: the anchor sample at "
            "t = 0 currently comes from `problem.initial_angles`. Supply the new domain's "
            "equivalent starting state, or drop the anchor if the encoding has none."
        ),
        contract=(
            "MUST return exactly `target_dim` elements for every input the islands can produce. "
            "The caller inserts the result into the target island's population without "
            "re-checking the length."
        ),
    ),
    Site(
        key="tau_fallback",
        layer="rewrite",
        path="hm-ibf-robo/src/islands/transforms/mod.rs",
        anchor=r"pub fn transform_uniformly",
        title="tau fallback - uniform resampling when the backbone is degenerate",
        change=(
            "Keep the structure, swap the angle handling for the new topology. It is reached "
            "whenever the backbone cannot be built, so it must stay correct, not just present."
        ),
        contract="Same length guarantee as the primary transform.",
    ),
    Site(
        key="instance_generator",
        layer="rewrite",
        path="hm-ibf-robo/preprocessing/prepare_instances.py",
        anchor=r"^def build_instance",
        title="Instance generator",
        change=(
            "Replace the `Pos_pnts.mat` sampling with the new domain's case generation, and "
            "`fetch_benchmark.py` with wherever the source data comes from. Emit the JSON "
            "schema the new instance descriptor deserialises."
        ),
        contract=(
            "Stays a standalone module with its own dependencies - it is not part of the "
            "binary. Keep the deterministic per-instance seed so instances are reproducible."
        ),
    ),
    # ------------------------------------------------------------------- bind
    Site(
        key="dimension_validation",
        layer="bind",
        path="hm-ibf-robo/src/config.rs",
        anchor=r"^fn validate_dimensions_allowed",
        title="dimensions_allowed validation",
        change="Point the multiple-of check at the new block size constant.",
        contract=(
            "Non-empty, strictly increasing, positive multiples of B. This is the only place "
            "that catches a mis-specified dimension set before it becomes a mid-run panic."
        ),
    ),
    Site(
        key="dimensions_conf",
        layer="bind",
        path="params_training.conf",
        anchor=r"^dimensions_allowed",
        title="The allowed island dimensions themselves",
        change=(
            "Set to the new domain's block multiples. Pick a range wide enough that migration "
            "between different dimensions is actually exercised - a single entry turns the "
            "framework back into a homogeneous island model."
        ),
        contract=(
            "Changing this invalidates every stored `elitist_*.json`: a trained elitist's "
            "IRACE-tuned `dimension` categorical may no longer be in the set. Retrain."
        ),
    ),
    Site(
        key="train_binding",
        layer="bind",
        path="hm-ibf-robo/src/main.rs",
        anchor=r"RoboProblem::load_instances",
        title="Train stage type binding",
        change=(
            "Swap the concrete problem, evaluator and transformer. `training::run` itself is "
            "generic and needs no change."
        ),
    ),
    Site(
        key="eval_elitist_binding",
        layer="bind",
        path="hm-ibf-robo/src/evaluation/mod.rs",
        anchor=r"load_elitist::<RoboProblem>",
        title="Evaluate stage type binding",
        change="Swap the concrete problem type; `load_elitist` is generic.",
    ),
    Site(
        key="eval_dimension_gate",
        layer="bind",
        path="hm-ibf-robo/src/evaluation/mod.rs",
        anchor=r"dimensions_allowed\.contains",
        title="Exported-dimension gate",
        change=(
            "Keep it. It asserts the exported best individual's length is an allowed island "
            "dimension - the end-to-end proof that the transform chain preserved lengths."
        ),
        contract=(
            "Do not relax this to a warning. It is the check that catches a tau which returned "
            "the wrong length."
        ),
    ),
    Site(
        key="eval_grouping",
        layer="bind",
        path="hm-ibf-robo/src/evaluation/mod.rs",
        anchor=r"\.entry\(instance\.instance\.nr_points\)",
        title="Result grouping axis of the summary",
        change=(
            "Replace `nr_points` with the new domain's instance-size axis - the property that "
            "makes one case harder than another. It becomes column `P` of the summary CSV."
        ),
    ),
    Site(
        key="export_payload",
        layer="bind",
        path="hm-ibf-robo/src/robo/output.rs",
        anchor=r"pub struct RunMetadata",
        title="results.json payload",
        change=(
            "Replace `nr_points` and `nr_changes` with the new domain's descriptors. Keep "
            "`solution_dim`, `x`, `best_value` and `n_evals` - downstream analysis reads them."
        ),
    ),
    Site(
        key="export_schema",
        layer="bind",
        path="hm-ibf-robo/src/robo/output.rs",
        anchor=r"^pub const OUTPUT_EXPORT_SCHEMA_VERSION",
        title="Export schema version",
        change="Bump it. Any change to the payload above changes the schema.",
    ),
    Site(
        key="export_validation",
        layer="bind",
        path="hm-ibf-robo/src/robo/output.rs",
        anchor=r"pub fn select_global_best_solution",
        title="Global-best extraction and its validity checks",
        change=(
            "Point the encoding check at the new block-count decoder. Keep the finite-objective "
            "check."
        ),
        contract=(
            "The exported `x` is the verbatim global best individual - no output projection is "
            "applied. If the new domain needs one, that is a design change, not a retarget: "
            "`OUTPUT_TRANSFORM_NOT_APPLIED` and the export doc comments claim otherwise."
        ),
    ),
    Site(
        key="cli_defaults",
        layer="bind",
        path="hm-ibf-robo/src/cli.rs",
        anchor=r"^pub const DEFAULT_SUMMARY_CSV",
        title="CLI defaults and naming",
        change=(
            "Rename the run directory, summary file and binary naming to the new domain. "
            "Cosmetic, but these strings appear in every runbook command."
        ),
    ),
    # ------------------------------------------------------------------- keep
    Site(
        key="keep_training",
        layer="keep",
        path="hm-ibf-robo/src/training.rs",
        anchor=r"^pub fn run<P, O>",
        title="GRAHF structure search",
        change="No change. Already generic over the problem and its evaluator.",
    ),
    Site(
        key="keep_island_builders",
        layer="keep",
        path="hm-ibf-robo/src/islands/mod.rs",
        anchor=r"^pub fn island_builders<P",
        title="Island builder set and the node-weight encoding",
        change=(
            "No change. Adding, removing or reordering a builder shifts the node weight "
            "encoding and invalidates every stored elitist - that is an island-set change, a "
            "separate task from retargeting the problem."
        ),
    ),
    Site(
        key="keep_dimension_trait",
        layer="keep",
        path="hm-ibf-robo/src/problems/mod.rs",
        anchor=r"^pub trait DimensionAwareDomain",
        title="The dimension-aware domain trait",
        change=(
            "No change. Implement it for the new problem instead; read the coherence note "
            "before considering a blanket impl."
        ),
    ),
    Site(
        key="keep_migrations",
        layer="keep",
        path="hm-ibf-robo/src/migrations/mod.rs",
        anchor=r"pub fn migration_builders",
        title="Migration policy set and the edge-weight encoding",
        change="No change. Domain-independent: selection, condition and replacement policies.",
    ),
    Site(
        key="keep_safe_operators",
        layer="keep",
        path="hm-ibf-robo/src/islands/safe_de.rs",
        anchor=r"pub struct SafeDEBinomialCrossover",
        title="Dimension-safe operators",
        change=(
            "No change. These exist precisely because they read `solution.len()` instead of "
            "`problem.dimension()`. A retarget that reintroduces a global dimension read here "
            "breaks mixed-dimension populations."
        ),
    ),
)

RESIDUES: tuple[Residue, ...] = (
    Residue(
        key="periodic_angle_topology",
        pattern=r"unwrap_angles|bound_unwrapped_angle_signal|ANGLE_PERIOD|ANGLE_LIMIT",
        severity="high",
        message=(
            "The 2*PI phase-unwrap topology is still live. Unless the new variables really are "
            "periodic, every migrant is being shifted by whole periods before it is inserted - "
            "silently, with plausible-looking results. See the `variable_topology` site."
        ),
    ),
    Residue(
        key="forward_kinematics",
        pattern=r"forward_kinematics|dh_transform|end_effector|initial_angles",
        severity="high",
        message=(
            "The manipulator decoder is still referenced. The backbone and the objective are "
            "still reading a robot's Cartesian pose."
        ),
    ),
    Residue(
        key="joint_block_size",
        pattern=r"\bJOINTS\b",
        severity="high",
        message=(
            "The block size is still named and sized for joints. Every dimension validity "
            "check, the export and the transform split on this constant."
        ),
    ),
    Residue(
        key="waypoint_vocabulary",
        pattern=r"nr_changes|nr_points|waypoint",
        severity="note",
        message=(
            "Waypoint vocabulary survives in the export payload, the summary grouping or the "
            "docs. Harmless at runtime, but the results no longer describe what was optimized."
        ),
    ),
    Residue(
        key="robo_naming",
        pattern=r"RoboProblem|RoboInstance|RoboEvaluator|robo_run|results_robo",
        severity="note",
        message=(
            "Robotics naming survives in types, defaults or paths. Cosmetic - but check "
            "`params_training.conf`, the runbook and `run.sh` for stale command lines."
        ),
    ),
    Residue(
        key="global_dimension_read",
        pattern=r"problem\.dimension\(\)",
        severity="note",
        message=(
            "A global dimension read. Legitimate in `VectorProblem::dimension` itself and in "
            "the documented island fallbacks; anywhere else it breaks islands running below "
            "the maximum. Read each hit before acting."
        ),
    ),
    Residue(
        key="unbumped_export_schema",
        pattern=r"OUTPUT_EXPORT_SCHEMA_VERSION: u32 = 4",
        severity="note",
        message=(
            "The export schema is still at the robotics version 4 while the payload changed. "
            "Bump it so downstream readers can tell the formats apart."
        ),
    ),
)
