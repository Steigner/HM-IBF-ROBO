# HM-IBF-Robo

Robotics benchmark for GRAHF with heterogeneous migration between island dimensions,
configured by `dimensions_allowed` in [`../params_training.conf`](../params_training.conf)
(shipped as `6, 12, 18, 24, 30`, i.e. `1, 2, 3, 4, 5` joint waypoints of a 6-DOF arm).

## The problem

A solution is a flattened vector of joint angles, `6` per waypoint. The arm starts at its
home pose and moves linearly in joint space from waypoint to waypoint; each segment is
sampled 100 times. The objective is

```text
f(x) = 100 * max_i dist(target_i, trajectory) + length(trajectory)
```

Lower is better and the known optimum is `0`. Because the evaluator uses `len(x)` rather
than a fixed problem dimension, candidates from islands of different dimensions are
directly comparable — no projection is applied anywhere.

There are 40 instances: `nr_points in {3, 4, 5, 6}` times ten instance seeds. They are
checked into `instances/`; `preprocessing/prepare_instances.py` only regenerates them from
`Pos_pnts.mat`, which is not part of this repository. Unless `--source` is given, the file
is fetched automatically by `preprocessing/fetch_benchmark.py`, which shallow-clones
[JakubKudela89/Robotics-Benchmarking](https://github.com/JakubKudela89/Robotics-Benchmarking)
into `external/robotics-benchmark/`.

## Islands

`islands::island_builders` defines the node weight encoding of an island graph:

| Weight | Island |
| --- | --- |
| `0` | Differential evolution (`de`) |
| `1` | Evolution strategy (`es`) |
| `2` | Local search (`ls`) |
| `3` | Simulated annealing (`sa`) |
| `4` | Random search (`rs`) |
| `5` | Passive archive (`archive`) |

The constants `ISLAND_DE` … `ISLAND_ARCHIVE` name these weights so the encoding cannot
drift silently. The archive is a dependent island: a graph consisting only of archives is
infeasible.

## Heterogeneous migration

When a migrant crosses to an island of a different dimension it is resized by
`islands::transforms`:

1. every waypoint is projected onto the shortest Cartesian route through the instance's
   targets, giving a resolution-independent position in `[0, 1]`,
2. each joint is treated as a 1D signal, unwrapped so it carries no `2*PI` jumps,
3. the signal is resampled onto the target island's waypoint grid and smoothed by the
   IRACE-selected method (`Akima`, `ClampedCubic`, `CT_Spline`, `DouglasPeucker`, `PCHIP`,
   `TVDenoise`, `VSpline`),
4. the result is mapped back into the joint limits `[-2*PI, 2*PI]`.

## Output

Evaluation exports the run's **global best individual** verbatim: `results.json` holds the
exact `x` that produced the reported `best_value`, so `len(x)` is whatever dimension the
winning island used.

See [runbook.md](runbook.md) for the commands.
