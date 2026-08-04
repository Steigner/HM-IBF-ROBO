# HM-IBF-Robo — Runbook

Pipeline: **(preprocess) → train → evaluate**.

Every command runs inside the container. Start it once from the repository root, on the
**host** (`run.sh`/`run.bat` talk to the Docker daemon, so don't call them again once
you're already inside):

```bash
./run.sh                 # opens a shell in the container at /app
```

On Windows, `.\run.bat` is the same thing (see [../README.md](../README.md) for the
PowerShell/cmd env var syntax needed by the `HM_IBF_NIX` switch below).

From there, the pipeline is the `hm-ibf` command installed on `PATH` — it builds the
release binary if needed and always runs it from `hm-ibf-robo/`, so its relative defaults
(`instances/`, `robo_run/`, `results/`) resolve correctly no matter which directory you
call it from:

```bash
hm-ibf --help
```

`./run.sh <command>` runs a single command in the container instead of opening a shell,
e.g. `./run.sh hm-ibf --help`.

## Optional step 0: regenerate instances

The 40 instance JSONs are checked in, so this is only needed after changing the selection
rule. The source point cloud, `Pos_pnts.mat`, is not part of this repository; by default it
is fetched automatically by shallow-cloning
[JakubKudela89/Robotics-Benchmarking](https://github.com/JakubKudela89/Robotics-Benchmarking)
into `--repo-dir` (default `external/robotics-benchmark/`, gitignored). This is a plain
Python module, not wrapped by `hm-ibf`, so run it from `hm-ibf-robo/`:

```bash
cd hm-ibf-robo
python3 -m preprocessing.prepare_instances --output-dir instances
```

Pass `--source` to use an already-downloaded `.mat` file instead (no network access, no
clone):

```bash
python3 -m preprocessing.prepare_instances \
    --source ../robo-evo-apps/Pos_pnts.mat \
    --output-dir instances
```

Output:

```text
instances/
  3_pnts_inst01.json
  ...
  6_pnts_inst10.json
  summary.json
```

## Step 1: train

Searches for a GRAHF island graph and stores the elitist. This stage calls IRACE, which
needs R and the `irace` package from the Nix flake, so it only works in the `dev-nix`
image — the default `dev` container does not include Nix. Start the Nix-enabled
container instead (`HM_IBF_NIX=1` builds and uses `hm-ibf-robo:dev-nix`, in its own
`hm-ibf-robo-nix` container, so it does not disturb a plain `dev` container you may
already have running):

```bash
HM_IBF_NIX=1 ./run.sh          # host, opens a shell in the Nix-enabled container
hm-ibf train                    # inside that shell
```

`hm-ibf` detects it is running where `nix` is on `PATH` and automatically re-execs
itself through `nix develop` — no need to type `nix develop --command …` yourself.

Useful flags (`--help` lists them all):

| Flag | Default | Meaning |
| --- | --- | --- |
| `--seed` | `42` | Seed of the outer structure search. |
| `--training-params` | `../params_training.conf` | TOML file with the search's tuning parameters (see below). |
| `--jobs` | available cores | Worker threads. |

The search's algorithm tuning parameters — generations, repetitions, IRACE tuning budget,
the evaluation budget shared with `evaluate`, the outer GA's own hyper-parameters, and the
`dimensions_allowed` list of allowed island dimensions — live in
[`params_training.conf`](../params_training.conf) at the repository root instead of CLI
flags, so they can be edited without recompiling. Edit that file directly; `hm-ibf` reads
it fresh on every invocation. `dimensions_allowed` must be non-empty, strictly increasing,
and every entry a positive multiple of `JOINTS` (`6`); changing it invalidates any
previously trained `elitist_*.json` (its stored `dimension` may no longer be an allowed
value), so retrain after editing it.

Output in `robo_run/`:

```text
robo_run/
  elitist_0.json      the island graph
  elitist_0.dot       the same graph, for graphviz
  elitist_0.params    the IRACE-tuned parameters
  run.log
  statistics.json
  irace/              IRACE working directory
```

## Step 2: evaluate

Replays the trained graph on every instance for 15 consecutive seeds. This stage does not
call IRACE, so the container's own toolchain is enough — the default `dev` container is
fine:

```bash
hm-ibf evaluate
```

| Flag | Default | Meaning |
| --- | --- | --- |
| `--first-seed` | `42` | First evaluation seed. |
| `--num-seeds` | `15` | Number of consecutive seeds. |
| `--elitist` | `elitist_0` | Base name of the trained artefacts. |
| `--results-dir` | `results` | Destination of the run folders. |
| `--summary-csv` | `results_robo.csv` | Aggregated summary. |
| `--evaluation-params` | `../params_evaluation.conf` | TOML file with the rebuild bounds and best-value tolerance. |

The island bounds used to rebuild the trained graph only shape IRACE's parameter space —
evaluation replays the stored, tuned parameters, so any value reproduces the same runs —
and the tolerance used to cross-check the exported best value live in
[`params_evaluation.conf`](../params_evaluation.conf) at the repository root, in the same
spirit as `params_training.conf` above.

Output:

```text
results/
  3_pnts_inst01_GRAHF_seed42_<hash8>/
    results.json      global best x, its exact f(x), and the run metadata
    best_value.csv    best value against evaluations
    avg_value.csv     mean island value against evaluations
    run.log
  ...
results_robo.csv      mean ± std per target-point count, over all seeds
```

`results.json` uses export schema `4`: one `x`, one `solution_dim`, one `nr_changes`.

Both stages in one go (needs the Nix-enabled container, same as `train`):

```bash
hm-ibf pipeline
```

## Notes

- Allowed island dimensions: `dimensions_allowed` in `params_training.conf`, shipped as
  `6, 12, 18, 24, 30` (`M = 1, 2, 3, 4, 5`).
- Island node weights: `0 = de`, `1 = es`, `2 = ls`, `3 = sa`, `4 = rs`, `5 = archive`.
- Migration transforms: `Akima`, `ClampedCubic`, `CT_Spline`, `DouglasPeucker`, `PCHIP`,
  `TVDenoise`, `VSpline`.
- Training uses a single seed by default; evaluation uses 15.
