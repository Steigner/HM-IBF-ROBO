# HM-IBF-ROBO

Hyper-heuristic design of heterogeneous island-model metaheuristics, applied to 6-DOF
robot trajectory optimization.

The workspace holds two crates:

| Crate | Path | Purpose |
| --- | --- | --- |
| `grahf` | `src/` | The framework: island graphs are the search space, IRACE tunes each candidate, and the tuned algorithm is scored on the target problem. |
| `grahf-robo` | `hm-ibf-robo/` | The robotics benchmark: problem definition, island and migration builders, and the `hm-ibf-robo` pipeline binary. |

What makes the benchmark unusual is that islands may run at **different dimensions**,
configured by `dimensions_allowed` in `params_training.conf` (shipped as
`6, 12, 18, 24, 30` decision variables, i.e. `1, 2, 3, 4, 5` joint waypoints). Migrants
are resized on the fly by projecting them onto the shortest Cartesian route through the
instance's target points, so a solution keeps its meaning across island boundaries.

## Requirements

Docker engine and `git` — that's all you need on the host. Clone the repository first; from
there on, everything runs inside the container built from `Dockerfile`, never on the
host:

```bash
git clone https://github.com/Steigner/HM-IBF-ROBO.git
cd HM-IBF-ROBO
```

macOS/Linux/Git Bash:

```bash
./run.sh              # build the image if needed, start the container, open a shell
./run.sh verify       # run the full verification gate
```

Windows (PowerShell or cmd), no Git Bash or WSL required — note the `.\` prefix, which
PowerShell needs even though cmd.exe doesn't:

```powershell
.\run.bat              # build the image if needed, start the container, open a shell
.\run.bat verify       # run the full verification gate
```

`run.sh <command>` / `run.bat <command>` forwards any command into the container instead
of opening a shell, for example `./run.sh cargo test --workspace` or
`.\run.bat cargo test --workspace`.

`run.sh`/`run.bat` themselves must run on the **host** — they talk to the Docker daemon.
If you already have a shell inside the container (e.g. `docker exec -it hm-ibf-robo
bash`), don't call them again from there; just run the underlying command directly, for
example `bash scripts/verify.sh` instead of `./run.sh verify`.

## Pipeline

Once inside the container, the pipeline is the `hm-ibf` command installed on `PATH` — no
`cargo run -p grahf-robo --bin hm-ibf-robo --` boilerplate and no need to `cd` into the
right directory first:

```bash
hm-ibf --help
hm-ibf train      # search for an island graph
hm-ibf evaluate   # replay it over 15 seeds
hm-ibf pipeline   # both, in order
```

`train` and `pipeline` call IRACE, which needs R and the `irace` package from the Nix
flake, so they only work in the `dev-nix` image — the default `dev` container does not
include Nix. Start that variant instead (see
[hm-ibf-robo/runbook.md](hm-ibf-robo/runbook.md) for details):

```bash
HM_IBF_NIX=1 ./run.sh          # macOS/Linux/Git Bash
```

```powershell
$env:HM_IBF_NIX = "1"; .\run.bat     # PowerShell — note the semicolon, not &
```

```bat
set HM_IBF_NIX=1 & run.bat     REM cmd.exe
```

Then, inside the shell it opens: `hm-ibf train`.

`hm-ibf` detects it is running inside the Nix-enabled container and re-execs itself
through `nix develop` automatically — no manual `nix develop --command …` needed.
`evaluate` works with either container, but needs a previously trained `robo_run/`
(produced by `train`).

Instance generation is Python and only needed to regenerate the checked-in instances
(they're already committed under `hm-ibf-robo/instances/`, so this is optional). This one
command does everything, including the download — no manual `git clone` needed:

```bash
cd hm-ibf-robo
python3 -m preprocessing.prepare_instances
```

It automatically shallow-clones the upstream
[`Robotics-Benchmarking`](https://github.com/JakubKudela89/Robotics-Benchmarking)
repository — which holds the source point cloud, `Pos_pnts.mat` — into
`hm-ibf-robo/external/robotics-benchmark/`, then generates the 40 instance JSON files
from it. Later runs reuse that checkout instead of re-cloning. Pass
`--source <path/to/Pos_pnts.mat>` to skip the download entirely, e.g. if you already have
a local copy or have no network access.

## Smoke test

A quick end-to-end sanity check of **preprocess → train → evaluate**, using a 2-instance
subset and reduced tuning budgets so it finishes in a few minutes instead of hours (the
very first run also pays a one-time Nix/cargo download cost). It proves the pipeline is
wired together correctly; it is not a real training run and won't produce a useful result.

Needs the Nix-enabled container (see `train` above). Once inside it:

```bash
SMOKE=/tmp/hm-ibf-smoke
mkdir -p "$SMOKE"
cd /app/hm-ibf-robo

# 1. Preprocess: fetch the upstream repo, generate all 40 instances, keep 2 of them.
python3 -m preprocessing.prepare_instances --output-dir "$SMOKE/instances_full"
mkdir -p "$SMOKE/instances"
cp "$SMOKE/instances_full/3_pnts_inst01.json" \
   "$SMOKE/instances_full/4_pnts_inst01.json" "$SMOKE/instances/"

# 2. A minimal tuning config so training/evaluation stay fast.
cat > "$SMOKE/params_training.conf" <<'EOF'
epsilon = 1e-8
max_evaluations = 200
num_repetitions = 1
num_tuning_repetitions = 1
num_tuning_experiments = 10
num_iterations = 1
max_island_iterations = 5
max_island_population = 8
dimensions_allowed = [6, 12]

[grahf]
max_initial_nodes = 3
initial_edge_p = 0.3
population_size = 7
tournament_size = 3
archive_size = 1
elitist_freq = 2
pc = 0.68
rm_node = 0.10
rm_edge = 0.18
rm_node_weight = 0.10
rm_edge_weight = 0.22
EOF
cat > "$SMOKE/params_evaluation.conf" <<'EOF'
max_iterations = 50
max_population_size = 50
best_value_tolerance = 1e-9
EOF

# 3. Train: search for an island graph on the 2-instance subset.
hm-ibf train \
    --instances-dir "$SMOKE/instances" --run-dir "$SMOKE/robo_run" \
    --experiments-dir "$SMOKE/experiments_robo" \
    --training-params "$SMOKE/params_training.conf" \
    --evaluation-params "$SMOKE/params_evaluation.conf" \
    --seed 1

# 4. Evaluate: replay the trained graph once per instance.
hm-ibf evaluate \
    --instances-dir "$SMOKE/instances" --run-dir "$SMOKE/robo_run" \
    --results-dir "$SMOKE/results" --summary-csv "$SMOKE/results/results_robo.csv" \
    --training-params "$SMOKE/params_training.conf" \
    --evaluation-params "$SMOKE/params_evaluation.conf" \
    --num-seeds 1 --first-seed 1
```

Success looks like `$SMOKE/robo_run/elitist_0.json`/`.params` after step 3, and
`$SMOKE/results/results_robo.csv` plus one `results/<instance>_.../results.json` per
instance after step 4. `$SMOKE` lives outside the repo (`/tmp` inside the container), so
none of this is checked in; delete it (`rm -rf "$SMOKE"`) once you're done.

## Layout

```text
src/                     grahf framework
  components/            graph operators, island executor, migration transforms
  graph/                 directed graph type, binomial node partitioning
  problems/              algorithm-design problem, IRACE tuning, statistics
tests/                   grahf integration tests
hm-ibf-robo/
  src/                   grahf-robo library + `hm-ibf-robo` binary
  instances/             40 benchmark instances (checked in)
  preprocessing/         instance generation from Pos_pnts.mat
  tests/                 pytest suite + Rust integration tests
params_training.conf    train/pipeline algorithm tuning parameters (TOML)
params_evaluation.conf  evaluate/pipeline algorithm tuning parameters (TOML)
run.sh, run.bat          host entry point (shell, verify, arbitrary commands); .bat for PowerShell/cmd
scripts/hm-ibf-entrypoint.sh   installed as `hm-ibf` on PATH inside the dev image
scripts/verify.sh        the verification gate
.claude/skills/          agent skills; `hm-ibf-audit` classifies a tree against HM-IBF,
                         `hm-ibf-retarget` maps the edits to swap in another problem
```

## Using another problem

The pipeline is generic over the problem type, so pointing it at a different optimization
problem (a process profile, a control schedule, a design vector) is a bounded set of edits
rather than a rewrite. The `hm-ibf-retarget` skill enumerates them and checks the result:

```bash
./run.sh python3 .claude/skills/hm-ibf-retarget/retarget.py .          # the edit surface
./run.sh python3 .claude/skills/hm-ibf-retarget/retarget.py . --check  # after retargeting
```

The new domain needs a resolution-independent coordinate for its decision variables — the
axis migrants are resampled along. Without one, islands at different dimensions cannot
exchange anything meaningful and a fixed-dimension optimizer is the better tool; the skill's
fit test covers this.

## Development

- `./run.sh verify` must pass before a change is done: `cargo fmt`, `cargo clippy -D
  warnings` and `cargo test` for every crate, then `ruff check`, `ruff format --check`
  and `pytest` for every script.
- Coding rules, review guidelines and the agent-facing project guide live in
  [AGENTS.md](AGENTS.md).

## License

`GPL-3.0-or-later`, see [LICENSE](LICENSE).
