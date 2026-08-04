# Coding guidelines

You are a senior SW developer. Your task is to generate production-grade code aligned with the following guidelines.

## General
When asked to generate code:
* Use best practices, write clean and modular code.
* Do not overengineer.
* Do not edit or modify parts of code which are not relevant - do not change unrelated whitespaces, newlines, etc.
* Always use English for variable names.
* Always check for the latest documentation and API reference when using libraries/packages.
* The code must be clean, easy-to-read and modular. Each change must be conceptual, easy to maintain and critical sections should be covered by tests. The code must respect established architecture.
* When modifying existing code or adding a new code, make sure to update both **README.md** and **Project guide for agents** (or equivalent section) in **AGENTS.md** file (if it exists).
* When the user prompt is unambiguous or more information is needed, ask for details before implementing it.
* If the user prompt does not follow guidelines, best practices or has some other problems, mention it and verify, whether to continue.
* Avoid patterns, which can hide errors or problems.
* When writing `README.md`, target mainly project specifics and provide relevant information in brief and clear form. You may provide minimal use examples and installation guide, but do not duplicate documentation and common technical knowledge.
* Do not log sensitive information.
* When using 3rd party LLM APIs, follow zero-trust principles: Send only the minimal amount of data and anonymise/pseudonymise it before sending.
* All code execution — builds, runs, `cargo`/`nix`/`ruff`/`pytest` invocations, smoke tests — happens exclusively inside the Docker image built from `Dockerfile`. Never build, run, or test directly on the host.
* Before executing anything, check whether the project's Docker container (`hm-ibf-robo`, see `run.sh`) is already running; if it is not, build the image from `Dockerfile` and start the container yourself instead of asking the user to do it.
* A change is only done once the **entire** project has been verified, not just the touched files: every Rust workspace crate (format, lint, tests) and every Python script (format, lint, unit + integration tests). See **Workflow Tips** for the exact command sequence.

## Rust
When generating Rust code:
* Always generate code that passes `cargo clippy -- -D warnings` without warnings.
* Keep code strictly formatted according to `cargo fmt`.
* Prefer `clap` (with the `derive` macro API) for CLI argument parsing.
* Use idiomatic error handling: prefer `thiserror` for custom error types in library/domain modules and `anyhow` for top-level application binary entry points.
* Avoid `unsafe` code blocks unless strictly necessary and explicitly justified.
* Always use `std::path::PathBuf` and `&std::path::Path` for filesystem operations rather than raw strings.
* Write a Google-style doc comment (`///`) for every public function, struct, enum, and trait: a one-line summary, then `# Arguments`, `# Returns`, and `# Errors`/`# Panics` sections describing parameters, return values, and failure/panic conditions.
* Target the latest stable Rust edition (2021 edition or newer).
* Cover the whole design — not just new code — with tests: unit tests live inline in the same file as the code they test (`#[cfg(test)] mod tests`), while integration tests live under each crate's `tests/` directory; keep the two structurally separate.
* Keep each `.rs` file to at most 500 lines; split into modules before exceeding the limit.

## Python
When generating or modifying Python scripts (preprocessing, tooling):
* Write a Google-style docstring for every public module, function, method, and class: a one-line summary, then `Args:`, `Returns:`, and `Raises:` sections.
* Format and lint every script with `ruff format` and `ruff check`; a script is not done until both pass cleanly.
* Cover the whole design — not just new code — with tests: unit tests colocated with the module under test (e.g. `test_<module>.py` beside it, or mirrored under `tests/unit/`), and integration tests kept in a separate `tests/integration/` tree; write both for functionality and for cross-component integration.
* Keep each `.py` file to at most 500 lines; split into modules before exceeding the limit.

## Nix
When generating Nix configurations:
* Always use modern Nix Flakes (`flake.nix`).
* Ensure `devShells.default` includes all required toolchains (`rustc`, `cargo`, `clippy`, `rustfmt`, Python's `ruff` and `pytest`), language servers, and build utilities.
* Keep dependencies minimal, explicit, and pin dependencies via `flake.lock`.

## Docker
When generating Dockerfiles:
* Always use multi-stage builds to produce lightweight, minimal production runtime images (e.g., using `debian:bookworm-slim`, `distroless`, or `alpine` as final runtime stage).
* Optimize build caching (e.g., using `cargo-chef` or pre-building dependencies) to avoid re-compiling full dependency trees on source-only changes.
* Never run application containers as `root`; explicitly create and use an unprivileged system user.
* Ensure container configuration is supplied exclusively via environment variables or explicitly mounted configuration volumes.
* This project's `Dockerfile` is the single sanctioned execution environment: all builds, `cargo`/`nix`/`ruff`/`pytest` invocations, and smoke tests must run inside a container built from it. Keep it supplied with whatever the verification gate needs (Rust toolchain plus `rustfmt`/`clippy` components, `ruff`, `pytest`, and Python runtime deps).
* Whenever you touch the Docker workflow, re-check whether `Dockerfile` is still optimal against the practices above (layering/caching, image size, non-root user, env/volume-based config) and fix drift before relying on it.

---

# Review Guidelines
You are a senior SW developer and your task is to perform a pull (merge) request review. Do not summarise the changes done, rather provide constructive feedback: review concept and architecture, search for bugs and possible inefficiencies and propose fixes or improvements. If in doubt, ask for more context/information.

## Bugs and possible inefficiencies
Search for all bugs, security vulnerabilities and inefficiencies. Propose fixes/improvements according to your best knowledge.

## Code style
Refer to Coding guidelines to check whether the code matches the guidelines.

---

# Project Guide For Agents

- Overview: A Cargo workspace with two crates. `grahf` (`src/`) is a hyper-heuristic framework whose search space is a *graph of islands*: node weights choose the island metaheuristic, edge weights choose the migration policy, IRACE tunes each candidate, and the tuned algorithm is scored on the target problem. `grahf-robo` (`hm-ibf-robo/`) is the robotics benchmark built on it, plus the `hm-ibf-robo` pipeline binary and the Python instance generator.
- Defining trait of the benchmark: islands may run at **different dimensions**, configured by `dimensions_allowed` in `params_training.conf` (shipped as `6, 12, 18, 24, 30` variables = `1, 2, 3, 4, 5` joint waypoints). Migrants are resized by projecting them onto the shortest Cartesian route through the instance targets. No output projection is applied: the exported `x` is verbatim the global best individual.
- Island node weights are positional, fixed by the order of `islands::island_builders`: `0 = de`, `1 = es`, `2 = ls`, `3 = sa`, `4 = rs`, `5 = archive`, mirrored by the `ISLAND_*` constants.

## Directory Structure

- `src/`: The `grahf` framework.
  - `src/components/`: Graph operators (`initialization`, `mutation`, `recombination`, `normalization`), the island executor (`island.rs`) and the migration transform trait (`transform.rs`).
  - `src/graph/`: `DiGraph` (split into `di/mod.rs`, `di/generate.rs`, `di/topology.rs`), disjoint sets and the binomial node split used by crossover.
  - `src/problems/algorithm_design/`: The design problem (`mod.rs`), its evaluator (`evaluator.rs`), cross-instance statistics (`statistics.rs`), IRACE tuning (`tuning.rs`), builders (`builder.rs`).
- `tests/`: `grahf` integration tests.
- `hm-ibf-robo/`: The `grahf-robo` crate.
  - `src/cli.rs`: `clap` definitions for the `hm-ibf-robo` binary.
  - `src/config.rs`: Loads `params_training.conf`/`params_evaluation.conf` into `TrainingParams`/`EvaluationParams`.
  - `src/main.rs`: Thin binary entry point; all logic lives in the library.
  - `src/training.rs`, `src/evaluation/`: The train and evaluate stages.
  - `src/robo/`: Problem, evaluator and result export.
  - `src/islands/`, `src/migrations/`: Island and migration builders; `islands/transforms/` holds the dimension transforms.
  - `instances/`: The 40 benchmark instances (checked in).
  - `preprocessing/`: Python package that regenerates the instances from `Pos_pnts.mat`.
    `fetch_benchmark.py` auto-clones the upstream `Robotics-Benchmarking` repository into
    `external/robotics-benchmark/` when `prepare_instances.py` is run without `--source`.
  - `tests/`: `tests/unit/` (pytest) plus `tests/pipeline.rs` (Rust integration tests).
- `params_training.conf`, `params_evaluation.conf`: TOML files at the repository root holding the algorithm tuning parameters of the `train`/`pipeline` and `evaluate`/`pipeline` stages respectively; loaded by `hm-ibf-robo/src/config.rs`. See **Environment & Config**.
- `scripts/verify.sh`: The verification gate.
- `.claude/skills/hm-ibf-audit/`: Agent skill that classifies any source tree against the
  HM-IBF definition and reports deviations (`audit.py` scanner and CLI, `criteria.py` trait
  catalog, `model.py` dataclasses). Extend the catalog, not the scanner, when adding a trait.
- `scripts/hm-ibf-entrypoint.sh`: Installed by `Dockerfile` as `hm-ibf` on `PATH` inside the `dev`/`dev-nix` images; builds the release binary if needed and always runs it from `hm-ibf-robo/`. For `train`/`pipeline`, re-execs itself through `nix develop` when `nix` is on `PATH` and it is not already inside a Nix shell.
- `run.sh`, `run.bat`: Host entry point; builds/starts the container and forwards commands (interactive shell, verify, arbitrary commands, including `hm-ibf` itself). `run.bat` is the same thing for PowerShell/cmd, no Git Bash/WSL required; keep the two in sync.
- `Cargo.toml`, `Cargo.lock`, `rustfmt.toml`: Rust workspace metadata and formatting.
- `pyproject.toml`: `ruff` and `pytest` configuration.
- `requirements-dev.txt`: Python environment of the container's `dev` stage.
- `flake.nix`, `flake.lock`: Nix devshell (needed for R/IRACE during training).
- `Dockerfile`, `.dockerignore`: Multi-stage container specification.
- `AGENTS.md`, `README.md`, `hm-ibf-robo/README.md`, `hm-ibf-robo/runbook.md`: Docs.
- Transient: `target/`, `.direnv/`, `result`, `robo_run/`, `results/`, `trajectory_exports/`,
  `hm-ibf-robo/external/` (auto-cloned `Robotics-Benchmarking` checkout).

## Quickstart

- `./run.sh` (from the **host**; it talks to the Docker daemon) builds the `dev` image if missing, starts the `hm-ibf-robo` container and opens a shell at `/app`. `.\run.bat` is the Windows equivalent — the `.\` prefix is required in PowerShell (unlike cmd.exe, it does not run scripts from the current directory by bare name).
- `HM_IBF_NIX=1 ./run.sh` (bash) / `$env:HM_IBF_NIX = "1"; .\run.bat` (PowerShell — `&` is not a command separator there) / `set HM_IBF_NIX=1 & run.bat` (cmd.exe) uses the `dev-nix` image/`hm-ibf-robo-nix` container instead — adds Nix/R/IRACE, needed for `train`/`pipeline`. Runs alongside a plain `dev` container under a different name.
- `./run.sh <command>` runs a single command inside the container instead of opening a shell, e.g. `./run.sh hm-ibf --help`.
- `./run.sh verify` runs the whole verification gate.
- Inside the container, the pipeline is the `hm-ibf` command installed on `PATH` — no `cargo run -p grahf-robo --bin hm-ibf-robo --` boilerplate, and always run from `hm-ibf-robo/` so relative defaults (`instances/`, `robo_run/`, `results/`) resolve correctly regardless of the caller's directory. It auto-detects the Nix-enabled container and wraps `train`/`pipeline` in `nix develop` itself.
- The container mounts the repository at `/app` and keeps `target/` in a named volume, so cargo builds are not slowed down by the host filesystem.

## Common Commands

All commands run inside the container (see **General**: Docker-only execution).

- Develop:
  - Quick check: `cargo check --workspace --all-targets`
  - Nix shell (only needed for R/IRACE): `nix develop`
- Run:
  - Help: `hm-ibf --help`
  - Train (needs the `dev-nix`/`hm-ibf-robo-nix` container): `hm-ibf train`
  - Evaluate: `hm-ibf evaluate`
  - Both: `hm-ibf pipeline`
- Test:
  - Full suite: `cargo test --workspace`
  - Focused test: `cargo test <test_name>`
- Format/Lint:
  - Lint check: `cargo clippy --workspace --all-targets -- -D warnings`
  - Format check: `cargo fmt --all -- --check`
  - Auto-format: `cargo fmt --all`
- Build/Release:
  - Release build: `cargo build --release --workspace`
  - Runtime image: `docker build -t hm-ibf-robo:latest .`
  - Dev image: `docker build --target dev -t hm-ibf-robo:dev .`
- Python (from the repository root):
  - Lint check: `ruff check .`
  - Format check: `ruff format --check .`
  - Auto-format: `ruff format .`
  - Tests: `python3 -m pytest` (paths come from `pyproject.toml`)

Note: Prefer the existing tooling as configured; avoid duplicating dependency or linter settings present in `Cargo.toml`, `pyproject.toml` or `flake.nix`.

## Environment & Config

- Run identity and file locations (seeds, `--jobs`, `--instances-dir`, `--run-dir`, `--experiments-dir`, evaluation's `--results-dir`/`--summary-csv`/`--elitist`) are CLI flags (`src/cli.rs`); add new settings there with a documented default, and update `hm-ibf-robo/runbook.md`.
- The GRAHF search's and evaluate stage's algorithm *tuning parameters* live in two TOML files at the repository root instead: `params_training.conf` (read by `train`/`pipeline`; also supplies the `max_evaluations` budget shared with `evaluate` and the `dimensions_allowed` list of allowed island dimensions) and `params_evaluation.conf` (read by `evaluate`/`pipeline`). `hm-ibf-robo/src/config.rs` defines `TrainingParams`/`EvaluationParams` and loads them; the `Cli`'s `--training-params`/`--evaluation-params` flags only point at the file, defaulting to `../params_training.conf`/`../params_evaluation.conf` relative to the `hm-ibf-robo/` working directory. `TrainingParams::load` rejects a `dimensions_allowed` that is empty, not strictly increasing, or not a positive multiple of `robo::JOINTS`. Add new tuning parameters as fields there (with a doc comment) and update both `.conf` files and `hm-ibf-robo/runbook.md`.
- `RUST_LOG` selects the log level (`info` by default in the container; the flake sets `full` backtraces). The binaries log through `log`/`pretty_env_logger` — do not use `println!` in library code.
- The runtime image takes configuration exclusively from environment variables and mounted volumes (`/work/results`, `/work/robo_run`).
- Container Python lives in `/opt/venv`; add dependencies to `requirements-dev.txt` *and* to the `pythonEnv` in `flake.nix`.

## Security Guidelines

- Secrets: Never commit secrets; `.env` is ignored by VCS and by Docker.
- Inputs: Validate and sanitize all CLI and file inputs; avoid passing user input directly to shell or external processes.
- Filesystem: Use `std::path::PathBuf`, avoid writing outside the project or OS temp dirs; handle permissions and existence checks explicitly.
- Dependencies: This project is licensed `GPL-3.0-or-later` (see `LICENSE`); any OSI-approved open-source license — permissive or copyleft — is acceptable for Rust crates and Python packages alike. Avoid proprietary or non-OSI "source-available" dependencies.
- Logging: Do not log secrets or PII.
- Execution: Avoid `unsafe` code blocks, dynamic code evaluation, or deserializing untrusted data without validation.

## Workflow Tips

- Rust edition: Target Rust 2021 edition as defined in `Cargo.toml`. The pinned toolchain is `RUST_VERSION` in `Dockerfile`; `Cargo.lock` requires cargo >= 1.85 (edition2024 dependencies).
- Nix shell: Run `nix develop` *inside* the Docker container (never on the bare host). It is only needed for the training stage, which shells out to R/IRACE; everything else works with the container's own toolchain.
- Tests first: Add/adjust tests for any new behaviour. Rust unit tests live inline in the file they cover; Rust integration tests live in `tests/` (per crate); Python unit tests live in `hm-ibf-robo/tests/unit/` and integration tests in `hm-ibf-robo/tests/integration/`.
- CLI changes: Update `hm-ibf-robo/src/cli.rs`. Every subcommand shares the global flags on `Cli`; clap requires argument ids to be unique across flattened groups, and `cli::tests::the_command_definition_is_valid` catches violations.
- Island set changes: adding, removing or reordering an entry of `islands::island_builders` shifts the node weight encoding and invalidates every stored `elitist_*.json`. Update the `ISLAND_*` constants, `training::initial_population` and `hm-ibf-robo/README.md` together with the list. Editing `dimensions_allowed` in `params_training.conf` is a similar break: a stored elitist's IRACE-tuned `dimension` categorical may no longer be in the new set, so retrain after changing it.
- Export schema: `hm-ibf-robo/src/robo/output.rs` writes schema `4`; bump `OUTPUT_EXPORT_SCHEMA_VERSION` when the payload changes.
- Documentation: Update `README.md` for user-facing changes and this **Project Guide For Agents** section for agent-facing guidance. For the `hm-ibf-robo/` crate, also keep `hm-ibf-robo/README.md` and `hm-ibf-robo/runbook.md` current whenever preprocessing, training or evaluation behaviour changes.
- Definition of done: inside the container, run `./run.sh verify` (or `scripts/verify.sh` directly). It runs `cargo fmt --check`, `cargo clippy -- -D warnings` and `cargo test` for every workspace crate, then `ruff check`, `ruff format --check` and the full `pytest` suite. Nothing is finished until all of these are green — not just the parts you touched.
- Automation: the preprocess → train → evaluate pipeline runs end-to-end without manual steps. The Rust stages are one `clap` binary with `train`/`evaluate`/`pipeline` subcommands; instance generation stays a separate Python module because it is an independent tool with its own dependencies. Keep new automation inside the binary rather than adding ad hoc scripts.

## Known Pitfalls

- Host execution: `cargo`/`pytest` on the bare host will not match the pinned toolchain. Always go through `run.sh`.
- Toolchain floor: building with cargo < 1.85 fails on `hashbrown` ("feature `edition2024` is required"). Bump `RUST_VERSION` rather than editing `Cargo.lock`.
- `rustfmt.toml` sets `imports_granularity` and `group_imports`, which are nightly-only. Stable `cargo fmt` prints a warning for each and ignores them; the exit code is unaffected.
- Nix lock drift: If system packages or Rust dependencies fail to build in Nix, update the flake via `nix flake update`.
- Nix flake vs. bind mount: `/app` is bind-mounted from the host, so the `dev-nix` image's `git config --global --add safe.directory /app` is required or `nix develop`'s libgit2-based flake fetcher refuses it ("repository path is not owned by current user"). Keep this if the Dockerfile's user/mount setup changes.
- Docker context: `.dockerignore` excludes `target/`, `.git/` and experiment output; keep it in sync when adding generated directories.
- Instance regeneration needs network access: `preprocessing.fetch_benchmark` shells out to `git clone` against the upstream `Robotics-Benchmarking` repository unless `--source` points at an already-downloaded `Pos_pnts.mat`. Offline/air-gapped runs must pass `--source` explicitly.

## When Adding Code

- Structure: Keep modules cohesive and small; separate CLI (`cli.rs`), stage orchestration (`training.rs`, `evaluation/`) and domain logic (`robo/`, `islands/`). Keep each `.rs`/`.py` file to at most 500 lines; split into modules before exceeding the limit.
- Types & docs: Leverage Rust's strong type system, custom `enum`s for state, and write Google-style doc comments (summary + `# Arguments`/`# Returns`/`# Errors`/`# Panics`) for public constructs; the same docstring standard applies to every Python module, function, method and class.
- Errors: `thiserror`/`eyre` in library code, `anyhow`/`eyre` at the binary entry point. Do not swallow errors: fall back only where the fallback is correct and documented, and prefer failing loudly over guessing.
- Tests: Cover new logic with unit tests colocated in the same file/module and integration tests kept separately. Run the whole gate before considering a change done.
- Config: Run identity and file locations are CLI flags with safe defaults; algorithm tuning parameters are fields on `TrainingParams`/`EvaluationParams` (`hm-ibf-robo/src/config.rs`) read from `params_training.conf`/`params_evaluation.conf`. Document either kind in `hm-ibf-robo/runbook.md`.
- Dependencies: Keep Cargo/Python dependencies minimal; any OSI-approved open-source license is acceptable given this project's own `GPL-3.0-or-later` license.
