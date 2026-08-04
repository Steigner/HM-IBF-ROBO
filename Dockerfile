# syntax=docker/dockerfile:1
#
# Multi-stage build for the GRAHF / HM-IBF-ROBO workspace.
#
#   base    - Rust toolchain plus the native libraries the workspace links against.
#   builder - compiles the release binary (BuildKit cache mounts keep deps warm).
#   dev     - sanctioned execution environment for build / lint / test (see AGENTS.md).
#   dev-nix - dev plus Nix, for the IRACE-backed training stage that needs R.
#   runtime - final, minimal, non-root image shipping only the binary and instances.
#
# `docker build .` produces `runtime`, the last stage. Use `--target dev` for the
# verification gate (this is what `run.sh` does) and `--target dev-nix` for training.

# `Cargo.lock` pins crates that require edition2024, so cargo >= 1.85 is mandatory.
ARG RUST_VERSION=1.90
ARG DEBIAN_RELEASE=bookworm
ARG APP_UID=1000

# --------------------------------------------------------------------------- #
# base                                                                        #
# --------------------------------------------------------------------------- #
FROM rust:${RUST_VERSION}-${DEBIAN_RELEASE} AS base

# `irace-rs` embeds CPython through `pyo3`, so libpython headers are build inputs.
RUN apt-get update && apt-get install -y --no-install-recommends \
        clang \
        git \
        libclang-dev \
        libpython3-dev \
        pkg-config \
    && rm -rf /var/lib/apt/lists/*

ENV LIBCLANG_PATH=/usr/lib/llvm-14/lib

WORKDIR /app

# --------------------------------------------------------------------------- #
# builder                                                                     #
# --------------------------------------------------------------------------- #
FROM base AS builder

COPY . .

# Cache mounts keep the registry and the target directory warm across builds, so a
# source-only change never recompiles the dependency tree. The binary is copied out of
# the cache mount because cache mounts are not part of the image layer.
RUN --mount=type=cache,target=/usr/local/cargo/registry,sharing=locked \
    --mount=type=cache,target=/usr/local/cargo/git,sharing=locked \
    --mount=type=cache,target=/app/target,sharing=locked \
    cargo build --release --workspace --bins \
    && mkdir -p /out \
    && cp target/release/hm-ibf-robo /out/

# --------------------------------------------------------------------------- #
# dev                                                                         #
# --------------------------------------------------------------------------- #
FROM base AS dev
ARG APP_UID

ENV VIRTUAL_ENV=/opt/venv \
    PATH=/opt/venv/bin:$PATH \
    RUST_BACKTRACE=full \
    RUST_LOG=info

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3 \
        python3-venv \
    && rm -rf /var/lib/apt/lists/* \
    && python3 -m venv "$VIRTUAL_ENV"

# Python tooling for the verification gate plus the runtime deps of the scripts.
COPY requirements-dev.txt /tmp/requirements-dev.txt
RUN pip install --no-cache-dir -r /tmp/requirements-dev.txt \
    && rm /tmp/requirements-dev.txt

RUN rustup component add clippy rustfmt \
    && groupadd --gid ${APP_UID} robo \
    && useradd --uid ${APP_UID} --gid robo --create-home robo \
    && mkdir -p /app /app/target \
    && chown -R robo:robo /app "$CARGO_HOME" "$RUSTUP_HOME"

# `hm-ibf` is the day-to-day pipeline CLI: it wraps `cargo build` plus the binary so an
# interactive shell can just call `hm-ibf train` instead of
# `cargo run -p grahf-robo --bin hm-ibf-robo -- train`.
COPY --chmod=755 scripts/hm-ibf-entrypoint.sh /usr/local/bin/hm-ibf

USER robo
CMD ["bash"]

# --------------------------------------------------------------------------- #
# dev-nix                                                                     #
# --------------------------------------------------------------------------- #
FROM dev AS dev-nix

USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl xz-utils ca-certificates \
    && rm -rf /var/lib/apt/lists/* \
    && mkdir -m 0755 /nix && chown robo:robo /nix
COPY nix.conf /etc/nix/nix.conf

USER robo
# Single-user install: the daemon variant needs an init system the container lacks.
RUN curl -fsSL https://nixos.org/nix/install | sh -s -- --no-daemon
ENV PATH=/home/robo/.nix-profile/bin:$PATH
# `/app` is bind-mounted at runtime and does not carry robo's ownership on the host side,
# so the flake's libgit2-based git fetcher refuses to read it without this.
RUN git config --global --add safe.directory /app

# --------------------------------------------------------------------------- #
# runtime                                                                     #
# --------------------------------------------------------------------------- #
FROM debian:${DEBIAN_RELEASE}-slim AS runtime
ARG APP_UID

# `pyo3` links libpython, so the shared library must be present at runtime too.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libpython3.11 \
    && rm -rf /var/lib/apt/lists/* \
    && groupadd --system --gid ${APP_UID} robo \
    && useradd --system --uid ${APP_UID} --gid robo --create-home robo

WORKDIR /work

COPY --from=builder /out/hm-ibf-robo /usr/local/bin/
COPY --chown=robo:robo hm-ibf-robo/instances /work/instances

# Runtime configuration comes from the environment or from mounted volumes only.
ENV RUST_LOG=info \
    RUST_BACKTRACE=1

USER robo
VOLUME ["/work/results", "/work/robo_run"]
ENTRYPOINT ["hm-ibf-robo"]
CMD ["evaluate"]
