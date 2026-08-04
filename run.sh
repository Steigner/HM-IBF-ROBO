#!/usr/bin/env bash
#
# Entry point for every command in this repository.
#
# All builds, tests and pipeline runs happen inside the container built from `Dockerfile`
# (see AGENTS.md). This script builds the `dev` image if it is missing, starts the
# `hm-ibf-robo` container if it is not running, and forwards its arguments to a shell
# inside it. Once inside (interactive shell, or `docker exec -it hm-ibf-robo bash`), the
# pipeline itself is the `hm-ibf` command installed on PATH — see hm-ibf-robo/runbook.md.
#
#   ./run.sh                       # interactive shell in the container
#   ./run.sh verify                # the full verification gate
#   ./run.sh cargo test --workspace
#
# Set HM_IBF_NIX=1 to use the `dev-nix` image instead (adds Nix/R/IRACE, needed for
# `hm-ibf train`/`pipeline`). It runs as a separate container so both variants can be up
# at the same time:
#
#   HM_IBF_NIX=1 ./run.sh          # shell in the Nix-enabled container
#
set -euo pipefail

TARGET_VOLUME="hm-ibf-robo-target"
if [ "${HM_IBF_NIX:-}" = "1" ]; then
    IMAGE="hm-ibf-robo:dev-nix"
    CONTAINER="hm-ibf-robo-nix"
    BUILD_TARGET="dev-nix"
else
    IMAGE="hm-ibf-robo:dev"
    CONTAINER="hm-ibf-robo"
    BUILD_TARGET="dev"
fi

cd "$(dirname "$0")"

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "==> Building $IMAGE"
    DOCKER_BUILDKIT=1 docker build --target "$BUILD_TARGET" -t "$IMAGE" .
fi

if [ -z "$(docker ps -q -f "name=^${CONTAINER}$")" ]; then
    docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
    docker volume create "$TARGET_VOLUME" >/dev/null
    echo "==> Starting $CONTAINER"
    # The target directory lives in a named volume: bind-mounting it would make cargo
    # builds crawl on Docker Desktop's shared filesystem. Both image variants share it,
    # since they use the same Rust toolchain version.
    docker run -d --name "$CONTAINER" \
        -v "$(pwd)":/app \
        -v "$TARGET_VOLUME":/app/target \
        -w /app \
        "$IMAGE" sleep infinity >/dev/null
fi

if [ "$#" -eq 0 ]; then
    exec docker exec -it "$CONTAINER" bash
fi

if [ "$1" = "verify" ]; then
    exec docker exec "$CONTAINER" bash /app/scripts/verify.sh
fi

exec docker exec "$CONTAINER" bash -c 'cd /app && "$@"' -- "$@"
