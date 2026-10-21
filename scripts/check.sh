#!/usr/bin/env bash
#
# Local mirror of the CI pipeline (.github/workflows/ci.yml).
#
# The CI has two jobs: a fast static-checks job that needs no ROS, and a
# build-and-test job that builds the workspace with -Werror and runs
# `colcon test` (the GTest unit tests plus the ament linters). This script
# runs both, so `make check` answers "would CI be green?" before a push.
#
# The build runs in a *separate* workspace inside the container rather than in
# /workspace/interceptor_ws. The container bind-mounts its src directory from
# wherever it was started, which is not necessarily this checkout, and its
# build/install volumes are shared with everything else that runs in it.
# Copying the source into a scratch workspace makes the check independent of
# both: it always tests the tree you point it at, and it never disturbs the
# volumes the main build uses.
#
# Usage:
#   scripts/check.sh                 # check this checkout
#   scripts/check.sh <src-dir>       # check another source tree (e.g. a worktree)
#
# Environment:
#   CONTAINER   name of the Docker container   (default: interceptor)
#   KEEP_WS     set to 1 to keep the scratch workspace for inspection

set -euo pipefail

CONTAINER="${CONTAINER:-interceptor}"
KEEP_WS="${KEEP_WS:-0}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

SRC_DIR="${1:-${REPO_ROOT}/interceptor_ws/src}"
if [ ! -d "${SRC_DIR}" ]; then
  echo "error: no such source directory: ${SRC_DIR}" >&2
  exit 2
fi
SRC_DIR="$(cd "${SRC_DIR}" && pwd)"

FAILURES=0
BUILD_FAILED=0

section() {
  echo ""
  echo "=============================================================="
  echo "  $1"
  echo "=============================================================="
}

fail() {
  echo "FAIL: $1" >&2
  FAILURES=$((FAILURES + 1))
}

# --- 1. Static checks (no ROS needed) ------------------------------------
section "Static checks"

if command -v python3 > /dev/null 2>&1; then
  if python3 "${REPO_ROOT}/scripts/roadmap.py" check; then
    echo "roadmap: OK"
  else
    fail "roadmap check"
  fi

  if git -C "${REPO_ROOT}" ls-files '*.py' | xargs -r python3 -m py_compile; then
    echo "python syntax: OK"
  else
    fail "python syntax"
  fi
else
  echo "skip: python3 not found"
fi

if command -v shellcheck > /dev/null 2>&1; then
  # Lint the scripts of this checkout, not of the container image.
  if (cd "${REPO_ROOT}" && git ls-files '*.sh' | xargs -r shellcheck --severity=warning); then
    echo "shellcheck: OK"
  else
    fail "shellcheck"
  fi
else
  echo "skip: shellcheck not installed"
fi

if command -v yamllint > /dev/null 2>&1; then
  # Only the tracked YAML files: CI lints a clean checkout, and this repo's
  # working copy also holds .claude/worktrees, which are not part of it.
  if (cd "${REPO_ROOT}" && git ls-files '*.yml' '*.yaml' | xargs -r yamllint --strict); then
    echo "yamllint: OK"
  else
    fail "yamllint"
  fi
else
  echo "skip: yamllint not installed"
fi

# --- 2. Build and test ---------------------------------------------------
section "Build and test (container: ${CONTAINER})"

if ! docker inspect "${CONTAINER}" > /dev/null 2>&1; then
  echo "error: container '${CONTAINER}' not found; run 'make up' first" >&2
  exit 2
fi

# A unique scratch workspace inside the container. The name carries the run
# id so two checks never share a directory, and the guard below makes sure
# the cleanup can only ever touch a path this script created.
RUN_ID="$$-$(date +%s)"
WS="/tmp/skyintercept-check-${RUN_ID}"
WS_PREFIX="/tmp/skyintercept-check-"

cleanup() {
  if [ "${KEEP_WS}" = "1" ]; then
    echo "workspace kept at ${WS} in ${CONTAINER}"
    return
  fi
  case "${WS}" in
    "${WS_PREFIX}"*) docker exec "${CONTAINER}" rm -rf "${WS}" > /dev/null 2>&1 || true ;;
  esac
}
trap cleanup EXIT

echo "workspace: ${WS} (in ${CONTAINER})"
echo "source:    ${SRC_DIR}"

docker exec "${CONTAINER}" mkdir -p "${WS}/src"
# The trailing /. copies the *contents* of src; without it colcon finds a
# nested src/ directory and refuses the duplicate package names.
docker cp "${SRC_DIR}/." "${CONTAINER}:${WS}/src/"

echo ""
echo "--- build (-Werror, as CI does) ---"
if docker exec "${CONTAINER}" bash -lc "cd ${WS} && \
    source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-Werror"; then
  echo "build: OK"
else
  fail "build"
  BUILD_FAILED=1
fi

echo ""
echo "--- test ---"
# The build failed: the tests would only report the same failure again.
if [ "${BUILD_FAILED}" -eq 0 ]; then
  if docker exec "${CONTAINER}" bash -lc "cd ${WS} && \
      source /opt/ros/humble/setup.bash && \
      source install/setup.bash && \
      colcon test --return-code-on-test-failure"; then
    echo "colcon test: OK"
  else
    fail "colcon test"
  fi

  echo ""
  echo "--- test results ---"
  docker exec "${CONTAINER}" bash -lc "cd ${WS} && colcon test-result --verbose" || fail "colcon test-result"
else
  echo "skip: the build failed"
fi

# --- 3. Verdict ----------------------------------------------------------
section "Result"
if [ "${FAILURES}" -eq 0 ]; then
  echo "All checks passed."
  exit 0
fi

echo "${FAILURES} check(s) failed."
exit 1
