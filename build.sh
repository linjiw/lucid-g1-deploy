#!/usr/bin/env bash
# Compile the C++ runner.  source env.sh first.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[ -n "${onnxruntime_ROOT:-}" ] || { echo "run 'source env.sh' first"; exit 1; }
BUILD="${BUILD_DIR:-$HERE/.build}"
echo "== configure =="
cmake -S "$HERE/runner" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release
echo "== compile =="
cmake --build "$BUILD" -j "${JOBS:-$(nproc)}"
echo
echo "runner: $HERE/runner/target/release/g1_deploy_onnx_ref"
ls -la "$HERE/runner/target/release/g1_deploy_onnx_ref"
