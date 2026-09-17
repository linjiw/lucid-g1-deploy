# Source this in every shell that builds or runs the bundle:  source env.sh
#
# Source it DIRECTLY. `source env.sh | tail` runs it in a subshell and silently
# discards every export.
#
# TensorRT_ROOT is deliberately NOT set. runner/cmake/FindTensorRT.cmake calls
# find_path without NO_DEFAULT_PATH, so an apt-installed TensorRT under
# /usr/include/x86_64-linux-gnu is found through CMake's default system search.
# Pointing TensorRT_ROOT at a directory without the tarball's include/ + lib/
# layout makes the search fail rather than succeed.
_tc="$HOME/opt/sonic-deploy-toolchain"
export LUCID_DEPLOY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PATH="$HOME/.local/bin:$PATH"

# The first glob is the tarball setup.sh unpacks -- x86_64, gpu_cuda12 -- and it
# is tried first so an x86_64 box resolves to exactly the directory it did
# before. The two fallbacks exist because setup.sh refuses to run on aarch64
# ("This bundle's setup targets x86_64 Ubuntu") and the documented arm64 path,
# runner/scripts/install_deps.sh, unpacks onnxruntime-linux-aarch64 into
# /opt/onnxruntime instead (install_deps.sh:380 ONNX_INSTALL_PATH, :390 the
# aarch64 tarball URL, :416 the sudo mv into it) -- a different directory name
# AND a different parent, so the x64 glob could never match it. Without a
# fallback env.sh exports no onnxruntime_ROOT on the Orin and build.sh:5 answers
# "run 'source env.sh' first" to an operator who has just done exactly that.
# Read from install_deps.sh, not executed: there is no aarch64 box in reach, so
# both fallbacks are gated on lib/cmake/onnxruntime actually being there rather
# than assumed to be. Ungated, a half-unpacked tarball under $_tc would set
# onnxruntime_ROOT, satisfy build.sh:5, and turn a clear "run 'source env.sh'
# first" into a find_package failure in the middle of a configure.
_ort="$(ls -d "$_tc"/onnxruntime-linux-x64-gpu_cuda12-* 2>/dev/null | sort -V | tail -1)"
if [ -z "$_ort" ]; then
  _ort="$(ls -d "$_tc"/onnxruntime-linux-*/lib/cmake/onnxruntime 2>/dev/null \
          | sort -V | tail -1)"
  _ort="${_ort%/lib/cmake/onnxruntime}"
fi
if [ -z "$_ort" ] && [ -d /opt/onnxruntime/lib/cmake/onnxruntime ]; then
  _ort=/opt/onnxruntime
fi
if [ -n "$_ort" ]; then
  export onnxruntime_ROOT="$_ort"
  export onnxruntime_DIR="$_ort/lib/cmake/onnxruntime"
  export CMAKE_PREFIX_PATH="$_ort:${CMAKE_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="$_ort/lib:${LD_LIBRARY_PATH:-}"
fi
[ -f "$_tc/include/zmq.hpp" ] && export CPLUS_INCLUDE_PATH="$_tc/include:${CPLUS_INCLUDE_PATH:-}"

_cuda="$(ls -d /usr/local/cuda-12.* /usr/local/cuda 2>/dev/null | sort -V | tail -1)"
if [ -n "$_cuda" ]; then
  export CUDA_HOME="$_cuda" CUDAToolkit_ROOT="$_cuda"
  export PATH="$_cuda/bin:$PATH"
  export LD_LIBRARY_PATH="$_cuda/lib64:${LD_LIBRARY_PATH:-}"
fi
# The bundle's own python venv, created by setup.sh. tools/ and test.sh need
# numpy, onnxruntime, pyyaml, joblib, scipy and mujoco, none of which a fresh
# Ubuntu box has, and none of which belong in the system python.
if [ -x "$LUCID_DEPLOY_ROOT/.venv/bin/python" ]; then
  export PATH="$LUCID_DEPLOY_ROOT/.venv/bin:$PATH"
  export PYTHON="$LUCID_DEPLOY_ROOT/.venv/bin/python"
fi

# The DDS robot simulator runs in its OWN venv (.venv-sim) at python 3.10-3.12,
# because cyclonedds 0.10.2 -- the version unitree_sdk2py pins -- does not import
# on 3.13. Its two import roots are the vendored SDK and the vendored simulator;
# neither is pip-installed, so PYTHONPATH is how they are found.
if [ -x "$LUCID_DEPLOY_ROOT/.venv-sim/bin/python" ]; then
  export LUCID_SIM_PYTHON="$LUCID_DEPLOY_ROOT/.venv-sim/bin/python"
  export PYTHONPATH="$LUCID_DEPLOY_ROOT/sim:$LUCID_DEPLOY_ROOT/sdk:${PYTHONPATH:-}"
fi
# CycloneDDS built from the libraries vendored inside runner/thirdparty.
[ -d "$_tc/cyclonedds/lib" ] && export CYCLONEDDS_HOME="$_tc/cyclonedds"

unset _tc _ort _cuda

printf 'lucid-g1-deploy env:\n'
printf '  bundle      %s\n' "$LUCID_DEPLOY_ROOT"
printf '  just        %s\n' "$(command -v just || echo MISSING)"
printf '  cmake       %s\n' "$(command -v cmake || echo MISSING)"
printf '  python      %s\n' "${PYTHON:-MISSING (run setup.sh)}"
printf '  onnxruntime %s\n' "${onnxruntime_DIR:-MISSING -- setup.sh (x86_64), install_deps.sh (arm64)}"
printf '  sim python  %s\n' "${LUCID_SIM_PYTHON:-MISSING (DDS simulator unavailable)}"
# $(uname -m), not x86_64 hardcoded. TensorRT is a Debian multiarch package, so
# on the robot's own Orin both halves use the aarch64-linux-gnu triplet:
# install_deps.sh:862 names the library half ("TensorRT is usually available at
# /usr/lib/aarch64-linux-gnu/") and the headers follow the same triplet by
# multiarch convention -- install_deps.sh names no include path at all. The
# x86_64 path can never exist there, so this line would print MISSING on an Orin
# that had TensorRT, and the remedy it named was setup.sh, which refuses to run
# on aarch64 at all ("This bundle's setup targets x86_64 Ubuntu"). Naming
# install_deps.sh in its place would be little better: on a Jetson it does
# apt-install nvidia-jetpack (install_deps.sh:873), but it pins no TensorRT
# version anywhere, and the Orin needs 10.7 -- so the message points at the doc
# that says so. Read from install_deps.sh and checked on the x86_64 box this was
# written on; not executed on an Orin.
_trt="/usr/include/$(uname -m)-linux-gnu/NvInferVersion.h"
if [ -f "$_trt" ]; then
  printf '  TensorRT    %s\n' "$(grep -E 'define TRT_(MAJOR|MINOR|PATCH)_ENTERPRISE' \
    "$_trt" | awk '{printf "%s.", $3}' | sed 's/\.$//')"
elif [ "$(uname -m)" = x86_64 ]; then
  printf '  TensorRT    MISSING -- run setup.sh\n'
else
  printf '  TensorRT    MISSING -- needs JetPack TensorRT 10.7, see docs/ETHERNET_AND_SDK.md\n'
fi
unset _trt
