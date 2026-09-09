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

_ort="$(ls -d "$_tc"/onnxruntime-linux-x64-gpu_cuda12-* 2>/dev/null | sort -V | tail -1)"
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
printf '  onnxruntime %s\n' "${onnxruntime_DIR:-MISSING}"
printf '  sim python  %s\n' "${LUCID_SIM_PYTHON:-MISSING (DDS simulator unavailable)}"
if [ -f /usr/include/x86_64-linux-gnu/NvInferVersion.h ]; then
  printf '  TensorRT    %s\n' "$(grep -E 'define TRT_(MAJOR|MINOR|PATCH)_ENTERPRISE' \
    /usr/include/x86_64-linux-gnu/NvInferVersion.h | awk '{printf "%s.", $3}' | sed 's/\.$//')"
else
  printf '  TensorRT    MISSING -- run setup.sh\n'
fi
