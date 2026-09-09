#!/usr/bin/env bash
# One-shot toolchain setup for this bundle on a fresh Ubuntu 22.04 x86_64 box.
#
#   bash setup.sh              install everything
#   bash setup.sh --dry-run    print what would happen, change nothing
#   bash setup.sh --no-sudo    only the parts that do not need root
#
# Two halves. The no-sudo half lands in ~/.local/bin and
# ~/opt/sonic-deploy-toolchain and can be removed by deleting those. The sudo
# half is apt packages plus NVIDIA's CUDA repository.
#
# Run this in a REAL TERMINAL. sudo needs a tty to prompt for a password, and
# will fail with "a terminal is required to read the password" if this is piped
# or run from a tool that does not allocate one.
#
# Every version here was chosen for a reason, and the reasons are load-bearing:
#
#   CUDA 12.9 runtime, not 13   Blackwell (RTX 50xx) needs 12.8+, and TensorRT
#                               10.x -- which is what this runner's code targets
#                               -- is built against the CUDA 12 line.
#   TensorRT pinned to 10.13    NOT a preference. SONIC's own
#       with its FULL closure   docs/source/getting_started/installation_deploy.md
#                               requires exactly 10.13 on x86_64 (and exactly 10.7
#                               on the G1's onboard Orin), under a `danger`
#                               admonition: another version "is known to produce
#                               incorrect inference results -- the planner will
#                               output wrong motion, which can cause dangerous
#                               robot behavior." Pin the FULL closure, not just the
#                               -dev packages: each -dev depends on an exactly-equal
#                               runtime package that otherwise resolves to the
#                               newest cuda13 build and the install fails outright.
#                               libnvinfer-safe-headers-dev is deliberately absent:
#                               it does not exist below 10.16, and nothing in the
#                               runner includes NvInferSafe*. A 10.16 copy left
#                               behind would shadow 10.13's headers, so setup
#                               removes it.
#   cuda-crt as well as cudart  Nothing here compiles .cu files, so nvcc is not
#                               needed -- but cuda_runtime_api.h includes four
#                               headers from crt/, which ship with the compiler
#                               headers rather than the runtime. 82 kB.
#   onnxruntime gpu_cuda12      Must match the CUDA line above.
#   cppzmq vendored             Ubuntu 22.04 has no cppzmq-dev; zmq.hpp is a
#                               single header. libzmq3-dev is the C library.
set -euo pipefail

DRY=0; NO_SUDO=0
for a in "$@"; do case "$a" in
  --dry-run) DRY=1 ;; --no-sudo) NO_SUDO=1 ;;
  *) echo "unknown option: $a"; exit 2 ;;
esac; done

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TC="$HOME/opt/sonic-deploy-toolchain"
ORT_VERSION=1.29.0
TRT_VERSION="${TRT_VERSION:-10.13.3.9-1+cuda12.9}"
TRT_LINE=10.13   # the version SONIC requires on x86_64; 10.7 on Jetson
CMAKE_VERSION=3.30.5

run() { echo "  \$ $*"; [ "$DRY" -eq 1 ] || "$@"; }
say() { echo; echo "== $* =="; }

[ "$(uname -m)" = "x86_64" ] || {
  echo "This bundle's setup targets x86_64 Ubuntu. On a Jetson (arm64) use"
  echo "runner/scripts/install_deps.sh, which handles the arm64 packages."; exit 1; }

# ---------------------------------------------------------------- no sudo --
say "1/5  just, cmake, onnxruntime, cppzmq  (no sudo, all under \$HOME)"
mkdir -p "$TC/include" "$HOME/.local/bin"
if ! command -v just >/dev/null && [ "$DRY" -eq 0 ]; then
  curl -sSfL https://just.systems/install.sh | bash -s -- --to "$HOME/.local/bin" >/dev/null
fi
echo "  just        $(command -v just || echo 'will install')"

if ! command -v cmake >/dev/null && [ "$DRY" -eq 0 ]; then
  curl -sSfL -o "$TC/cmake.tgz" \
    "https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-linux-x86_64.tar.gz"
  tar xzf "$TC/cmake.tgz" -C "$TC" && rm -f "$TC/cmake.tgz"
  ln -sf "$TC/cmake-${CMAKE_VERSION}-linux-x86_64/bin/cmake" "$HOME/.local/bin/cmake"
  ln -sf "$TC/cmake-${CMAKE_VERSION}-linux-x86_64/bin/ctest" "$HOME/.local/bin/ctest"
fi
echo "  cmake       $(command -v cmake || echo 'will install')"

ORT_DIR="$TC/onnxruntime-linux-x64-gpu_cuda12-${ORT_VERSION}"
if [ ! -d "$ORT_DIR" ] && [ "$DRY" -eq 0 ]; then
  curl -sSfL -o "$TC/ort.tgz" \
    "https://github.com/microsoft/onnxruntime/releases/download/v${ORT_VERSION}/onnxruntime-linux-x64-gpu_cuda12-${ORT_VERSION}.tgz"
  tar xzf "$TC/ort.tgz" -C "$TC" && rm -f "$TC/ort.tgz"
fi
echo "  onnxruntime $ORT_DIR"

if [ ! -f "$TC/include/zmq.hpp" ] && [ "$DRY" -eq 0 ]; then
  curl -sSfL -o "$TC/include/zmq.hpp" \
    https://raw.githubusercontent.com/zeromq/cppzmq/v4.10.0/zmq.hpp
fi
echo "  zmq.hpp     $TC/include/zmq.hpp"

if [ "$NO_SUDO" -eq 1 ]; then
  echo; echo "no-sudo half done: just, cmake, onnxruntime, cppzmq."
  echo "Still needed (root): apt packages, the python venv, CUDA and TensorRT."
  echo "Re-run without --no-sudo."
  exit 0
fi

# ------------------------------------------------------------------ sudo --
say "2/5  build tools and C++ libraries"
run sudo apt-get update
run sudo apt-get install -y \
  clang build-essential pkg-config patchelf zlib1g-dev libgtest-dev \
  git git-lfs curl wget ca-certificates \
  libmsgpack-dev libzmq3-dev libeigen3-dev nlohmann-json3-dev \
  python3-venv python3-pip

say "2b/5  python environment for tools/ and test.sh"
# tools/ and test.sh need numpy, onnxruntime, pyyaml, joblib, scipy and mujoco.
# A fresh Ubuntu box has none of them, and the system python should not be
# polluted, so they go in a venv inside the bundle. This comes AFTER the apt
# step because python3-venv is itself an apt package -- creating the venv first
# fails on a clean machine with "ensurepip is not available".
# onnxruntime CPU is enough here: the tools verify and convert, while the GPU
# inference is the C++ runner's job through TensorRT.
VENV="$HERE/.venv"
if [ ! -x "$VENV/bin/python" ] && [ "$DRY" -eq 0 ]; then
  rm -rf "$VENV"
  python3 -m venv "$VENV"
  "$VENV/bin/pip" -q install --upgrade pip
  "$VENV/bin/pip" -q install numpy onnxruntime pyyaml joblib scipy mujoco
fi
if [ -x "$VENV/bin/python" ] && [ "$DRY" -eq 0 ]; then
  "$VENV/bin/python" - <<'PYCHK' || echo "  (some packages missing; re-run setup.sh)"
import numpy, onnxruntime, yaml, joblib, scipy, mujoco
print(f"  numpy {numpy.__version__} · onnxruntime {onnxruntime.__version__} · mujoco {mujoco.__version__}")
PYCHK
fi

say "2c/5  Unitree SDK and the DDS robot simulator"
# Two things are needed to rehearse a deployment without a robot:
#
#   * unitree_sdk2py -- vendored in sdk/, not pip-installed. Pure python.
#   * cyclonedds 0.10.2 python bindings -- these need the CycloneDDS C library
#     to build against, and the version has to match. The runner already ships
#     one: runner/thirdparty/unitree_sdk2/thirdparty carries CycloneDDS 0.10.2
#     headers and libddsc.so for x86_64 and aarch64, which is exactly the version
#     unitree_sdk2py pins. Nothing is downloaded; a CYCLONEDDS_HOME is assembled
#     from what is already here.
#
# This lives in its own venv at python3.10. cyclonedds 0.10.2 predates
# Python 3.13 and its C extension references _Py_IsFinalizing, which 3.13
# removed -- it imports on 3.10/3.11/3.12 and fails at import on 3.13.
ARCH_DIR=$([ "$(uname -m)" = "aarch64" ] && echo aarch64 || echo x86_64)
TP="$HERE/runner/thirdparty/unitree_sdk2/thirdparty"
CDDS="$TC/cyclonedds"
if [ "$DRY" -eq 0 ] && [ -f "$TP/lib/$ARCH_DIR/libddsc.so" ]; then
  rm -rf "$CDDS"; mkdir -p "$CDDS/lib" "$CDDS/bin"
  cp -a "$TP/include" "$CDDS/include"
  cp -a "$TP/lib/$ARCH_DIR/libddsc.so" "$CDDS/lib/"
  ln -sf libddsc.so "$CDDS/lib/libddsc.so.0"
  # cyclone_search.good_directory() requires include/, lib/ AND bin/ to exist
  # before it will accept a CYCLONEDDS_HOME. bin/ is empty: idlc is only needed
  # to compile new IDL, and unitree_sdk2py ships pre-generated python types.
  echo "  CycloneDDS   $CDDS  (from the vendored unitree_sdk2, $ARCH_DIR)"
fi
VENVSIM="$HERE/.venv-sim"
if [ ! -x "$VENVSIM/bin/python" ] && [ "$DRY" -eq 0 ]; then
  SIMPY3=$(command -v python3.10 || command -v python3.11 || command -v python3.12 || true)
  if [ -z "$SIMPY3" ]; then
    echo "  ⚠ no python3.10/3.11/3.12 found; the DDS simulator will not be available."
    echo "    Everything else still works. Install one and re-run to enable drill.sh."
  else
    rm -rf "$VENVSIM"
    "$SIMPY3" -m venv "$VENVSIM"
    "$VENVSIM/bin/pip" -q install --upgrade pip setuptools wheel
    CYCLONEDDS_HOME="$CDDS" "$VENVSIM/bin/pip" -q install "cyclonedds==0.10.2" \
      || echo "  ⚠ cyclonedds build failed -- see the note above"
    "$VENVSIM/bin/pip" -q install mujoco numpy scipy pyyaml
  fi
fi
if [ -x "$VENVSIM/bin/python" ] && [ "$DRY" -eq 0 ]; then
  PYTHONPATH="$HERE/sim:$HERE/sdk" "$VENVSIM/bin/python" - <<'PYCHK' \
      || echo "  ⚠ the DDS simulator is not importable; drill.sh will not run"
import mujoco
from cyclonedds.domain import DomainParticipant  # noqa: F401
from unitree_sdk2py.core.channel import ChannelFactory  # noqa: F401
from gear_sonic_sim.simulator_factory import SimulatorFactory  # noqa: F401
print(f"  sim venv     python OK, mujoco {mujoco.__version__}, cyclonedds 0.10.2, unitree_sdk2py")
PYCHK
fi

say "3/5  NVIDIA CUDA apt repository"
if [ ! -f /usr/share/keyrings/cuda-archive-keyring.gpg ] && [ "$DRY" -eq 0 ]; then
  tmp=$(mktemp -d)
  curl -sSfL -o "$tmp/k.deb" \
    https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
  run sudo dpkg -i "$tmp/k.deb"; rm -rf "$tmp"
  run sudo apt-get update
else
  echo "  keyring already present (or dry run)"
fi

say "4/5  CUDA 12.9 runtime + crt headers"
run sudo apt-get install -y cuda-cudart-dev-12-9 cuda-crt-12-9

say "5/5  TensorRT $TRT_VERSION  (the version SONIC requires on x86_64)"
# Drop a stale libnvinfer-safe-headers-dev first: it exists only at >=10.16, so a
# leftover copy would put 10.16 headers on the include path beside 10.13's.
if dpkg -l libnvinfer-safe-headers-dev 2>/dev/null | grep -q '^ii'; then
  run sudo apt-get remove -y libnvinfer-safe-headers-dev
fi
run sudo apt-get install -y --allow-downgrades \
  "libnvinfer10=$TRT_VERSION" "libnvinfer-plugin10=$TRT_VERSION" \
  "libnvonnxparsers10=$TRT_VERSION" \
  "libnvinfer-headers-dev=$TRT_VERSION" "libnvinfer-headers-plugin-dev=$TRT_VERSION" \
  "libnvinfer-dev=$TRT_VERSION" "libnvinfer-plugin-dev=$TRT_VERSION" \
  "libnvonnxparsers-dev=$TRT_VERSION"

# A serialized TensorRT engine is tied to the version that built it. After a
# version change the cached engines beside the policies are stale: TensorRT will
# refuse to deserialize them, and the failure surfaces at robot-startup time,
# which is the worst possible moment. Drop them here so the next run rebuilds.
if [ "$DRY" -eq 0 ] && ls "$HERE"/policies/*.trt >/dev/null 2>&1; then
  if [ -f /usr/include/x86_64-linux-gnu/NvInferVersion.h ]; then
    _trt_now=$(grep -E 'define TRT_(MAJOR|MINOR)_ENTERPRISE' \
      /usr/include/x86_64-linux-gnu/NvInferVersion.h | awk '{printf "%s.", $3}' | sed 's/\.$//')
    _stamp="$HERE/policies/.trt_built_with"
    if [ ! -f "$_stamp" ] || [ "$(cat "$_stamp")" != "$_trt_now" ]; then
      echo
      echo "  TensorRT is now $_trt_now; removing engines cached under a different version:"
      for e in "$HERE"/policies/*.trt; do echo "    $(basename "$e")"; rm -f "$e"; done
      echo "$_trt_now" > "$_stamp"
    fi
  fi
fi

[ "$DRY" -eq 1 ] && { echo; echo "dry run only."; exit 0; }

# ---------------------------------------------------------------- verify --
say "verify"
# Resolve the invoking user's home: under `sudo bash setup.sh` $HOME is root's
# and the no-sudo half would look missing when it is installed.
_home="$HOME"
[ -n "${SUDO_USER:-}" ] && _home=$(getent passwd "$SUDO_USER" | cut -d: -f6)
export PATH="$_home/.local/bin:$PATH"
ok=1
chk() { printf "  %-24s " "$1"; shift; if "$@" >/dev/null 2>&1; then echo OK; else echo MISSING; ok=0; fi; }
chk just             command -v just
chk cmake            command -v cmake
chk clang            command -v clang
chk cuda_runtime.h   bash -c 'ls /usr/local/cuda-12.9/include/cuda_runtime.h'
chk crt/host_defines bash -c 'ls /usr/local/cuda-12.9/targets/*/include/crt/host_defines.h'
chk NvInfer.h        test -f /usr/include/x86_64-linux-gnu/NvInfer.h
chk msgpack.hpp      test -f /usr/include/msgpack.hpp
chk zmq.h            test -f /usr/include/zmq.h
chk zmq.hpp          test -f "$_home/opt/sonic-deploy-toolchain/include/zmq.hpp"
chk Eigen            bash -c 'ls -d /usr/include/eigen3/Eigen'
chk nlohmann/json    test -f /usr/include/nlohmann/json.hpp
chk "python venv"      test -x "$HERE/.venv/bin/python"
chk "sim venv"         test -x "$HERE/.venv-sim/bin/python"
chk "unitree_sdk2py"   test -d "$HERE/sdk/unitree_sdk2py"
chk "DDS sim"          test -f "$HERE/sim/run_robot_sim.py"
chk onnxruntime      bash -c "ls -d $_home/opt/sonic-deploy-toolchain/onnxruntime-linux-x64-gpu_cuda12-*/lib/cmake/onnxruntime"
if [ -f /usr/include/x86_64-linux-gnu/NvInferVersion.h ]; then
  # NV_TENSORRT_MAJOR is #defined as TRT_MAJOR_ENTERPRISE, so read the numeric
  # TRT_* defines rather than the NV_* indirection.
  trt=$(grep -E 'define TRT_(MAJOR|MINOR|PATCH)_ENTERPRISE' \
        /usr/include/x86_64-linux-gnu/NvInferVersion.h | awk '{printf "%s.", $3}' | sed 's/\.$//')
  printf "  %-24s %s\n" "TensorRT" "$trt"
  # 10.13 exactly, not "the 10.x line". SONIC's deploy guide makes this a safety
  # requirement, not a compatibility one -- see the header comment.
  case "$trt" in
    ${TRT_LINE}.*) ;;
    *) echo "  ⚠ TensorRT $trt is NOT the $TRT_LINE line SONIC requires on x86_64."
       echo "    A different version is documented to produce wrong inference."
       echo "    Re-run:  TRT_VERSION=$TRT_VERSION bash $0"
       ok=0 ;;
  esac
fi

echo
[ "$ok" -eq 1 ] || { echo "Some components are missing; fix them before building."; exit 1; }
cat <<EOF
Toolchain ready. Next:

    source env.sh          # every new shell
    bash build.sh          # compile the runner (first build takes a few minutes)
    bash test.sh           # verify the bundle end to end
    bash run.sh --help     # deploy

Read docs/DEPLOY_G1.md before running on a robot. In particular section 7:
these policies cannot perceive their own horizontal position.
EOF
