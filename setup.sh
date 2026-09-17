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
#   cppzmq vendored             NOT because the distro lacks it. libzmq3-dev --
#                               installed below -- does ship /usr/include/zmq.hpp
#                               (`dpkg -S /usr/include/zmq.hpp`, jammy 4.3.4-2),
#                               but that copy is cppzmq 4.8.1. The vendored
#                               single header is 4.10.0, and env.sh's
#                               CPLUS_INCLUDE_PATH export puts it ahead of
#                               /usr/include, so any build with env.sh sourced
#                               compiles against 4.10.0. That export is
#                               conditional on the vendored header being
#                               present, so a build without it falls back to the
#                               distro's 4.8.1. libzmq3-dev is still what
#                               provides the C library underneath.
set -euo pipefail

DRY=0; NO_SUDO=0
for a in "$@"; do case "$a" in
  --dry-run) DRY=1 ;; --no-sudo) NO_SUDO=1 ;;
  *) echo "unknown option: $a"; exit 2 ;;
esac; done

# Refuse `sudo bash setup.sh`. TC below is derived from $HOME, and under sudo
# $HOME is root's: the entire no-sudo half (cmake, onnxruntime, zmq.hpp) lands
# in /root/opt, while verify at the bottom resolves the INVOKING user's home
# through SUDO_USER and reports zmq.hpp and onnxruntime MISSING -- correctly,
# they are not in that user's home -- with nothing saying why. This script calls
# sudo itself, per apt command; it must not be started under one.
if [ "$(id -u)" -eq 0 ] && [ -n "${SUDO_USER:-}" ]; then
  echo "Do not run this under sudo: \$HOME would be /root, so cmake,"
  echo "onnxruntime and zmq.hpp would install into /root/opt and the verify"
  echo "step -- which looks in ${SUDO_USER}'s home -- would report them MISSING."
  echo
  echo "Run:  bash setup.sh      It asks for sudo itself, for the apt steps."
  exit 2
fi

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

# Step 1 below downloads four things with curl (just, cmake, onnxruntime,
# zmq.hpp) BEFORE step 2 apt-installs curl and ca-certificates. On a minimal
# image -- a docker base, a cloud image -- curl is absent and `set -e` kills the
# script at the first download with a bare "curl: command not found". Installing
# curl up here instead is not an option: --no-sudo is defined by reaching no
# sudo call at all, so the guard has to refuse rather than fix.
command -v curl >/dev/null || {
  echo "curl is missing, and setup.sh downloads four things before apt runs."
  echo "Install it first:  sudo apt-get install -y curl ca-certificates"; exit 1; }

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
# ffmpeg is here for the recorders, not for the runner: sim/run_robot_sim.py
# --record pipes raw RGB frames straight into ffmpeg's stdin, and tools/ shells
# out to it for the demo video and the MuJoCo story clips. run_robot_sim.py and
# build_demo_video.py test shutil.which("ffmpeg") up front and fail before doing
# any work; mujoco_story.py's ff() has no such guard and would raise
# FileNotFoundError partway through a render. Either way this is the step that
# should have put ffmpeg on the box.
run sudo apt-get install -y \
  clang build-essential pkg-config patchelf zlib1g-dev libgtest-dev \
  git git-lfs curl wget ca-certificates \
  libmsgpack-dev libzmq3-dev libeigen3-dev nlohmann-json3-dev \
  ffmpeg \
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
# The rebuild is gated on the IMPORTS working, not on bin/python existing.
# `python3 -m venv` writes bin/python before pip runs, so a failed pip install
# leaves a venv that looks installed; the old `[ ! -x "$VENV/bin/python" ]` guard
# was then false on every later run, the rm -rf was never reached, and the remedy
# this script printed -- re-run setup.sh -- took the skip branch and could never
# repair it. verify below runs the same import, so a venv that exists but cannot
# import no longer reaches "Toolchain ready". The cost of gating on the import is
# that a venv failing it for an environmental reason -- a system library missing
# under mujoco, say -- is now torn down and reinstalled on every later run and
# still fails; the pip output and the import error are the thing to read, not a
# re-run.
VENV_IMPORTS='import numpy, onnxruntime, yaml, joblib, scipy, mujoco'
venv_ok() { [ -x "$VENV/bin/python" ] && "$VENV/bin/python" -c "$VENV_IMPORTS" >/dev/null 2>&1; }
if [ "$DRY" -eq 0 ] && ! venv_ok; then
  rm -rf "$VENV"
  python3 -m venv "$VENV"
  "$VENV/bin/pip" -q install --upgrade pip
  "$VENV/bin/pip" -q install numpy onnxruntime pyyaml joblib scipy mujoco
fi
if [ "$DRY" -eq 0 ]; then
  if venv_ok; then
    "$VENV/bin/python" - <<'PYCHK'
import numpy, onnxruntime, mujoco
print(f"  numpy {numpy.__version__} · onnxruntime {onnxruntime.__version__} · mujoco {mujoco.__version__}")
PYCHK
  else
    echo "  ⚠ .venv cannot import numpy/onnxruntime/yaml/joblib/scipy/mujoco even"
    echo "    after a fresh create+install. pip itself succeeded -- set -e would have"
    echo "    stopped the script otherwise -- so this is an import-time failure, not a"
    echo "    download one, and re-running setup.sh will reproduce it. See it with:"
    echo "      .venv/bin/python -c 'import numpy, onnxruntime, yaml, joblib, scipy, mujoco'"
  fi
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
# Gated on the imports for the same reason as .venv above, and the need is worse
# here: the cyclonedds install below ends in `|| echo`, which swallows the
# failure under set -e. A failed build therefore left a .venv-sim that existed,
# failed its import check, and -- because the guard was `[ ! -x bin/python ]` --
# was never rebuilt on any later run, while the message below said drill.sh will
# not run and offered no way out.
SIM_IMPORTS='import mujoco
from cyclonedds.domain import DomainParticipant
from unitree_sdk2py.core.channel import ChannelFactory
from gear_sonic_sim.simulator_factory import SimulatorFactory'
venvsim_ok() { [ -x "$VENVSIM/bin/python" ] \
  && PYTHONPATH="$HERE/sim:$HERE/sdk" "$VENVSIM/bin/python" -c "$SIM_IMPORTS" >/dev/null 2>&1; }
if [ "$DRY" -eq 0 ] && ! venvsim_ok; then
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
if [ "$DRY" -eq 0 ] && [ -x "$VENVSIM/bin/python" ]; then
  if venvsim_ok; then
    PYTHONPATH="$HERE/sim:$HERE/sdk" "$VENVSIM/bin/python" - <<'PYCHK'
import mujoco
print(f"  sim venv     python OK, mujoco {mujoco.__version__}, cyclonedds 0.10.2, unitree_sdk2py")
PYCHK
  else
    echo "  ⚠ the DDS simulator is not importable; drill.sh will not run."
    echo "    Re-running setup.sh now deletes .venv-sim and rebuilds it. cyclonedds"
    echo "    0.10.2 builds against CYCLONEDDS_HOME=$CDDS and needs python3.10-3.12."
  fi
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
#
# The stamp is written whether or not an engine exists. It used to be written
# only inside the "engines are stale" branch, which on a fresh machine never runs
# -- setup.sh installs TensorRT before the first engine is ever built -- and
# .gitignore:10 keeps the file out of the clone, so setup.sh is its only possible
# writer. The stamp was therefore permanently absent on a new box, so the
# `# tensorrt` line run.sh writes into every run_info.txt -- `cat
# policies/.trt_built_with ... || echo unknown` -- read `unknown` for exactly the
# machine a future operator would be deploying from. Writing the stamp
# unconditionally stays honest: any engine built later is built by the TensorRT
# recorded here, and if TensorRT changes, the branch below deletes the engines
# that no longer match it.
if [ "$DRY" -eq 0 ] && [ -f /usr/include/x86_64-linux-gnu/NvInferVersion.h ]; then
  _trt_now=$(grep -E 'define TRT_(MAJOR|MINOR)_ENTERPRISE' \
    /usr/include/x86_64-linux-gnu/NvInferVersion.h | awk '{printf "%s.", $3}' | sed 's/\.$//')
  _stamp="$HERE/policies/.trt_built_with"
  if ls "$HERE"/policies/*.trt >/dev/null 2>&1 \
     && { [ ! -f "$_stamp" ] || [ "$(cat "$_stamp")" != "$_trt_now" ]; }; then
    echo
    echo "  TensorRT is now $_trt_now; removing engines cached under a different version:"
    for e in "$HERE"/policies/*.trt; do echo "    $(basename "$e")"; rm -f "$e"; done
  fi
  echo "$_trt_now" > "$_stamp"
fi

[ "$DRY" -eq 1 ] && { echo; echo "dry run only."; exit 0; }

# ---------------------------------------------------------------- verify --
say "verify"
# Resolve the invoking user's home, because the no-sudo half installs under
# $HOME. The `sudo bash setup.sh` case this used to compensate for is refused
# outright at the top of the script now: under sudo the files land in /root/opt,
# so they really are absent from the user's home, and looking them up elsewhere
# would have reported OK over a toolchain installed where nothing else can find
# it. What remains here covers a SUDO_USER exported by an earlier sudo shell.
_home="$HOME"
[ -n "${SUDO_USER:-}" ] && _home=$(getent passwd "$SUDO_USER" | cut -d: -f6)
export PATH="$_home/.local/bin:$PATH"
ok=1
chk() { printf "  %-24s " "$1"; shift; if "$@" >/dev/null 2>&1; then echo OK; else echo MISSING; ok=0; fi; }
chk just             command -v just
chk cmake            command -v cmake
chk clang            command -v clang
chk ffmpeg           command -v ffmpeg
chk cuda_runtime.h   bash -c 'ls /usr/local/cuda-12.9/include/cuda_runtime.h'
chk crt/host_defines bash -c 'ls /usr/local/cuda-12.9/targets/*/include/crt/host_defines.h'
chk NvInfer.h        test -f /usr/include/x86_64-linux-gnu/NvInfer.h
chk msgpack.hpp      test -f /usr/include/msgpack.hpp
chk zmq.h            test -f /usr/include/zmq.h
chk zmq.hpp          test -f "$_home/opt/sonic-deploy-toolchain/include/zmq.hpp"
chk Eigen            bash -c 'ls -d /usr/include/eigen3/Eigen'
chk nlohmann/json    test -f /usr/include/nlohmann/json.hpp
# These two run the imports rather than `test -x bin/python`: python3 -m venv
# writes bin/python before pip runs, so existence proves nothing, and this is
# what used to let "Toolchain ready" print over a venv that could not import
# numpy (tools/, test.sh) or cyclonedds (drill.sh, run.sh's rehearsal).
chk "python venv"      venv_ok
chk "sim venv"         venvsim_ok
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

# /usr/local/cuda is an update-alternatives symlink, and a CUDA 13 toolkit
# installed beside 12.9 normally owns it (on this box: priority 131 vs 129,
# link -> /usr/local/cuda-13.1). That matters because find_package(CUDAToolkit
# 10.2 QUIET) at runner/CMakeLists.txt:42 locates the toolkit through nvcc, and
# step 4/5 above installs only the 12.9 runtime and crt headers -- no nvcc, by
# design, since nothing here compiles .cu files. On a mixed box the search comes
# up empty and runner/CMakeLists.txt:144-176 falls through to hardcoded
# /usr/local/cuda paths: CUDA 13 libraries under a TensorRT built for 12.9.
# A warning, not a failure -- this cannot tell whether the binary is actually
# wrong, and the ldd check below can.
if [ -e /usr/local/cuda ]; then
  _cudalink=$(readlink -f /usr/local/cuda)
  if [ "$_cudalink" != /usr/local/cuda-12.9 ]; then
    echo "  ⚠ /usr/local/cuda -> $_cudalink, not /usr/local/cuda-12.9."
    echo "    The runner's CMake fallback hardcodes /usr/local/cuda, so the build can"
    echo "    link against that toolkit instead. Install cuda-nvcc-12-9 as well, then"
    echo "    confirm after building with:"
    echo "      ldd runner/target/release/g1_deploy_onnx_ref | grep cudart   # must say .so.12"
  fi
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
