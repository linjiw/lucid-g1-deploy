#!/usr/bin/env bash
# Launch a policy from this bundle.  source env.sh first.
#
#   bash run.sh --policy deploy_dr --iface lo   --sim     bench on loopback
#   bash run.sh --policy deploy_dr --iface eth0           on the robot network
#   bash run.sh --list                                    show what is available
#
# --sim adds --disable-crc-check, which is what deploy.sh sim does.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
POLICY=deploy_dr; IFACE=""; SIM=0; EXTRA=()
while [ $# -gt 0 ]; do case "$1" in
  --policy) POLICY="$2"; shift 2 ;;
  --iface)  IFACE="$2";  shift 2 ;;
  --sim)    SIM=1; shift ;;
  --list)
    echo "policies:"; for p in "$HERE"/policies/*.onnx; do echo "  $(basename "$p" _s8600_g1.onnx)"; done
    echo "motions:";  for m in "$HERE"/motions/*/; do echo "  $(basename "$m")"; done
    exit 0 ;;
  --help|-h) sed -n '2,10p' "$0"; exit 0 ;;
  *) EXTRA+=("$1"); shift ;;
esac; done

RUNNER="$HERE/runner/target/release/g1_deploy_onnx_ref"
[ -x "$RUNNER" ] || { echo "runner not built. Run: bash build.sh"; exit 1; }
ONNX="$HERE/policies/${POLICY}_s8600_g1.onnx"
[ -f "$ONNX" ] || { echo "no such policy: $POLICY  (try --list)"; exit 1; }

if [ -z "$IFACE" ]; then
  IFACE=$(ip -4 addr show 2>/dev/null | awk '/^[0-9]+:/{gsub(/:$/,"",$2);i=$2}/inet 192\.168\.123\./{print i;exit}')
  [ -n "$IFACE" ] && echo "auto-detected robot interface: $IFACE" \
    || { echo "no 192.168.123.x interface found; pass --iface (use 'lo' for a bench run)"; exit 1; }
fi
[ "$SIM" -eq 1 ] && EXTRA+=(--disable-crc-check)

echo "policy   $ONNX"
echo "motions  $HERE/motions/"
echo "iface    $IFACE"
[ "$SIM" -eq 1 ] && echo "mode     SIMULATION (CRC check disabled)" || {
  echo "mode     REAL ROBOT"
  echo
  echo "Before you continue: is the emergency stop within reach, the robot on a"
  echo "harness or gantry, and a fallback controller ready? None of those exist"
  echo "in this software. See docs/DEPLOY_G1.md section 8."
  read -r -p "Proceed? [y/N] " a; [ "$a" = y ] || [ "$a" = Y ] || { echo "cancelled."; exit 0; }
}
exec "$RUNNER" "$IFACE" "$ONNX" "$HERE/motions/" \
  --obs-config "$HERE/config/observation_config_lucid_g1_1570.yaml" "${EXTRA[@]}"
