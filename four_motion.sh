#!/usr/bin/env bash
# Latest walking / turning / crouch-hold / side-step policies, simulation only.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/env.sh"
exec "$PYTHON" "$HERE/tools/four_motion.py" "$@"
