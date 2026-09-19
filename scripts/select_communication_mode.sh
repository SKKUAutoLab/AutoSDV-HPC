#!/bin/bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="$ROOT/src/launch_pkg${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -m launch_pkg.communication_mode "$@"
