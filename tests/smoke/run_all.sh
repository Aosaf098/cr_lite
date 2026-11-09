#!/usr/bin/env bash
# Run every environment smoke test. Exit code is non-zero if any fails.
# Usage (from the repo root, after `source env.sh`):  tests/smoke/run_all.sh
# Optionally pass a different Python:  PYTHON=/path/to/python tests/smoke/run_all.sh
set -uo pipefail
PY="${PYTHON:-python}"
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
status=0
for t in smoke_mujoco.py smoke_open3d.py smoke_osqp.py; do
  echo "=== $t ==="
  "$PY" "$DIR/$t" 2>&1 | grep -v "^ERROR in" | tee /tmp/cr_smoke_last.txt
  grep -q "SMOKE TEST: PASSED" /tmp/cr_smoke_last.txt || status=1
  echo
done
[ $status -eq 0 ] && echo "ALL SMOKE TESTS PASSED" || echo "SOME SMOKE TESTS FAILED"
exit $status
