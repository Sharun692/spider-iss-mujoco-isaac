#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
isaac_root="${ISAAC_SIM_ROOT:-$HOME/simulators/isaac}"
action="${1:-preview}"
if (($#)); then shift; fi
case "$action" in
 simulate) entry="$project_root/src/run_physx.py"; defaults=(--duration 120 --step .001 --name physx_full);;
 preview) entry="$project_root/src/render_rtx.py"; defaults=(--preview --trajectory physx_full);;
 check) entry="$project_root/src/check_dynamics.py"; defaults=();;
 render) entry="$project_root/src/render_rtx.py"; defaults=(--trajectory physx_full);;
 *) echo 'Usage: launch.sh {simulate|preview|render|check} [extra options]' >&2; exit 2;;
esac
exec env -u CONDA_PREFIX -u PYTHONHOME OPENBLAS_NUM_THREADS=1 "$isaac_root/python.sh" "$entry" "${defaults[@]}" "$@"
