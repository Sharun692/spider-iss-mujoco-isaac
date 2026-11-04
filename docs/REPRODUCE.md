# Reproducing the recorded experiments

Run commands from the repository root unless a `cd` is shown. Existing recorded results and movies are already included. Re-running with the same output name overwrites the local result; use a new name when retaining the archived data matters. No installation is performed automatically by this repository.

## MuJoCo

Use an isolated Python environment. The original project specifies MuJoCo >=3.15; the supplied requirements are not a fully frozen environment lock. `environment-export.json` records available package versions from the export machine where obtainable.

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-mujoco.txt
OPENBLAS_NUM_THREADS=1 .venv/bin/python outputs/canadarm-capture/spider_truss_iss/physics/src/run.py --duration 120 --step .001 --name rerun_full
```

The freshly integrated result is `outputs/canadarm-capture/spider_truss_iss/physics/results/rerun_full.npz`. The original full movie renderer reads `full_mission.npz`, which is already provided. To render a new run, deliberately copy/rename the chosen result to that filename after retaining a backup, or edit the renderer's `trajectory` variable.

To render the archived full run:

```bash
OPENBLAS_NUM_THREADS=1 .venv/bin/python outputs/canadarm-capture/scripts/render_truss_iss_full.py
```

Add `--preview` for five sample images. EGL is selected by the renderer by default. FFmpeg with libx264 and DejaVu Sans fonts must be available on the host. The required station meshes and NASA Earth textures are included. A working graphics driver is needed for rendering; ordinary dynamics/checks do not need the Earth/ISS render assets.

The existing short-run checker reads the included 36 s `iss_walk.npz`:

```bash
.venv/bin/python outputs/canadarm-capture/scripts/check_truss_run.py
```

## Isaac Sim

Use the separately installed Isaac Sim 5.1 distribution and its bundled Python. Do not install Isaac into the MuJoCo environment. Set the installation location explicitly if it is not `$HOME/simulators/isaac`:

```bash
export ISAAC_SIM_ROOT=/path/to/isaac-sim
```

On the original machine, a missing `scipy.io.matlab` module in bundled SciPy was repaired with SciPy 1.15.3 in a project-local directory. Check whether your installation needs this before installing it:

```bash
"$ISAAC_SIM_ROOT/python.sh" -c "import scipy.io.matlab; print('SciPy import OK')"
```

If the same failure occurs, the repair command is:

```bash
"$ISAAC_SIM_ROOT/python.sh" -m pip install --no-deps --target outputs/isaac_spider_iss/python_deps scipy==1.15.3
```

The supplied `imported_spider.usda` is self-contained for the articulated robot. Reconstruction of the bridge XML is a pure-Python step; it writes a separate file so the recorded input remains intact:

```bash
python3 outputs/isaac_spider_iss/src/prepare_bridge.py
```

To regenerate the native robot USD from the archived bridge input (overwrites `imported_spider.usda`):

```bash
"$ISAAC_SIM_ROOT/python.sh" outputs/isaac_spider_iss/src/import_robot.py
```

The new importer entry point packages the original tested import configuration. It was syntax-checked for this export; the full GPU import was not repeated simply to package the repository. The archived USD and full run already exist.

Use the launcher for the maintained simulator:

```bash
bash outputs/isaac_spider_iss/launch.sh simulate --name physx_rerun
bash outputs/isaac_spider_iss/launch.sh check physx_rerun
bash outputs/isaac_spider_iss/launch.sh preview --trajectory physx_rerun
bash outputs/isaac_spider_iss/launch.sh render --trajectory physx_rerun
```

With no `--trajectory`, preview/render use the archived `physx_full` data. Add `--gui` to preview/render to show the Isaac window. These jobs close Isaac on completion. The final renderer overwrites `results/isaac_iss_full_sideview.mp4`; keep a copy of the archived movie if comparing renders.

The renderer regenerates `iss_spider_scene.usda` using texture paths on the current machine. That exported USD is a visual scene; opening it by itself does not execute the custom gait/force controller. Use `launch.sh simulate` to run the physics program. The scene's intended camera is `/World/Camera`.

To inspect the recorded numerical checks without rerunning GPU physics, a Python environment with NumPy can run:

```bash
.venv/bin/python outputs/isaac_spider_iss/src/check_dynamics.py physx_full
.venv/bin/python outputs/isaac_spider_iss/src/check_dynamics.py physx_fullrate_audit
```

These commands recalculate and overwrite their JSON/energy diagnostic outputs. A full-rate pilot can be generated with:

```bash
bash outputs/isaac_spider_iss/launch.sh simulate --duration 3 --step .001 --sample-period .001 --name audit_rerun
bash outputs/isaac_spider_iss/launch.sh check audit_rerun
```

## Reproducing the exact archived full-run implementation

`source_archive/full_run_script_used.py` is the exact dynamics source used for the delivered full PhysX run. Later `src/run_physx.py` changes added diagnostics and modified sleep/stabilization settings. For an archival implementation rerun, copy the archive into `src/` under a distinct temporary filename (its imports require that location), run it with the bundled Python and `--duration 120 --step .001 --name archival_rerun`, then remove that temporary source copy. Avoid replacing the maintained script. Bitwise reproducibility across different drivers/GPUs/software versions is not promised.

The delivered video used `source_archive/scene_rtx_before_instancing.py`. The maintained scene builder uses point instancing and has the same intended scene geometry/camera sequence. The original video is retained for direct comparison. Rendering and dynamics are independent jobs; changing lighting does not regenerate physical motion.

## Export checks

`docs/export_checks.json` records packaging checks run on 4 November 2026: Python syntax, bridge structural equality, parameter equality, and preserved artifact hashes. The full 120 s GPU simulation was not rerun for this upload. Numerical validation reports describe the original runs, with their limitations explained in `METHODS.md`.
