# Your four-gripper spider walking on a truss beside the ISS

Video: `spider_grasp_walk_iss.mp4` (24 seconds, 1280 × 720, 30 fps).

This uses your actual prior `spider_mujoco_only` research model, not the simple ROS quadruped used in the earlier posed video. Original files in your other project are unchanged.

## Physics

A new 36-second simulation is integrated by MuJoCo RK4 at 1 ms. The 18 robot coordinates and 16 FEM modal coordinates evolve dynamically. Twelve torque-limited motors drive the gait. Each of four grippers transfers spring-dashpot attachment forces to the robot and equal/opposite generalized loads to the flexible truss. The supervisor releases one gripper at a time and gates recapture by distance and relative speed. The renderer never advances robot positions along an invented trajectory.

The video shows simulation seconds 12–33 at real time after the establishing shot; the initial wide hold keeps simulation time at 12 s. Green jaws indicate an active compliant attachment; orange jaws indicate a released leg. The inset shows gripper 1 and its computed attachment force. Jaw opening is a visualization of the attachment state, not a separately simulated finger actuator.

Important scope: attachment is a compliant bond model. Individual jaw/rail contact, frictional slip, finger actuation, breakage and hardware grasp strength are not resolved. The 6 m truss is a linear modal FEM with a fixed root. The ISS meshes and mounting pedestal are visual context, not a dynamically recoiling spacecraft or a verified ISS mounting interface. The placement is illustrative. This is numerical simulation, not experimental validation or flight-qualified behavior.

The truss deformation displayed is reconstructed from the integrated FEM coordinates at actual scale. The robot dimensions and gait are preserved from your research project.

## Reproduce

From the parent `canadarm-capture` directory:

```bash
OPENBLAS_NUM_THREADS=1 .venv/bin/python spider_truss_iss/physics/src/run.py --duration 36 --step .001 --name iss_walk
.venv/bin/python scripts/check_truss_run.py
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/render_truss_iss.py
```

`physics_checks.json` records displacement, grip transitions, motor/attachment loads and the RK4-stage work/energy audit. `physics/results/iss_walk.npz` contains the freshly integrated states and work. `iss_truss_scene.xml` is the combined rendering model; its pose is supplied from saved dynamic states.

The Earth background uses NASA's geographic Blue Marble and cloud mosaics, projected at an illustrative 420 km altitude. It is not live imagery. NASA/CSA sources and original model credits remain in the parent project and `spider_iss/README.md`.

## Full video with fixed side view

`spider_iss_full_sideview.mp4` is the revised 136-second video:

- 0–12 s: full ISS view, approach and different oblique views of the mounting location.
- 12–132 s: the complete saved 120-second physics run, at normal speed, with a fixed side camera. No camera tracking, zoom, cut or moving inset during the walk. The whole 6 m walking truss remains visible.
- 132–136 s: final hold.

The root now joins the existing ISS beam at world (-29, 8, -1.559 m) through a flange and two braces. This is an illustrative integrated structural extension, not an assertion that this hardware exists on the real ISS. The root boundary condition remains fixed, preserving the research physics. Other ISS geometry remains visual context.

The full trajectory is copied from the original validated standalone MuJoCo research run; its first 36 seconds match the fresh run in this project exactly in body position. See `full_video_checks.json` and `full_mission_validation.json`. The original trajectory is stored locally as `physics/results/full_mission.npz`.

Regenerate:

```bash
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/render_truss_iss_full.py
```

`iss_integrated_truss_scene.xml` contains the revised rendering scene. The earlier short movie and its scene are retained separately.
