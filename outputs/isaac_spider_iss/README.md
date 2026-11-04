# Four-gripper spider on the ISS — Isaac Sim / GPU PhysX / RTX

This project ports the existing robot, 6 m flexible-truss model and camera sequence to the installed Isaac Sim 5.1. It contains an independent GPU PhysX simulation and a separate RTX rendering pipeline. The dynamics program never reads MuJoCo trajectories and never runs MuJoCo.

## Outputs

- `results/isaac_iss_full_sideview.mp4`: full RTX movie, 136 seconds at 1280 × 720 and 30 fps.
- `results/physx_full.npz` and `.json`: independently integrated full-mission states and grip events.
- `iss_spider_scene.usda`: native USD visual scene, including original NASA ISS geometry, photographic Earth, robot, truss and camera. This is the rendering scene; opening it alone does not run the custom physics controller.
- `imported_spider.usda`: intermediate native PhysX robot asset. The runtime script adds the correct fixed-base modal articulations and custom coupling.
- `assets/parameters.json`, `geometry.json`, `data_provenance.json`: existing model coefficients, geometry, units and provenance.
- `ASSUMPTIONS.md`: physics scope and numerical qualifications.

## Run

The existing Isaac installation is `/home/r82367sa/simulators/isaac`. `launch.sh` uses its bundled Python. Set `ISAAC_SIM_ROOT` only if the installation moves.

```bash
./launch.sh simulate
./launch.sh preview
./launch.sh render
./launch.sh check physx_full
```

These are long calculations on this laptop. The physics and rendering programs close Isaac when their job finishes. To show the rendering window, add `--gui` to the preview or render command.

The approved SciPy 1.15.3 repair is installed only in `python_deps/`; scripts prepend that directory before loading Isaac. No global Python installation, ROS installation, MATLAB installation, driver or shell startup file was modified.

For an energy check logged at the actual solver rate:

```bash
./launch.sh simulate --duration 3 --sample-period .001 --name audit
./launch.sh check audit
```

The audit itself uses source geometry, inertias, NumPy calculations and the recorded PhysX states. It does not call MuJoCo. The native USD visual scene uses `/World/Camera`; select that camera if opening the file directly in Isaac.

## Motion and physics

PhysX integrates the native floating-base robot and 16 mass-normalised modal coordinates on CUDA device 0. Twelve explicitly torque-limited motors execute the existing gait. Grasp forces are applied to each foot and their equal/opposite generalized loads to the truss. The modal springs are implicit PhysX drives, with the original cross-modal damping and orbital terms supplied by the controller.

The native MJCF importer cannot represent six root joints on one body, so the port uses a floating base. Two-joint shoulders are split into separate yaw and shoulder links. The four explicit yaw carriers add 0.0001 kg each and 1e-6 kg m² diagonal inertia as numerical regularization. All original link masses and inertias remain recorded.

The short fine-step comparison uses 0.5 ms / 64 position iterations and the practical run uses 1 ms / 32 position iterations, with 4 velocity iterations in the practical configuration. The faster 2 ms candidate was rejected because it saturated the motor effort and deviated more from the fine-step result.

The render program reads actual saved PhysX link poses and modal states. It does not use the old MuJoCo movie or its recorded poses. The opening pans use the same camera keys as the previous full movie. After 12 seconds, camera position, orientation and focal length are fixed while the entire 120-second simulation plays at normal speed, followed by a four-second hold.

The truss is joined visually to the existing ISS beam through the same flange and braces. Its root is physically fixed in the research model. The rest of the ISS is visual context, not a recoiling spacecraft or a certified attachment design.

## What is and is not validated

The native robot positions and foot velocities agree with independent kinematics from the source geometry to sub-micrometre and sub-micrometre-per-second precision in the pilots. Native mass-matrix kinetic energy agrees with the independent inertia calculation to within a microjoule.

The full-rate pilot's energy check uses midpoint work for torque commands held over each physics step. Treating those commands as continuously changing gave a misleading residual; the corrected pilot residual is about 1.7% of its mechanical-energy range. The 100 Hz full-mission log is not a complete integration-rate energy audit. Pilot checks are not a full-mission energy certificate or experimental validation.

**Grippers remain the existing assumed compliant attachment model. Individual jaw contact/friction, finger actuator dynamics, grasp failure strength, nonlinear flexible-member collision and spacecraft recoil are not implemented.** Displayed jaw opening/colour indicates the actual attachment flag. It is not evidence that a resolved force-closure grasp has been simulated. Existing values are labelled assumptions, as requested; no hardware measurements or friction values have been invented.

## Graphics and hardware

RTX real-time ray-traced lighting, PBR materials, reflections and shadows are rendered on the RTX GPU. The maintained renderer uses GPU instances for the deforming truss members. The delivered movie uses the equivalent individual capsule geometry preserved in `source_archive/scene_rtx_before_instancing.py`; instancing was separately checked in the RTX preview. The ISS mesh vertices and faces are reused without geometric simplification. Earth uses NASA's Blue Marble and companion cloud imagery; its illustrative 420 km geometry is visual only, not a live orbital ephemeris or atmospheric scattering model.

The local RTX 1000 Ada laptop GPU has 6 GB VRAM. The installed driver is 550.163.01. This particular scene has been tested locally; the configuration is below NVIDIA's published Isaac 5.1 hardware/driver recommendations and is not a promise that maximum settings or larger scenes will work. GPU dynamics on one small scene is not necessarily faster than CPU simulation.

## Credits

ISS: NASA 3D Resources, International Space Station (B), originally fetched by `canadarm-capture`.

Earth imagery: NASA Goddard Space Flight Center, Reto Stöckli and Robert Simmon. The land/cloud composite combines the original NASA geographic texture and cloud mosaic; it is not AI-generated imagery.

The original `canadarm-capture` project and NASA/CSA asset terms remain applicable. The user's existing spider research project supplies the robot, gait and truss coefficients. `source_archive/full_run_script_used.py` preserves the exact dynamics script used for the first full mission; the maintained launcher includes additional logging and diagnostic improvements.
