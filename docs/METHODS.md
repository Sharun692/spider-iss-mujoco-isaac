# Recreating the MuJoCo ISS spider experiment in Isaac Sim

Prepared for supervisor review, 4 November 2026. This document describes the October 2026 walking-on-truss demonstration in this repository. It does not describe the separate, later modular-assembly project.

## Research claim and scope

The Isaac movie was produced by transferring the existing robot geometry, inertial parameters, gait, compliant attachment law, reduced structural model and camera design into an Isaac Sim implementation. GPU PhysX integrated a new trajectory. Isaac RTX then rendered that saved PhysX trajectory with the NASA ISS and Earth assets.

This was a model-and-controller port, followed by independent numerical integration. It was not an import of the MuJoCo video, a playback of MuJoCo joint positions inside PhysX, or a one-click transfer of all MuJoCo semantics. The two engines share intended model ingredients but have different state representations, solver behaviour, control evaluation timing and some implementation approximations. Numerical equivalence has not been established for the complete coupled system.

The grippers are spring–dashpot attachments. Their fingers are visual indicators of attachment state. Frictional jaw contact, force closure, finger actuator mechanics, slip and hardware failure strength were not simulated. The truss uses 16 linear FEM modes with a fixed root. Neither flight qualification nor experimental validation is claimed.

## Source material and what was transferred

The robot and structural coefficients came from the user's existing `spider_mujoco_only` research model. A copy of that model's geometry, parameters and MJCF plant is under `outputs/canadarm-capture/spider_truss_iss/physics/`. The surrounding ISS visualization originated in the `dara-miao/canadarm-capture` project; upstream attribution is retained.

Transferred data include link dimensions, hip locations, link masses and inertia tensors; initial robot posture; joint axes; 12 motor gains and torque limits; gait timing and inverse kinematics; four attachment coefficients and capture thresholds; truss nodes and connectivity; modal shape, stiffness, damping and orbital matrices; and the opening/side-view camera keyframes. The existing research values were accepted as assumptions rather than substituted with invented measurements.

Selected dimensions and coefficients are:

- Body: 0.50 × 0.30 × 0.14 m, mass 47.95 kg.
- Four legs: yaw, shoulder and knee motion; two 0.27 m links per leg, each link 1.776 kg; each gripper 0.4 kg.
- Original robot mass: 63.758 kg, before the Isaac yaw-carrier regularization described below.
- Truss: 6 m long, 0.30 m wide, 96 stored nodes, 288 member edges, and 16 retained modes. Stored interface nodes can be coincident; this is the source modular FEM representation.
- Motor proportional gain: 500 N m/rad; derivative gain: 20 N m s/rad; commanded torque saturation: ±40 N m per motor.
- Attachment stiffness: 20,000 N/m; attachment damping: 160 N s/m.
- Recapture thresholds: position gap ≤0.006 m and relative speed ≤0.16 m/s.
- Nominal body travel: x = 0.65 m to x = 5.35 m; nominal gait-progress duration 96 s; leg-selection period 0.75 s.

The JSON files, rather than the rounded numbers above, are the authoritative numerical inputs. Parameter files are included with both engine implementations. `docs/original_file_manifest.json` records hashes of the original local files selected for this export.

## Original MuJoCo implementation

The MuJoCo plant contains 18 robot generalized coordinates (six base coordinates and twelve motor coordinates) and sixteen modal coordinates: 34 continuous position and velocity coordinates. Twelve actuators receive explicitly computed joint torques. Geometry contact is disabled in this research plant; grasp loads are calculated separately.

`physics/src/run.py` loads `spider_modal_plant.xml`, maps the source roll/pitch/yaw ordering to MuJoCo joint ordering with `PERM`, initializes the state, and registers a control callback. MuJoCo RK4 advances both the robot and modal coordinates at a 1 ms timestep. The controller and force law are evaluated at RK4 stages. The work integral uses the corresponding stage weights 1, 2, 2, 1 divided by six.

The short ISS experiment was freshly integrated for 36 s. The full 120 s trajectory used in the final MuJoCo movie was copied from the existing standalone research run; the local comparison report records a match in body position over the first 36 s. Thus the full MuJoCo movie should not be described as a new 120 s integration performed by its renderer.

## Robot conversion: MJCF to USD and PhysX

The intermediate `bridge_mjcf.xml` adapts the original plant for Isaac's MJCF importer. The import configuration uses `merge_fixed_joints=False`, `import_inertia_tensor=True`, `fix_base=False`, and `distance_scale=1.0`. The imported result is stored as `outputs/isaac_spider_iss/imported_spider.usda`.

The six MuJoCo base joints on one body are replaced by a native floating base. PhysX stores base orientation as a quaternion. At each control evaluation, the implementation converts that quaternion to the source Euler-angle convention and converts angular velocity to Euler rates before calling the existing controller. Quaternion component ordering is handled explicitly: recorded link transforms use XYZW, while the initialization API receives its expected WXYZ identity quaternion. Euler-rate conversion remains subject to the source representation's pitch singularity; no general attitude-envelope validation is claimed.

Each two-joint shoulder construction is split into a yaw carrier followed by the shoulder link. This expresses the separate rotational axes as an articulation chain supported by the importer. Each of the four added yaw carriers has mass 0.0001 kg and diagonal inertia 1e-6 kg m². This adds 0.0004 kg in total and is a numerical regularization, not a measured robot component. Its effect requires a dedicated sensitivity study before claiming exact model equivalence.

PhysX joint and body ordering is not assumed to match MuJoCo ordering. The program maps names such as `yaw_0`, `shoulder_0`, `knee_0` and `grip_0` to native indices. Implicit robot motor gains are set to zero because the source PD controller computes and applies explicit torque commands.

The import transfers the articulated asset, not the custom gait, attachment supervisor, modal coupling or camera rendering logic. Those functions were implemented separately in Python. A portable importer entry point is included as `outputs/isaac_spider_iss/src/import_robot.py`.

## Flexible truss representation

The physical truss is a reduced linear structural model. Nodal displacement is reconstructed from the retained mode shapes:

$$u = \Phi\eta, \qquad x_{nodes}=x_{rest}+\Phi\eta.$$

Here, eta contains 16 modal coordinates and Phi contains the corresponding displacement shapes for the stored nodes. The original mass-normalized formulation has a modal mass matrix equal to the identity. Each modal coordinate is represented in PhysX by a unit-mass prismatic degree of freedom with its own fixed-base articulation. These sliders are numerical coordinates, not literal mechanisms installed on the ISS.

The importer initially supplied modal joints, but a slider attached directly to the world was insufficient for the articulation tensor interface used here. The runtime therefore creates a fixed rigid base for each slider and connects it to the world with a fixed joint. Modal indices are mapped by prim name.

Native PhysX prismatic drives supply the diagonal modal spring stiffness. Python supplies cross-modal damping and the robot/structure coupling. In the adopted sign convention, the implemented modal equation is represented by

$$\ddot\eta + K\eta = -C_m\dot\eta - \sum_j J_{a,j}^{T}F_j + K_{orb}\eta+C_{orb}\dot\eta+F_{orb}.$$

The matrices and mode shapes come from the existing model. No new solid-element mesh or native Isaac soft-body FEM solve was introduced. Nonlinear deformation, member failure and beam-to-jaw collision are outside this formulation.

## Grasp forces, reaction loads and locomotion

For each foot, the rail anchor position is interpolated within its truss bay and displaced by the modal field. Its modal Jacobian is J_a. With foot position p and anchor position a, the attachment force applied to the robot is

$$F_j=w_j\{k_g(a_j-p_j)+d_g(\dot a_j-\dot p_j)\}.$$

The attachment weight w ranges from zero to one during release and recapture. The opposite generalized structural load is `-J_a.T @ F`. Consequently, robot forces excite the flexible truss and the deformed truss changes the anchor positions seen by the robot. This is bidirectional force coupling. It is a three-dimensional translational attachment law; it does not impose a separately modelled jaw orientation constraint or finger contact moment.

The existing supervisor releases one leg only when all four are attached. The swing target follows a smooth trajectory with clearance. Recapture requires both the distance and relative-speed thresholds. If the swinging leg has not recaptured near the end of its allotted phase, gait progress pauses. Attachment loads are ramped after capture. The nominal 96 s gait is followed by the remainder of the 120 s simulated interval for settling/holding; nominal gait progress can also pause when the capture condition demands it.

Inverse kinematics generates joint-angle targets from the desired foot/body geometry. Desired joint velocities are obtained with the source finite-difference procedure. The explicit motor law is

$$\tau=\operatorname{clip}\left[K_p\operatorname{wrap}(q_{ref}-q)+K_d(\dot q_{ref}-\dot q),-40,40\right].$$

The robot base pose is read from the integrator rather than prescribed along the rail during the dynamics run. However, during the later rendering pass the visual bodies are deliberately positioned from the already-computed results. Green/orange jaw colours and the displayed jaw aperture indicate the supervisor attachment flag. They are not independently simulated contact evidence.

## Orbital environment and coupling differences

Uniform terrestrial gravity is set to zero. The source local orbital terms remain as custom forces, using orbit rate approximately 0.0011331559073 rad/s. In the source coordinate convention the body acceleration contribution is

$$a_{orb}=(-2n v_z,\;-n^2 y,\;2n v_x+3n^2 z).$$

This is an illustrative local orbital approximation, not a full Earth gravitational field and propagated ISS ephemeris. The truss root remains fixed; the large ISS mesh is visual context and does not recoil from robot forces. The flange/braces show an illustrative connection to the mesh, not a verified ISS hardware interface.

A code-inspection qualification is important: the MuJoCo implementation evaluates orbital loads at body centres of mass using COM Jacobians. The archived Isaac full-run implementation evaluates them at link-frame positions/velocities and applies them at those positions. The additional yaw-carrier bodies are also included. These are implementation differences; their effect has not been isolated in a dedicated equivalence study. The small orbital-rate coefficient does not by itself prove the difference is negligible.

Discrete supervisor updates and control evaluation also differ. MuJoCo evaluates its control callback within RK4 stages; Isaac computes and holds an explicit command over each PhysX step. Importing the same gains therefore does not guarantee identical numerical closed-loop dynamics.

## GPU simulation and recorded state

The Isaac full run used Isaac Sim 5.1, a Torch-backed World on `cuda:0`, GPU dynamics enabled, GPU broadphase, a 1 ms physics timestep, and TGS with 32 position and 4 velocity iterations. The log explicitly records GPU dynamics as enabled. The tested hardware was an RTX 1000 Ada laptop GPU with 6 GB VRAM.

PhysX integration runs on the GPU. The gait, inverse kinematics and force assembly use Python/NumPy on the CPU, with states copied out of GPU tensors and commands copied back. Thus this is not an entirely GPU-resident control pipeline, and GPU use does not imply real-time throughput or a demonstrated speedup over MuJoCo. Rendering is performed in a separate job.

The full PhysX run covers 120 simulated seconds, or 120,000 steps, and stores state at 100 Hz. `physx_full.npz` contains 12,001 rows plus native link poses and body names. The archived exact dynamics program is `source_archive/full_run_script_used.py`; the maintained `src/run_physx.py` includes later diagnostic logging and sleep/stabilization settings. The archive is retained as provenance, rather than silently claiming the maintained version is byte-identical to the delivered run.

The state row layout, using zero-based Python slices, is: time `[0]`; source robot coordinates `[1:19]`; modal coordinates `[19:35]`; robot velocities `[35:53]`; modal velocities `[53:69]`; supervisor state `[69:87]`; motor torques `[87:99]`; four Cartesian attachment forces `[99:111]`. MuJoCo rows additionally contain six energy/power/deflection metrics. See `DATA_FORMAT.md` for details.

## Recreating the ISS appearance and camera motion

The NASA ISS mesh vertex/face data were reused. The MuJoCo view loads processed STL meshes; Isaac builds USD meshes from saved NumPy vertex/face arrays. The Isaac robot visuals are regenerated from the bridge XML primitives and positioned from recorded native PhysX link transforms. This avoids relying on problematic imported rendering instances while retaining the source dimensions.

Truss graphics use the integrated modal coordinates to reconstruct nodal positions. Each visible member is positioned and oriented between its displaced endpoints with its corresponding length. The delivered Isaac movie used individual capsules; the maintained renderer subsequently used point instancing for rendering efficiency. This instancing preview was checked separately and does not alter the recorded physics.

The same physical-to-scene translation `(-29, 8, -1.15) m` places the robot and truss beside the ISS beam. It is a graphics placement transform, not an extra displacement applied to the orbital equations. Aluminium/steel/solar-cell materials, a directional Sun and fill illumination provide RTX shading and shadows. These materials are illustrative choices rather than measured optical properties.

Earth uses NASA Blue Marble and cloud imagery. MuJoCo composites a camera-aligned Earth background behind a segmentation mask; Isaac uses a textured sphere with radius 6,371 km and an illustrative 420 km altitude. The cloud/land composite is photographic, but illumination and atmosphere are not a validated radiative-transfer model. No live Earth imagery or actual flight ephemeris is implied.

Camera keys preserve the original sequence: full ISS at 0–3 s, approach and oblique views through 12 s, then one fixed side view. The side-view target is the scene offset plus `(3,0,0.28)`, with distance 5.6 m, azimuth 90 degrees and elevation -12 degrees. Isaac converts the source orbit-camera representation to a USD look-at transform and uses a 45-degree vertical field of view. Smooth interpolation controls the opening camera motion; the camera does not track the robot after 12 s.

Both final movies are 136 s at 1280×720 and 30 fps: 12 s opening with initial physics state held, 120 s of simulation at normal speed, and 4 s final hold. The Isaac renderer samples the saved PhysX state closest to each video timestamp. FFmpeg encodes 4,080 frames. Rendering saved states is normal offline visualization, but should not be confused with live real-time physics execution inside the rendered scene.

## Numerical checks and their limits

The full PhysX report records finite states, 4.7002359629 m body advance, at least three attached grippers in the saved data, 32 releases and 32 recaptures per leg, and four attached grippers at the end. The peak torque in the 100 Hz saved samples is 23.13428015 N m. The controller clamps each step to ±40 N m; the sampled peak is not a proof that no larger sub-sample torque occurred.

Independent source-geometry forward kinematics agreed with the saved full-run link positions to a maximum of 1.1666e-6 m (about 1.17 micrometres). In the short full-rate audit, maximum foot-velocity difference was 6.02e-7 m/s, and the independently calculated kinetic energy agreed with native mass-matrix kinetic energy within 7.60e-7 J. These checks help establish consistent geometry, frames and inertial bookkeeping. They do not establish experimentally correct robot behaviour.

The short timestep/configuration comparison reports maximum body-position difference 0.00038347 m and maximum joint-angle difference 0.0113754 rad between 0.5 ms/64-position-iteration and 1 ms/32-position-iteration configurations. Both timestep and solver iterations change in this comparison; it is a practical sensitivity check, not a clean convergence-order study. The faster 2 ms candidate was rejected during development.

The energy audit initially used inappropriate continuous-torque trapezoidal work for commands held over a PhysX step. For a 3 s pilot logged at 1 kHz, correcting motor work to `tau_n · (v_n+v_(n+1))/2 · dt` reduced the maximum residual from 22.2% to approximately 1.70% of the trajectory's mechanical-energy range, or 0.008539 J. Other powers are integrated trapezoidally. This is a numerical diagnostic and includes the defined attachment/damping/orbital terms.

The full PhysX log is only 100 Hz and cannot support the same step-rate energy audit. Its retained coarse-sample diagnostic has a large apparent residual (approximately 144% of energy range); this result is not silently removed or represented as a passing energy check. The short corrected audit is not a full-mission energy certificate. Full-rate logging and a full-mission energy audit remain required for a stronger validation claim.

MuJoCo's included full-mission report records final body x = 5.3502614702 m and an RK4-stage energy residual of approximately 0.2855% of energy range. These diagnostics use different sampling/work procedures from PhysX. Close final travel does not demonstrate identical transient forces, vibrations, capture timing or energy behaviour.

## Environment, reproducibility and remaining work

Isaac's bundled SciPy lacked a required MATLAB I/O submodule. A user-approved SciPy 1.15.3 installation in a project-local `python_deps` directory repaired the import. The scripts prepend that directory before loading Isaac. The dependency binaries and Isaac installation are not redistributed. The original GPU driver and global ROS/MATLAB installations were unchanged. This was a tested local configuration, not a declaration of official hardware/driver compatibility.

The repository includes the code, source coefficients, imported robot asset, original ISS/Earth assets needed by the renderers, full trajectories, final movies and selected numerical evidence. `REPRODUCE.md` explains setup and commands. The exported code removes two workstation-only optional-preview paths and makes the Isaac installation path configurable; these packaging edits are recorded separately from the original-file hashes. The original workspace remains untouched.

Priorities before publication as a quantitatively validated cross-engine model are: unify COM orbital-force application and update semantics; quantify yaw-carrier regularization; compare complete trajectories, attachment loads, modal response and event times; perform a controlled timestep/iteration study; and log a complete full-rate work/energy audit. Physical gripper validation would additionally require measured geometry, material/contact data, actuator characteristics and experimental comparison. Native finger–rail contact would constitute a new model beyond the compliant-attachment experiment documented here.

## Supervisor-ready description

> The existing MuJoCo spider-on-flexible-truss experiment was ported to Isaac Sim by adapting its MJCF articulation to USD/PhysX, preserving the source parameter set and implementing the gait supervisor, explicit motor controller, compliant attachment forces and 16-mode structural coupling in Python. GPU PhysX generated a new trajectory, which was subsequently rendered using RTX with NASA ISS/Earth assets and the original camera sequence. Geometry, selected numerical sensitivities, traversal and a short full-rate energy audit were checked. The demonstration retains assumed compliant grippers and a fixed-root linear modal truss; resolved jaw contact, spacecraft recoil, complete cross-engine equivalence and experimental validation are not claimed.
