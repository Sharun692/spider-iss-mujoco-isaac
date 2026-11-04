# Saved data

All arrays use SI units unless stated otherwise; joint angles are radians. The common state row uses zero-based Python indexing:

- `0`: physical simulation time, seconds.
- `1:19`: robot state, base xyz, source roll/pitch/yaw, then yaw/shoulder/knee per leg (four legs).
- `19:35`: sixteen modal coordinates eta.
- `35:53`: robot rates in the same source coordinate order; Euler rates are not world angular velocity.
- `53:69`: sixteen modal velocities.
- `69`: gait progress s.
- `70:74`: four rail anchor positions along x.
- `74:78`: attachment flags, 0 released or 1 latched.
- `78:82`: attachment ages.
- `82:86`: last capture errors.
- `86`: gait progress rate, zero when paused or completed.
- `87:99`: twelve commanded motor torques, N m.
- `99:111`: four xyz attachment-force vectors, N, flattened leg by leg.

The 117-column MuJoCo data additionally stores energy (J) at 111, motor power (W) at 112, dissipated power at 113, orbital power at 114, signed tip displacement (m) at 115 and time-varying attachment-stiffness power at 116. The NPZ `rk4_work` array stores accumulated stage-weighted work.

The full PhysX NPZ also contains `link_poses` with shape (samples, bodies, 7): xyz followed by quaternion xyzw, and `body_names` for those bodies. These poses are in the physics frame before the ISS graphics offset. The full trajectory has 12,001 samples at 0.01 s intervals, despite integration at 0.001 s.

The short `physx_fullrate_audit.npz` adds diagnostic arrays, including native kinetic energy and link velocities, at the actual solver rate. Refer to its NPZ keys rather than assuming all runs have the same optional arrays. The maintained simulator can log more diagnostic fields than the archived full run.

Use `numpy.load(path, allow_pickle=False)`; the supplied records do not require pickle loading. Original JSON diagnostics are retained even where they report unsuccessful candidates or an invalid coarse-sample energy audit. A JSON file's presence is not a passing validation certificate. Short candidate trajectories not needed to reproduce the final movies are not all included; the full mission and the corrected 3 s PhysX audit are included.
