# Physics and data provenance

NASA ISS mesh and NASA Blue Marble / cloud imagery are reused from the prior scene. Robot link geometry, link mass/inertia, 12 motor limits, gait, 6 m modular truss geometry, 16-mode FEM coefficients and grasp coefficients are copied from the user's existing spider research project.

These robot and structural values are research assumptions, not newly measured or certified hardware data. No friction coefficient or actuator performance has been invented and called validated. The user explicitly approved using existing values with assumptions labelled.

The current dynamics port preserves compliant spring–dashpot grasping and the linear modal FEM. Individual jaw friction, finger actuator dynamics, breakage, nonlinear beam contact and spacecraft recoil are outside this model. In particular a displayed closed jaw is not proof of a force-closure contact grasp.

PhysX uses native floating-base dynamics rather than MuJoCo's six base coordinates. Each two-joint shoulder is split into a yaw carrier and shoulder joint so Isaac's MJCF importer can preserve the separate axes. Four yaw carriers add 0.0001 kg each, with diagonal inertia 1e-6 kg m²; this explicit numerical regularization needs sensitivity checks. Modal coordinates are unit-mass fixed-base sliders, with original modal stiffness and custom cross-modal damping and force coupling.

PhysX TGS and MuJoCo RK4 are different integrators. Matching the controller and coefficients is not a claim of identical trajectories or verified physical accuracy. The port must pass motion, grip, force, energy and timestep checks before it is described as equivalent.

The computer has a 6 GB RTX 1000 Ada GPU. Isaac Sim 5.1 starts on driver 550.163.01, below the documented recommended driver; this is a tested local configuration, not official hardware support or a promise of maximum RTX settings. No driver or system package has been changed. The missing bundled SciPy module is repaired through an approved project-local SciPy 1.15.3 installation.

The initial energy audit incorrectly used continuously varying torque quadrature for held PhysX torque commands. A full-rate 1 kHz pilot with held-input midpoint motor work reduced its relative energy residual from 22.2% to 1.7%. Native mass-matrix kinetic energy agrees with the independent source-inertia calculation within 0.8 microjoules. Sleep/stabilization tests did not change the pilot. The full 120-second run is logged at 100 Hz and does not support a complete integration-rate energy audit; the pilot result is not a full-mission energy certificate. The original naive-audit values are retained in the diagnostic data for transparency.

RTX imagery is generated from the original mesh data with physically based materials and directional sunlight at an illustrative orientation. The Earth background is a NASA land/cloud photographic composite, not a live Earth ephemeris or a validated atmospheric scattering model.
