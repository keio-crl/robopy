# Rakuda-2 URDF post-processing

The original Onshape export is preserved as `urdf/assembly_2.urdf`.

## Added URDF variants

- `urdf/assembly_2_mesh_collision.urdf`
  - Adds collision geometry by reusing each visual STL.
  - Highest geometric fidelity, but computationally heavy (~1.1M source triangles total).
  - Useful as a reference / initial collision check.

- `urdf/assembly_2_convex_collision.urdf`
  - Adds one convex-hull STL per visual STL under `collision_meshes/`.
  - Recommended starting point for physics simulation because collision geometry is simpler and convex.
  - Convex hulls over-approximate concave parts, so inspect contact-critical regions (hands, narrow gaps, etc.).

## ROS package files added

- `package.xml`
- `CMakeLists.txt`
- `launch/assembly_2_mesh_collision.launch`
- `launch/assembly_2_convex_collision.launch`

The supplied launch file uses ROS 1 XML syntax, so these metadata files are set up as a ROS 1/catkin package.

## Model audit

- Links: 153
- Joints: 152
- Joint types: fixed=137, revolute=12, continuous=3
- Actuated/non-fixed DOF count: 15
- Visual meshes: 137
- Collision elements in original: 0
- Total exported mass: 0.002069298959 kg (suspiciously small; intentionally NOT modified)
- Convex hulls generated: 137/137
- Convex hull fallbacks: 0

### Continuous joints to verify in Onshape

- shoulder_pitch_left_dof
- shoulder_pitch_right_dof
- torso_yaw_dof

If these joints are not physically unlimited, add Mate limits in Onshape and re-export so they become bounded revolute joints.

### Important values intentionally not guessed

1. **Mass / inertia**: the exported total mass is unusually small. Correct values require Onshape material/density/unit verification or measured link masses.
2. **Effort / velocity limits**: current revolute limits use exporter values (mostly `1`). Replace with actuator-specific values if using dynamics/controllers.
3. **Self-collision filtering**: this is simulator/planner-specific (e.g. MoveIt SRDF), not safely inferable from CAD alone.
4. **Fixed-link reduction**: the model has many fixed sublinks. They are preserved to avoid changing transforms or frame names; many simulators can collapse fixed joints automatically.

## Recommended file

Start with `assembly_2_convex_collision.urdf` for physics simulation. Use `assembly_2_mesh_collision.urdf` when checking whether the convex approximation is too coarse.
