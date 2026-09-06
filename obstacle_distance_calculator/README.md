# Robot–obstacle distance

`/obstacle_distances` reports distances in metres, with points and normals in
`header.frame_id` (the MoveIt planning frame). The minimum, its pair names and
the corresponding entry in `distances` all come from the same calculation.

- Positive: surfaces are separated.
- Zero: surfaces touch, up to floating-point precision.
- Negative: overlap; the value is a signed clearance, not a binary flag.
- `inf` with empty pair names: no pair was found within `distance_threshold`
  (default 0.3 m), or the world has no eligible obstacles. It is not a measured
  infinite clearance. Increase the threshold to measure more distant objects.

This calculator checks robot links and attached bodies against the world. It
does not check self-collision. `group_name`, Allowed Collision Matrix entries
marked `ALWAYS`, and `use_unpadded_env` are respected. Like MoveIt's distance
query, conditional ACM contact predicates are not applied to distance values.

## Mesh–sphere calculation

UR3 collision links are triangle meshes and the simulation obstacles are
spheres. In the installed FCL 0.7 implementation, `sphereTriangleDistance`
with nearest points enabled leaves its output distance unwritten when a sphere
touches or intersects a triangle. The mesh traversal consumes that output;
this can produce tiny garbage numbers or stale positive distances. Its nearest
points also use individual shape frames. Relevant installed code:

- `/usr/include/fcl/narrowphase/detail/primitive_shape_algorithm/sphere_triangle-inl.h`
- `/usr/include/fcl/narrowphase/detail/traversal/distance/mesh_shape_distance_traversal_node-inl.h`

The calculator bypasses that mesh–sphere narrow-phase path. It searches the
mesh's bounding-volume tree for the closest point to the sphere centre, then
subtracts the radius. Closed-mesh containment is determined by ray crossings;
an enclosed centre has negative signed surface distance before subtracting the
radius. All witnesses are transformed into the planning frame. Open or
non-manifold meshes do not define reliable inside/outside containment.

For separated bodies, `point_on_obstacle - point_on_link = distance * normal`.
During mesh–sphere overlap, the normal follows the signed clearance's outward
direction. This field is not necessarily differentiable at a change of closest
triangle or closest pair; a CBF must not assume a globally smooth gradient.
Other shape combinations use MoveIt's signed FCL callback and its contact-depth
convention. No constant `-1e-6` or magnitude-based garbage filter is used.

## Build and verification

From `/home/dhcn-1/ros_ws`:

```bash
colcon build --packages-select obstacle_distance_calculator --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
colcon test --packages-select obstacle_distance_calculator
colcon test-result --test-result-base build/obstacle_distance_calculator/test_results --verbose
source install/setup.bash
```

Regression tests cover known separation, zero contact, overlap, containment,
mesh–sphere sweeps, frame transforms, multiple obstacles, ACM, group filtering,
padding, supplied robot state, attached mesh geometry and the distance cutoff.

`build/obstacle_distance_calculator/distance_snapshot_check` is an optional
read-only live diagnostic. Pass the simulation's robot-description parameter
file with `--ros-args --params-file PATH`. It requests `/get_planning_scene`,
checks witness consistency and sweeps a sphere across the closest surface in
its **local copy** of the scene. It publishes no robot or planning-scene updates.

After rebuilding, restart the running distance node to load the new library.
Then inspect the minimum:

```bash
ros2 topic echo /obstacle_distances --field overall_min_distance
```

Datasets/checkpoints generated using the old distance code should be regenerated
for quantitative clearance training; previous collision labels and distances
may be incorrect.
