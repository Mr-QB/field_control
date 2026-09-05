# field_control

ROS 2 C++ packages for robot-to-world distances using MoveIt's PlanningSceneMonitor:

- `obstacle_distance_msgs`: distance message definitions.
- `obstacle_distance_calculator`: distance calculation library and ROS node.

## Build and run

```bash
cd ~/ros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select obstacle_distance_msgs obstacle_distance_calculator --symlink-install
source install/setup.bash
ros2 launch obstacle_distance_calculator obstacle_distance.launch.py
```

Start the robot's MoveIt stack first. The node needs the robot URDF and SRDF,
joint states, transforms, and the planning scene. The default launch loads
`obstacle_distance_calculator/config/obstacle_distance.yaml`. MoveIt's model
loader can receive descriptions from the `robot_description` and
`robot_description_semantic` topics; ensure the upstream stack publishes them.
Alternatively, pass the XML directly as node parameters in your robot launch:

```python
Node(
    package='obstacle_distance_calculator',
    executable='obstacle_distance_node',
    name='obstacle_distance_node',
    parameters=[config_file, {
        'robot_description': urdf_xml,
        'robot_description_semantic': srdf_xml,
    }],
)
```

`robot_description_parameter` selects the parameter name (default
`robot_description`). It must not contain XML. When using a custom name, the
matching semantic description uses the same name with `_semantic` appended.
The node exits if the robot model cannot be loaded.

## Configuration

| Parameter | Default | Meaning |
| --- | --- | --- |
| `robot_description_parameter` | `robot_description` | URDF parameter/topic name |
| `planning_scene_topic` | `/monitored_planning_scene` | MoveIt scene updates |
| `planning_scene_service` | `/get_planning_scene` | Initial full-scene request; empty disables it |
| `group_name` | empty | All links; otherwise a robot planning group |
| `output_topic` | `obstacle_distances` | Distance array topic |
| `publish_frequency` | `50.0` | Positive, finite timer frequency in Hz |
| `distance_threshold` | `0.3` | Nonnegative, finite maximum reported distance in meters |
| `use_unpadded_env` | `true` | Use collision geometry without environment padding |

The node subscribes to the configured scene topic before requesting the initial
scene. If the request fails, it warns and continues waiting for topic updates.
For a setup publishing directly to `/planning_scene`, configure that topic.

## Output

```bash
ros2 topic echo /obstacle_distances
```

`obstacle_distance_msgs/msg/LinkObstacleDistanceArray` contains the frame and
publication timestamp, link/obstacle pairs, signed surface distances, nearest
points, and normals. The calculator requests the minimum distance for each
pair, subject to the configured threshold and allowed collision matrix.
It does not compute robot self-collision distances.

`overall_min_distance` is the minimum of reported pairs. If none are returned,
it is positive infinity and the closest names are empty. This is not proof
that the scene is populated or the robot state is current. The node does not
currently gate publication on state freshness.

This implementation does not publish RViz markers or provide the old Python API.

## UR simulation with random spheres

```bash
source ~/ros_ws/install/setup.bash
ros2 launch obstacle_distance_calculator obstacle_sim.launch.py
```

This starts a UR3 kinematic simulation, MoveIt, RViz, joint sliders, the distance
node, and five fixed orange spherical obstacles. Move the joint sliders to change the
robot pose and inspect `/obstacle_distances`. This launch uses wall time, without
Gazebo, physics, or trajectory execution. RViz displays the robot and collision
objects through its PlanningScene display.

```bash
# Eight random spheres, radius 3–8 cm, repeatable layout
ros2 launch obstacle_distance_calculator obstacle_sim.launch.py randomize:=true count:=8 radius_min:=0.03 radius_max:=0.08 seed:=42
# Headless validation / fixed robot pose
ros2 launch obstacle_distance_calculator obstacle_sim.launch.py rviz:=false joint_gui:=false seed:=42
```

The fixed obstacle list is in `obstacle_distance_calculator/config/fixed_obstacles.yaml`.
Edit `fixed_positions` and `fixed_radii` together: every `[x, y, z]` position has
one radius, in meters, and the object IDs are `obstacle_0`, `obstacle_1`, etc.

With `randomize:=true`, defaults are `count:=5`, `seed:=-1` (new layout each run),
`radius_min:=0.04`, `radius_max:=0.10`. Sphere centers are sampled in `base_link`
coordinates from `x_min/x_max=-0.55/0.55`, `y_min/y_max=-0.55/0.55`,
`z_min/z_max=0.15/0.75`, all in meters. Objects remain stationary after insertion.
Sphere pairs have at least 2 cm clearance and avoid a cylinder around the base;
this does not guarantee clearance from every robot link. Dense or impossible
sampling bounds produce an explicit error. The spawner waits for MoveIt's apply
scene service and retries failed responses rather than relying on a launch delay.
Use this launch in its own simulation session: object IDs are
`random_obstacle_0`, `random_obstacle_1`, etc.
