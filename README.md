# field_control

A clean, high-performance ROS 2 package providing real-time 3D solid obstacle distance calculations (MoveIt / RViz integration) for UR3/UR3e manipulators.

## Package Architecture (Clean `src/` Layout)

```
field_control/
├── launch/
│   ├── obstacle_distance.launch.py # Main launch file
│   └── field_control.launch.py     # Backward-compatible launch alias
├── src/
│   └── obstacle_distance/          # Python module (No duplicate folder names!)
│       ├── __init__.py
│       ├── obstacle_distance_node.py # ROS 2 node & RViz markers publisher
│       ├── obstacle_distance.py      # 3D solid obstacle distance calculator per link
│       └── urdf_kinematics.py        # Dynamic URDF forward kinematics engine
├── package.xml
├── setup.py
└── README.md
```

## Features
- **URDF Kinematics & Solid Geometry Support**: Reads robot URDF models dynamically and accounts for 3D link collision body volumes.
- **MoveIt / RViz PlanningScene Integration**: Automatically extracts solid obstacles (Boxes, Cylinders, Spheres) from MoveIt `/planning_scene` and `/monitored_planning_scene` topics.
- **Per-Link Minimum Distance Breakdown**: Computes real-time surface-to-surface minimum distance from all solid obstacles to each individual robot link.
- **RViz Visual Feedback**: Publishes 3D color-coded distance vectors and text labels (`/obstacle_distance/markers`) directly into RViz.

---

## Building the Package
```bash
cd ~/ros_ws
colcon build --packages-select field_control --symlink-install
source install/setup.bash
```

---

## Running the Real-Time Distance Calculator Node
```bash
source install/setup.bash
ros2 launch field_control obstacle_distance.launch.py
```

### Visualizing in RViz 2
1. Open RViz 2.
2. Click **Add** -> **MarkerArray**.
3. Set topic to `/obstacle_distance/markers`.
4. You will see 3D line vectors connecting each robot link to its closest solid obstacle point, along with floating text labels displaying the exact distance in meters.
   - **Green**: Distance $> 0.15$ m (Safe)
   - **Yellow**: Distance between $0.05$ m and $0.15$ m (Warning)
   - **Red**: Distance $< 0.05$ m (Critical / Collision Risk)

---

## Python API Usage

```python
import numpy as np
import shape_msgs.msg
from obstacle_distance import SolidObstacleDistanceCalculator

# 1. Initialize calculator (uses UR3 URDF by default)
calc = SolidObstacleDistanceCalculator()

# 2. Define current robot joint angles q (radians)
q = [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]

# 3. Create a solid BOX obstacle (e.g., Table at z = -0.01m)
table_box = shape_msgs.msg.SolidPrimitive()
table_box.type = shape_msgs.msg.SolidPrimitive.BOX
table_box.dimensions = [1.2, 0.8, 0.02]  # [dx, dy, dz]

table_pose = np.eye(4)
table_pose[2, 3] = -0.01

obstacles = [
    {
        'id': 'table',
        'primitives': [table_box],
        'primitive_poses': [table_pose]
    }
]

# 4. Compute per-link distance breakdown
result = calc.compute_per_link_distances(q, obstacles)

print(f"Overall Minimum Distance: {result['overall_min_distance']:.4f} m")
print(f"Overall Closest Link: {result['overall_closest_link']}")

print("\nPer-link distance breakdown:")
for link_name, info in result['per_link'].items():
    print(f"  {link_name}: {info['distance']:.4f} m (closest to '{info['closest_obstacle_id']}')")
```
