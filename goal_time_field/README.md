# Goal Time Field v1

`goal_time_field` learns `T(q, q_goal | O_fixed)` for one fixed planning scene.
It learns no trajectory labels: the loss enforces `S(d(q)) * ||dT/dq|| = 1`.
FCL clearance is only mapped heuristically from workspace meters to a joint-space
propagation speed in rad/s; it is not a joint-space metric from FCL.

The field is not scene-generalizable. If obstacle geometry changes materially,
its gradient is no longer guaranteed appropriate even though runtime speed uses
the current clearance. The node only publishes nominal velocity. A future CBF
layer must filter it into `qdot_safe`; this package never commands a robot.

Source code is deliberately separated by responsibility:

```text
goal_time_field/
├── core/       # Model, speed mapping, nominal-velocity calculation; no ROS or dataset I/O
├── training/   # CSV dataset, PDE training, evaluation and slice visualization
└── ros/        # inference_node.py: checkpoint + ROS topics only
```

```bash
source ~/ros_ws/install/setup.bash
# Start obstacle_sim.launch.py first, then generate a fixed-scene dataset.
ros2 run obstacle_distance_calculator field_dataset_generator --ros-args \
  -p samples:=20000 -p output_csv:=/tmp/ur3_field.csv \
  -p metadata_json:=/tmp/ur3_field.json

ros2 run goal_time_field goal_time_field_train -- \
  --csv /tmp/ur3_field.csv --metadata /tmp/ur3_field.json \
  --config $(ros2 pkg prefix goal_time_field)/share/goal_time_field/config/goal_time_field.yaml \
  --checkpoint /tmp/ur3_field.pt

ros2 run goal_time_field goal_time_field_evaluate -- --csv /tmp/ur3_field.csv \
  --metadata /tmp/ur3_field.json --checkpoint /tmp/ur3_field.pt

ros2 run goal_time_field goal_time_field_plot_slice -- --metadata /tmp/ur3_field.json \
  --checkpoint /tmp/ur3_field.pt --output field_slice.png

ros2 run goal_time_field goal_time_field_node --ros-args \
  -p checkpoint_path:=/tmp/ur3_field.pt \
  -p q_goal:="[0.0,-1.57,1.57,-1.57,-1.57,0.0]"
```
