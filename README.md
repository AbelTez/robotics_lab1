# Lab 1 Exercise — diffdrive_bot with depth camera, IMU, turn-away, services and actions

This repository contains the four packages that make up the Lab 1 exercise:

| Package | Contents |
| --- | --- |
| `diffdrive_bot` | Robot model, Gazebo world, bridge config and the lab's example nodes (extended with a depth camera and an IMU) |
| `lab_interfaces` | `ObstacleInfo` message, `SetSpeed` service, `DriveDistance` action (from the lab) |
| `my_robot_interfaces` | `RobotStatus` message, `SetTurnSpeed` service, `Rotate` action |
| `my_robot_nodes` | `turn_away`, `rotate_server`, `square_driver` and `square.launch.py` |

## Prerequisites

- Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic
- `ros-jazzy-ros-gz`, `ros-jazzy-xacro`, `ros-jazzy-robot-state-publisher`,
  `ros-jazzy-rviz2`, `ros-jazzy-joint-state-publisher-gui`,
  `ros-jazzy-teleop-twist-keyboard`, `ros-jazzy-tf2-tools`

Every terminal used below must be sourced:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

## Build

Clone the repository into the `src` folder of a workspace and build it:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <this-repository-url> lab1_exercise
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

`colcon list` must show the four packages: `diffdrive_bot`, `lab_interfaces`,
`my_robot_interfaces`, `my_robot_nodes`.

After every change rebuild with the same block. URDF, world and YAML edits are
symlinked, so they only need a launch restart; new or changed Python nodes and
interfaces need a rebuild.

## Step 1 — Depth camera and IMU

The depth camera sits on the back face of the chassis, turned half a turn about
`z`, and publishes at 10 Hz on `/depth_camera` (`320x240`, `R_FLOAT32`, 0.1–10 m,
camera info on `/depth_camera/camera_info`). The IMU sits at the chassis centre
and publishes at 50 Hz on `/imu`. The world loads `gz-sim-imu-system` and
`config/bridge.yaml` bridges the image, point cloud, camera info and IMU to ROS 2.

```bash
ros2 launch diffdrive_bot display.launch.py
# depth_camera_link and imu_link are in the model tree (Ctrl+C when done)

ros2 run tf2_ros tf2_echo base_link depth_camera_link
# translation -0.215 0 0, rpy 0 0 3.14159

ros2 launch diffdrive_bot gazebo.launch.py
# in RViz add Image /depth_camera/image and PointCloud2 /depth_camera/points

ros2 topic hz /imu
ros2 topic echo /imu --once     # linear_acceleration.z ~ 9.8
```

Screenshot: `images/rviz_depth_camera.png`.

## Step 2 — `turn_away` (topics)

`turn_away` subscribes to `/obstacle` and publishes `/cmd_vel`: forward at
0.2 m/s while the way is clear, and a turn at 0.5 rad/s away from the obstacle
when `too_close` is true.

```bash
# terminal 1
ros2 launch diffdrive_bot control.launch.py

# terminal 2
ros2 run my_robot_nodes turn_away

# terminal 3
rqt_graph     # turn_away sits between /obstacle and /cmd_vel
```

## Step 3 — Parameters

`forward_speed` (default 0.2) and `turn_speed` (default 0.5) are declared and
read inside the subscription callback, so a change takes effect on the next
obstacle message.

```bash
ros2 run my_robot_nodes turn_away
ros2 param list /turn_away
ros2 param set /turn_away forward_speed 0.4    # the robot speeds up immediately
```

## Step 4 — `RobotStatus` message

`my_robot_interfaces/msg/RobotStatus.msg`:

```
string state               # "forward", "turning" or "stopped"
float32 obstacle_distance  # m, distance to the nearest obstacle in front
```

`turn_away` publishes it on `/robot_status` for every `/obstacle` message.

```bash
ros2 interface show my_robot_interfaces/msg/RobotStatus
ros2 run my_robot_nodes turn_away
ros2 topic echo /robot_status
```

## Step 5 — Services

`turn_away` starts disabled and provides:

- `/turn_away_enable` (`std_srvs/srv/SetBool`) — switching off publishes one zero
  `Twist` and reports `state: stopped`;
- `/set_turn_speed` (`my_robot_interfaces/srv/SetTurnSpeed`) — rejects values
  `<= 0` or greater than the `max_turn_speed` parameter (default 1.5) and stores
  an accepted value in the `turn_speed` parameter.

```bash
ros2 run my_robot_nodes turn_away
ros2 service call /turn_away_enable std_srvs/srv/SetBool "{data: true}"
ros2 service call /set_turn_speed my_robot_interfaces/srv/SetTurnSpeed "{turn_speed: 1.0}"   # accepted
ros2 service call /set_turn_speed my_robot_interfaces/srv/SetTurnSpeed "{turn_speed: 3.0}"   # rejected
ros2 service call /turn_away_enable std_srvs/srv/SetBool "{data: false}"                    # robot stops
```

## Step 6 — `Rotate` action server

`my_robot_interfaces/action/Rotate.action`:

```
float32 angle     # goal, rad, positive turns left
---
float32 turned    # result, rad
---
float32 turned    # feedback, rad
```

`rotate_server` accumulates yaw deltas between successive `/odom` messages
(never `current - start`, which breaks when the yaw wraps at ±π), publishes
feedback, stops at the goal and stops the robot on cancel. A goal of `0` is
rejected. It uses a `MultiThreadedExecutor`.

```bash
ros2 run my_robot_nodes rotate_server
ros2 action send_goal --feedback /rotate my_robot_interfaces/action/Rotate "{angle: 1.57}"
ros2 run tf2_ros tf2_echo odom base_link
```

## Step 7 — `square_driver` and `square.launch.py`

`square_driver` sends four pairs of goals, strictly one after another: a
`/drive_distance` goal of `side_length` (default 0.5 m), then a `/rotate` goal of
`pi/2`. On a failure it stops the robot and logs which side failed.
`square.launch.py` includes `diffdrive_bot/launch/control.launch.py`, starts
`rotate_server` and `square_driver`, and forwards `side_length`.

```bash
ros2 launch my_robot_nodes square.launch.py side_length:=0.8
# logs Side 1..4 driving/turning, then "Square complete"
```

Screenshot: `images/rosgraph.png` (rqt_graph while `square.launch.py` runs).

## Notes on running the nodes

Only one node may own `/cmd_vel` at a time. Do not call `/drive_enable` or send
`/drive_distance` goals while `turn_away` or `square_driver` is running.

## What did not work and why I think so

- **`ros2 run my_robot_nodes turn_away` → `Package 'my_robot_nodes' not found`.**
  The package existed in `src` but had not been built yet, and the terminal had
  only sourced `/opt/ros/jazzy`. Building with `colcon build --symlink-install`
  and sourcing `install/setup.bash` in every terminal fixed it.
- **`ros2 topic echo /robot_status` → `The message type
  'my_robot_interfaces/msg/RobotStatus' is invalid`.** That shell had been
  sourced before `my_robot_interfaces` was built, so its environment did not
  know the new interface. Sourcing `install/setup.bash` again fixed it.
- **`rqt_graph` → `Could not find the Qt platform plugin "wayland"`.** On this
  Wayland session Qt picks a backend rqt does not ship. Running
  `QT_QPA_PLATFORM=xcb rqt_graph` works.
- **`rqt_graph` window empty.** The default filter is "Nodes/Topics (active)",
  and by the time it opened `square_driver` had already finished and exited.
  Selecting "Nodes/Topics (all)" and refreshing, or starting `rqt_graph` right
  after the launch, shows the graph.
- **`/imu` measured ~47 Hz instead of 50 Hz.** The rate is set in simulation
  time, and Gazebo runs slightly below real time on this machine, so fewer
  messages arrive per wall-clock second. The sensor configuration itself is
  50 Hz.
