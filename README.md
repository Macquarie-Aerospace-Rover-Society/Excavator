# Excavator ROS 2 Humble Project

ROS 2 Humble workspace for the MARS (Macquarie University Rover Society) front loader bucket subsystem, officially nicknamed **Excavator**, for the Australian Rover Challenge.

This project implements the kinematics and static force model from the provided MATLAB simulator and publishes state suitable for controller integration and RViz visualization.

## Workspace Layout

- `src/excavator_kinematics`: ROS 2 Python package with the model, node, launch, and config.

## Features

- Elevation (`re`) and tilt (`rt`) command handling.
- Actuator stroke clamping to hardware limits (380-630 mm).
- Joint coordinate calculation for `J1`, `J2`, `J3`, `J4`.
- Stroke percentages and end-stop warning flags.
- Angular speed estimation from actuator stroke speed.
- Static force estimation (`F_L1`, `F_L2`) under payload gravity.
- ROS publishers for joint state, telemetry, and RViz markers.

## Node Architecture

The project is split into multiple ROS nodes:

- `excavator_state_node`: computes mechanism state from desired command angles.
- `excavator_visualization_node`: publishes `/joint_states`, markers, and telemetry from computed state.
- `excavator_keyboard_node`: optional keyboard teleop for desired `re` / `rt` / `mass`.

## Build

From the workspace root (`Excavator`):

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Gazebo Prerequisite (Ubuntu-22.04 / ROS 2 Humble)

Install modern Gazebo Sim (`gz sim`) support:

```bash
sudo apt update
sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge
```

If `ros-humble-ros-gz-sim` is unavailable in your apt sources, install:

```bash
sudo apt install ros-humble-ros-gz ros-humble-ros-gz-bridge
```

## Run

Single supported launch path (Modern Gazebo GUI fallback + keyboard teleop):

```bash
ros2 launch excavator_kinematics excavator_gz_sim_gui.launch.py enable_keyboard:=true
```

VS Code task:

- `Launch Excavator`

Keyboard controls in teleop node:

- `w/s`: increase/decrease elevation `re`
- `a/d`: increase/decrease tilt `rt`
- `r/f`: increase/decrease load mass
- `z`: reset to initial setpoints
- `q`: quit keyboard node

## Command Topic

Publish command inputs to:

- Topic: `excavator/command`
- Type: `geometry_msgs/msg/Vector3`
- Fields:
- `x`: elevation angle `re` in degrees
- `y`: tilt angle `rt` in degrees
- `z`: payload mass in kg

Angle convention:

- `re` and `rt` follow the MATLAB world-frame definitions in the layout diagram.
- The URDF bucket joint is relative, so the node publishes `excavator_bucket_joint = rt - re` internally.

Example:

```bash
ros2 topic pub /excavator/command geometry_msgs/msg/Vector3 "{x: 30.0, y: -45.0, z: 10.0}" --once
```

## Published Topics

- `/joint_states` (`sensor_msgs/msg/JointState`)
- `/excavator/stroke_monitor` (`std_msgs/msg/Float32MultiArray`)
- `/excavator/forces` (`geometry_msgs/msg/Vector3`)
- `/excavator/markers` (`visualization_msgs/msg/MarkerArray`)

`/excavator/stroke_monitor` array order:

1. `L1_mm`
2. `L2_mm`
3. `L1_pct`
4. `L2_pct`
5. `re_dot`
6. `rt_dot`

## Notes

- Geometry is computed in millimeters to match the original mechanism model.
- RViz markers are converted to meters (`mm * 0.001`) for visualization.
- Dynamics are static/quasi-static estimates (no full multibody dynamics yet).
