# Holosoma Sim2Real ROS2 Packages

This repository groups the Holosoma-local ROS2 packages used by the
GeneralReward sim2real Docker workflow.

## Packages

- `far_msgs`: policy action and robot state message definitions.
- `holosoma_lidar_cpp_publisher`: ZeroMQ-to-ROS2 LiDAR publisher.
- `holosoma_robot_description`: self-contained G1 robot description and TF helpers.

These packages are intended to be cloned into a ROS2 workspace source directory,
for example `pkgs/ros_ws/src/holosoma_sim2real_pkgs`.
