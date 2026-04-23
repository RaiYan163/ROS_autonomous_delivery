#!/usr/bin/env bash
# Kill TurtleBot3 / Gazebo / SLAM / teleop style processes (ROS 2 Jazzy friendly).
set -euo pipefail

echo "Stopping Gazebo / RViz / Cartographer / teleop / common ROS bridges..."

# Gazebo (Harmonic / ros_gz)
pkill -f 'gz sim'           2>/dev/null || true
pkill -f 'ros_gz_sim'       2>/dev/null || true
pkill -f 'parameter_bridge' 2>/dev/null || true
pkill -f 'image_bridge'     2>/dev/null || true

# Visualization / SLAM
pkill -f 'rviz2'            2>/dev/null || true
pkill -f 'cartographer'     2>/dev/null || true
pkill -f 'cartographer_node' 2>/dev/null || true
pkill -f 'occupancy_grid'   2>/dev/null || true

# Teleop / generic ROS nodes often used with TurtleBot3
pkill -f 'teleop_keyboard'  2>/dev/null || true
pkill -f 'teleop_twist'     2>/dev/null || true
pkill -f 'turtlebot3_teleop' 2>/dev/null || true

# Robot state / launch leftovers
pkill -f 'robot_state_publisher' 2>/dev/null || true

echo "Done. If something still holds /cmd_vel, run: ros2 topic info /cmd_vel"
echo "Nuclear option (kills ALL ros2 processes on this user):"
echo "  pkill -f ros2"