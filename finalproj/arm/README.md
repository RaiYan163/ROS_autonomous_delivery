# Final Project Arm Workspace

This workspace contains two separate OpenManipulator-X paths:

1. The official ROBOTIS hardware bringup and keyboard teleop stack.
2. The custom `finalproj_openmanipulator_control` package for the course final.

## Official ROBOTIS Teleop

On this machine, the practical path is ROS 2 Jazzy on Ubuntu 24.04. The
current ROBOTIS OpenManipulator quick-start guide also points Jazzy users at
the unified `open_manipulator_*` packages, while the older Humble branch still
uses `open_manipulator_x_*`.

Set up the official hardware bringup and keyboard teleop packages in this
workspace:

```bash
cd /home/mkros/robotics/finalproj/arm
./setup-openmanipulator-teleop.sh
```

That script clones the official ROBOTIS repositories into `src/robotis/` and
builds the packages needed for:

- `ros2 launch open_manipulator_bringup open_manipulator_x.launch.py`
- `ros2 run open_manipulator_teleop open_manipulator_x_teleop`

Use the helper wrappers after the build:

```bash
cd /home/mkros/robotics/finalproj/arm
./run-openmanipulator-hardware.sh
./run-openmanipulator-teleop.sh
```

If you are using OpenCR instead of U2D2:

```bash
./run-openmanipulator-hardware.sh port_name:=/dev/ttyACM0
```

The wrappers auto-detect whether the installed packages are the newer Jazzy
names (`open_manipulator_*`) or the older Humble names
(`open_manipulator_x_*`).

## Final Project Package

The custom final-project package in `src/finalproj_openmanipulator_control/`
uses ROS 2 Jazzy in this workspace:

```bash
cd /home/mkros/robotics/finalproj/arm
./setup-openmanipulator-teleop.sh
./build-jazzy.sh
source install/setup.bash
```

Then launch it with:

```bash
ros2 launch finalproj_openmanipulator_control system.launch.py
```

The final-project controller can record and replay arm movement from the
gamepad. Playback is always available from an existing recording: press the
right trigger to play or stop the saved motion. Recording is disabled by
default so you cannot overwrite a recording by accident during normal operation.
Launch with `recording:=true`, press the left trigger once to start recording,
and press it again to stop and save. The recording is stored at
`~/.ros/finalproj_openmanipulator_motion.yaml`.

Recorded playback sends the saved joint positions back through the arm
trajectory controller. The move into the first recorded pose is slowed by
`playback_start_move_duration` in `config/controller.yaml`; increase it if the
arm still approaches the start too aggressively. The velocity-streaming
experiment is preserved for later in `velocity_playback_reference.py`, but it
is not active at runtime.

Or with OpenCR:

```bash
ros2 launch finalproj_openmanipulator_control system.launch.py \
  hardware_port_name:=/dev/ttyACM0
```

## Two-PC SSH Setup

Use this when the OpenManipulator-X is plugged into a remote PC, but the
joystick/teleop input stays on this local machine.

Both PCs must be on the same network and use the same ROS domain. This project
defaults the helper scripts to `ROS_DOMAIN_ID=30`, matching the local machine.
If you change it, change it on both PCs:

```bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
```

On the remote PC, clone or copy this workspace, build it, and start only the
arm-side nodes:

```bash
ssh USER@REMOTE_PC
cd /home/mkros/robotics/finalproj/arm
sudo apt-get update
sudo apt-get install -y ros-jazzy-moveit-servo
./setup-openmanipulator-teleop.sh
./build-jazzy.sh
./run-finalproj-remote-arm.sh
```

If the remote PC uses OpenCR instead of U2D2:

```bash
./run-finalproj-remote-arm.sh hardware_port_name:=/dev/ttyACM0
```

On this local PC, start only the joystick/controller side:

```bash
cd /home/mkros/robotics/finalproj/arm
./run-finalproj-local-teleop.sh
```

The local joystick helper still asks the arm-side MoveIt Servo node to enter
joint-jog mode, so start the arm-side launch before starting local teleop.

If local teleop connects to the joystick but does not move the arm, confirm both
machines use the same `ROS_DOMAIN_ID` and that the local machine can see
`/servo_node/switch_command_type` and `/servo_node/pause_servo`.

To enable recording/playback on the local joystick side:

```bash
./run-finalproj-local-teleop.sh recording:=true
```

For the official ROBOTIS keyboard teleop instead of the final-project joystick
controller, run this locally after the remote hardware launch is up:

```bash
cd /home/mkros/robotics/finalproj/arm
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
./run-openmanipulator-teleop.sh
```

Quick checks:

```bash
ros2 node list
ros2 topic echo /joint_states
ros2 topic echo /joy
```

If the machines cannot see each other, verify they are on the same Wi-Fi or
Ethernet network, neither shell has `ROS_LOCALHOST_ONLY=1`, and firewalls are
not blocking ROS 2 DDS discovery traffic.
