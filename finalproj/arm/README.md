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
is still set up around the existing Humble-era workflow:

```bash
cd /home/mkros/robotics/finalproj/arm
./build-humble.sh
source install/setup.bash
```

Then launch it with:

```bash
ros2 launch finalproj_openmanipulator_control system.launch.py
```

Or with OpenCR:

```bash
ros2 launch finalproj_openmanipulator_control system.launch.py \
  hardware_port_name:=/dev/ttyACM0
```
