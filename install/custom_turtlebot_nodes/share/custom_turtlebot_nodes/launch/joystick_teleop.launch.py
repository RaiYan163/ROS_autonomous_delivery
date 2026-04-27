from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    joy_dev_arg = DeclareLaunchArgument(
        "joy_dev",
        default_value="/dev/input/js0",
        description="Joystick device path for joy_node.",
    )

    deadzone_arg = DeclareLaunchArgument(
        "deadzone",
        default_value="0.05",
        description="Deadzone for custom joystick teleop node.",
    )
    cmd_vel_type_arg = DeclareLaunchArgument(
        "cmd_vel_type",
        default_value="twist_stamped",
        description="Velocity message type: 'twist' or 'twist_stamped'.",
    )

    return LaunchDescription(
        [
            joy_dev_arg,
            deadzone_arg,
            cmd_vel_type_arg,
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                parameters=[
                    {
                        "device_name": "",
                        "dev": LaunchConfiguration("joy_dev"),
                        "deadzone": 0.1,
                        "autorepeat_rate": 20.0,
                    }
                ],
            ),
            Node(
                package="custom_turtlebot_nodes",
                executable="joystick_teleop",
                name="joystick_teleop",
                output="screen",
                parameters=[
                    {
                        "deadzone": LaunchConfiguration("deadzone"),
                        "cmd_vel_type": LaunchConfiguration("cmd_vel_type"),
                    }
                ],
            ),
        ]
    )
