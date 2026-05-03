import os

from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def as_bool(context, name):
    value = context.perform_substitution(LaunchConfiguration(name)).strip().lower()
    return value in ('1', 'true', 'yes', 'on')


def include_optional_launch(
    context,
    enabled_arg,
    package_arg,
    file_arg,
    label,
    launch_arguments=None,
):
    if not as_bool(context, enabled_arg):
        return [LogInfo(msg=f'Skipping {label}.')]

    package_name = context.perform_substitution(LaunchConfiguration(package_arg)).strip()
    launch_file = context.perform_substitution(LaunchConfiguration(file_arg)).strip()

    try:
        package_share = get_package_share_directory(package_name)
    except PackageNotFoundError:
        return [
            LogInfo(
                msg=(
                    f'Could not find package "{package_name}" for {label}. '
                    f'Install it or override {package_arg}.'
                )
            )
        ]

    launch_path = os.path.join(package_share, 'launch', launch_file)
    if not os.path.exists(launch_path):
        return [
            LogInfo(
                msg=(
                    f'Could not find launch file "{launch_file}" in package '
                    f'"{package_name}" for {label}.'
                )
            )
        ]

    resolved_launch_arguments = {}
    for key, value in (launch_arguments or {}).items():
        resolved = context.perform_substitution(value).strip()
        if resolved:
            resolved_launch_arguments[key] = resolved

    return [
        LogInfo(msg=f'Launching {label}: {package_name}/{launch_file}'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_path),
            launch_arguments=resolved_launch_arguments.items(),
        ),
    ]


def generate_launch_description():
    package_name = 'finalproj_openmanipulator_control'
    controller_config_path = PathJoinSubstitution(
        [FindPackageShare(package_name), 'config', 'controller.yaml']
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    controller_config_arg = LaunchConfiguration('controller_config')
    joy_device_id = LaunchConfiguration('joy_device_id')
    joy_device_name = LaunchConfiguration('joy_device_name')
    joy_device_path = LaunchConfiguration('joy_device_path')
    joy_deadzone = LaunchConfiguration('joy_deadzone')
    joy_autorepeat_rate = LaunchConfiguration('joy_autorepeat_rate')
    recording = LaunchConfiguration('recording')
    auto_start_servo = LaunchConfiguration('auto_start_servo')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'launch_hardware',
                default_value='true',
                description='Include the real OpenManipulator-X hardware bringup launch.',
            ),
            DeclareLaunchArgument(
                'hardware_launch_package',
                default_value='open_manipulator_bringup',
                description='Package containing the OpenManipulator-X hardware launch file.',
            ),
            DeclareLaunchArgument(
                'hardware_launch_file',
                default_value='open_manipulator_x.launch.py',
                description='OpenManipulator-X hardware launch file name.',
            ),
            DeclareLaunchArgument(
                'hardware_port_name',
                default_value='',
                description='Optional serial port for OpenCR-based bringup, such as /dev/ttyACM0.',
            ),
            DeclareLaunchArgument(
                'launch_moveit',
                default_value='true',
                description='Include the OpenManipulator-X MoveIt/RViz launch.',
            ),
            DeclareLaunchArgument(
                'moveit_launch_package',
                default_value='open_manipulator_moveit_config',
                description='Package containing the MoveIt/RViz launch file.',
            ),
            DeclareLaunchArgument(
                'moveit_launch_file',
                default_value='open_manipulator_x_moveit.launch.py',
                description='MoveIt/RViz launch file name.',
            ),
            DeclareLaunchArgument(
                'launch_servo',
                default_value='true',
                description='Include the MoveIt Servo launch file.',
            ),
            DeclareLaunchArgument(
                'servo_launch_package',
                default_value=package_name,
                description='Package containing the MoveIt Servo launch file.',
            ),
            DeclareLaunchArgument(
                'servo_launch_file',
                default_value='servo.launch.py',
                description='MoveIt Servo launch file name.',
            ),
            DeclareLaunchArgument(
                'launch_joy',
                default_value='true',
                description='Launch the built-in raw joystick reader.',
            ),
            DeclareLaunchArgument(
                'launch_controller',
                default_value='true',
                description='Launch the final-project joystick controller node.',
            ),
            DeclareLaunchArgument(
                'joy_device_id',
                default_value='0',
                description='Linux joystick device index, such as 0 for /dev/input/js0.',
            ),
            DeclareLaunchArgument(
                'joy_device_name',
                default_value='',
                description='Optional joystick device name to match.',
            ),
            DeclareLaunchArgument(
                'joy_device_path',
                default_value='/dev/input/by-id/usb-0810_USB_Gamepad-joystick',
                description='Preferred Linux joystick device path.',
            ),
            DeclareLaunchArgument(
                'joy_deadzone',
                default_value='0.05',
                description='Joystick deadzone.',
            ),
            DeclareLaunchArgument(
                'joy_autorepeat_rate',
                default_value='20.0',
                description='Joystick autorepeat rate in Hz.',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use simulation time in the final-project controller.',
            ),
            DeclareLaunchArgument(
                'controller_config',
                default_value=controller_config_path,
                description='YAML file with final-project controller parameters.',
            ),
            DeclareLaunchArgument(
                'recording',
                default_value='false',
                description='Enable LT/RT motion recording and playback controls.',
            ),
            DeclareLaunchArgument(
                'auto_start_servo',
                default_value='true',
                description='Ask the controller to switch and unpause MoveIt Servo on startup.',
            ),
            OpaqueFunction(
                function=include_optional_launch,
                args=[
                    'launch_hardware',
                    'hardware_launch_package',
                    'hardware_launch_file',
                    'OpenManipulator-X hardware',
                    {'port_name': LaunchConfiguration('hardware_port_name')},
                ],
            ),
            OpaqueFunction(
                function=include_optional_launch,
                args=[
                    'launch_moveit',
                    'moveit_launch_package',
                    'moveit_launch_file',
                    'OpenManipulator-X MoveIt/RViz',
                ],
            ),
            OpaqueFunction(
                function=include_optional_launch,
                args=[
                    'launch_servo',
                    'servo_launch_package',
                    'servo_launch_file',
                    'OpenManipulator-X MoveIt Servo',
                    {'use_sim_time': use_sim_time},
                ],
            ),
            Node(
                package=package_name,
                executable='finalproj_raw_joy',
                name='joy_node',
                output='screen',
                parameters=[
                    {
                        'device_id': ParameterValue(joy_device_id, value_type=int),
                        'device_name': joy_device_name,
                        'device_path': joy_device_path,
                        'autorepeat_rate': ParameterValue(joy_autorepeat_rate, value_type=float),
                        'deadzone': ParameterValue(joy_deadzone, value_type=float),
                    }
                ],
                condition=IfCondition(LaunchConfiguration('launch_joy')),
            ),
            Node(
                package=package_name,
                executable='finalproj_joy_controller',
                name='finalproj_joy_controller',
                output='screen',
                parameters=[
                    ParameterFile(controller_config_arg, allow_substs=True),
                    {
                        'use_sim_time': use_sim_time,
                        'recording_enabled': ParameterValue(recording, value_type=bool),
                        'auto_start_servo': ParameterValue(auto_start_servo, value_type=bool),
                    },
                ],
                condition=IfCondition(LaunchConfiguration('launch_controller')),
            ),
        ]
    )
