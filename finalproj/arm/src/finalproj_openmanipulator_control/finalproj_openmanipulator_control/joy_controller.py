import math
import os
import random
import threading
import time
from pathlib import Path
from typing import Dict
from typing import List
from typing import Optional

import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from control_msgs.action import GripperCommand
from control_msgs.msg import JointJog
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool
from tf2_ros import Buffer
from tf2_ros import TransformListener
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import yaml

try:
    from moveit_msgs.msg import RobotState
    from moveit_msgs.srv import GetPositionIK
    from moveit_msgs.srv import ServoCommandType
except ImportError:  # pragma: no cover - depends on the MoveIt install
    RobotState = None
    GetPositionIK = None
    ServoCommandType = None

try:
    from ros_gz_interfaces.msg import Entity
    from ros_gz_interfaces.srv import SetEntityPose
except ImportError:  # pragma: no cover - optional on hardware
    Entity = None
    SetEntityPose = None


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    return {
        'x': sr * cp * cy - cr * sp * sy,
        'y': cr * sp * cy + sr * cp * sy,
        'z': cr * cp * sy - sr * sp * cy,
        'w': cr * cp * cy + sr * sp * sy,
    }


def yaw_quaternion(yaw):
    return quaternion_from_euler(0.0, 0.0, yaw)


class FinalProjJoyController(Node):
    def __init__(self):
        super().__init__('finalproj_joy_controller')

        self.arm_joints = list(
            self.declare_parameter('arm_joints', ['joint1', 'joint2', 'joint3', 'joint4']).value
        )
        self.gripper_joints = list(
            self.declare_parameter(
                'gripper_joints',
                ['gripper_left_joint', 'gripper_right_joint_mimic'],
            ).value
        )
        self.select_joint_buttons = list(
            self.declare_parameter('select_joint_buttons', [0, 1, 2, 3]).value
        )
        self.selected_joint_axis = int(self.declare_parameter('selected_joint_axis', 1).value)
        self.dpad_horizontal_axis = int(self.declare_parameter('dpad_horizontal_axis', 6).value)
        self.dpad_vertical_axis = int(self.declare_parameter('dpad_vertical_axis', 7).value)
        self.deadzone = float(self.declare_parameter('deadzone', 0.18).value)
        self.joint_velocity_scale = float(
            self.declare_parameter('joint_velocity_scale', 0.60).value
        )
        self.gripper_hold_rate = float(
            self.declare_parameter('gripper_hold_rate', 0.55).value
        )
        self.direct_publish_rate = float(self.declare_parameter('direct_publish_rate', 50.0).value)
        self.motion_point_duration = float(
            self.declare_parameter('motion_point_duration', 2.0).value
        )
        self.motion_sample_rate_hz = float(
            self.declare_parameter('motion_sample_rate_hz', 20.0).value
        )
        self.playback_finish_margin_sec = float(
            self.declare_parameter('playback_finish_margin_sec', 0.5).value
        )
        self.playback_start_move_duration = float(
            self.declare_parameter('playback_start_move_duration', 3.0).value
        )
        self.recording_enabled = bool(self.declare_parameter('recording_enabled', False).value)
        self.recording_file = os.path.expanduser(
            str(
                self.declare_parameter(
                    'recording_file',
                    '~/.ros/finalproj_openmanipulator_motion.yaml',
                ).value
            )
        )

        self.base_frame = str(self.declare_parameter('base_frame', 'world').value)
        self.ee_link = str(self.declare_parameter('ee_link', 'end_effector_link').value)
        self.group_name = str(self.declare_parameter('move_group_name', 'arm').value)
        self.ik_link_name = str(self.declare_parameter('ik_link_name', self.ee_link).value)

        self.joint_jog_topic = str(
            self.declare_parameter('joint_jog_topic', '/servo_node/delta_joint_cmds').value
        )
        self.trajectory_action_name = str(
            self.declare_parameter(
                'trajectory_action_name',
                '/arm_controller/follow_joint_trajectory',
            ).value
        )
        self.trajectory_topic_name = str(
            self.declare_parameter(
                'trajectory_topic_name',
                self.default_trajectory_topic(self.trajectory_action_name),
            ).value
        )
        self.gripper_action_name = str(
            self.declare_parameter('gripper_action_name', '/gripper_controller/gripper_cmd').value
        )
        self.gripper_open_position = float(
            self.declare_parameter('gripper_open_position', 0.019).value
        )
        self.gripper_closed_position = float(
            self.declare_parameter('gripper_closed_position', -0.010).value
        )
        self.gripper_motion_tolerance = float(
            self.declare_parameter('gripper_motion_tolerance', 0.002).value
        )
        self.gripper_settle_sec = float(self.declare_parameter('gripper_settle_sec', 0.45).value)
        self.auto_start_servo = bool(self.declare_parameter('auto_start_servo', True).value)

        self.world_name = str(self.declare_parameter('world_name', 'default').value)
        self.cube_name = str(self.declare_parameter('cube_name', 'finalproj_cube').value)
        self.cube_x = float(self.declare_parameter('cube_x', 0.22).value)
        self.cube_y = float(self.declare_parameter('cube_y', 0.0).value)
        self.cube_z = float(self.declare_parameter('cube_z', 0.018).value)
        self.pick_pregrasp_z = float(self.declare_parameter('pick_pregrasp_z', 0.13).value)
        self.pick_grasp_z = float(self.declare_parameter('pick_grasp_z', 0.055).value)

        self.button_close_gripper = int(self.declare_parameter('button_close_gripper', 0).value)
        self.button_open_gripper = int(self.declare_parameter('button_open_gripper', 1).value)
        self.button_record_joint = int(self.declare_parameter('button_record_joint', 2).value)
        self.button_record_ee = int(self.declare_parameter('button_record_ee', 3).value)
        self.button_play_joint_pose = int(
            self.declare_parameter('button_play_joint_pose', 4).value
        )
        self.button_play_joint_motion = int(
            self.declare_parameter('button_play_joint_motion', 5).value
        )
        self.button_save_motion = int(self.declare_parameter('button_save_motion', 6).value)
        self.button_play_all_motions = int(
            self.declare_parameter('button_play_all_motions', 7).value
        )
        self.button_toggle_dance = int(self.declare_parameter('button_toggle_dance', 8).value)
        self.button_toggle_pick_task = int(
            self.declare_parameter('button_toggle_pick_task', 9).value
        )
        self.button_play_ee_pose = int(self.declare_parameter('button_play_ee_pose', 10).value)
        self.button_play_ee_sequence = int(
            self.declare_parameter('button_play_ee_sequence', 11).value
        )
        self.button_print_status = int(self.declare_parameter('button_print_status', 12).value)
        self.button_record_motion = int(self.declare_parameter('button_record_motion', 6).value)
        self.button_play_motion = int(self.declare_parameter('button_play_motion', 7).value)
        self.left_trigger_axis = int(self.declare_parameter('left_trigger_axis', -1).value)
        self.right_trigger_axis = int(self.declare_parameter('right_trigger_axis', -1).value)
        self.trigger_threshold = float(self.declare_parameter('trigger_threshold', 0.5).value)
        self.trigger_release_debounce_sec = float(
            self.declare_parameter('trigger_release_debounce_sec', 0.25).value
        )

        self.joint_pub = self.create_publisher(JointJog, self.joint_jog_topic, 10)
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            self.trajectory_topic_name,
            10,
        )
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
        )

        self.servo_pause_client = self.create_client(SetBool, '/servo_node/pause_servo')
        self.servo_command_type_client = (
            self.create_client(ServoCommandType, '/servo_node/switch_command_type')
            if ServoCommandType is not None
            else None
        )
        self.traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.trajectory_action_name,
        )
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            self.gripper_action_name,
        )
        self.ik_client = (
            self.create_client(GetPositionIK, '/compute_ik') if GetPositionIK is not None else None
        )
        self.set_entity_pose_client = (
            self.create_client(SetEntityPose, f'/world/{self.world_name}/set_pose')
            if SetEntityPose is not None
            else None
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.current_joint_state: Optional[JointState] = None
        self.last_buttons: List[int] = []
        self.last_axes: List[float] = []
        self.manual_joint_names: List[str] = []
        self.manual_velocities: List[float] = []
        self.behavior_busy = False
        self.last_joy_time = None

        self.saved_joint_poses: List[Dict[str, float]] = []
        self.current_joint_motion: List[Dict[str, float]] = []
        self.saved_joint_motions: List[List[Dict[str, float]]] = []
        self.saved_ee_poses: List[Dict[str, object]] = []
        self.current_ee_sequence: List[Dict[str, object]] = []
        self.recorded_motion: List[Dict[str, object]] = []
        self.recording_active = False
        self.recording_start_time = None
        self.last_record_sample_time = None
        self.recording_lock = threading.Lock()
        self.playback_stop = threading.Event()
        self.playback_thread: Optional[threading.Thread] = None
        self.record_trigger_state = {'down': False, 'false_since': time.monotonic()}
        self.play_trigger_state = {'down': False, 'false_since': time.monotonic()}

        self.random_dance_stop = threading.Event()
        self.random_dance_thread: Optional[threading.Thread] = None
        self.pick_task_stop = threading.Event()
        self.pick_task_thread: Optional[threading.Thread] = None
        self.pick_cycle_source_sign = 1.0
        self.pick_cycle_initialized = False

        self.load_recording()

        if self.auto_start_servo:
            threading.Thread(target=self.start_servo_after_delay, daemon=True).start()

        self.get_logger().info('Final project OpenManipulator-X joystick controller ready.')
        self.get_logger().info(
            'Hold selector buttons 1-4 and move axis 1 for manual joint control; '
            'use L1/R1 for gripper hold.'
        )
        if self.recording_enabled:
            self.get_logger().info('Recording enabled: press LT to start/stop recording; press RT to play/stop.')
        else:
            self.get_logger().info('Recording disabled. Press RT to play/stop an existing recording.')

    def joint_state_callback(self, msg):
        self.current_joint_state = msg
        self.record_motion_sample(msg)

    def start_servo_after_delay(self):
        time.sleep(1.0)
        if self.servo_command_type_client is None:
            self.get_logger().warning('MoveIt Servo command type service is unavailable.')
            return

        deadline = time.monotonic() + 30.0
        while rclpy.ok() and time.monotonic() < deadline:
            command_type_ready = self.servo_command_type_client.wait_for_service(timeout_sec=0.5)
            pause_ready = self.servo_pause_client.wait_for_service(timeout_sec=0.5)
            if command_type_ready and pause_ready:
                if self.start_servo():
                    return
                time.sleep(1.0)
            else:
                time.sleep(0.5)

        self.get_logger().warning(
            'MoveIt Servo never became available for auto-start. '
            'Check that the arm-side launch is running and ROS_DOMAIN_ID matches.'
        )

    def joy_callback(self, msg):
        axes = list(msg.axes)
        buttons = list(msg.buttons)
        now = self.get_clock().now()
        if self.last_joy_time is None:
            dt = 0.02
        else:
            dt = (now - self.last_joy_time).nanoseconds * 1e-9
            dt = max(0.001, min(dt, 0.1))
        self.last_joy_time = now

        now_monotonic = time.monotonic()
        if self.recording_enabled:
            if self.rising_record_trigger(buttons, axes, now_monotonic):
                self.toggle_motion_recording()
        if self.rising_play_trigger(buttons, axes, now_monotonic):
            self.toggle_motion_playback()

        previous_joint_names = list(self.manual_joint_names)
        joint_names = []
        velocities = []
        selected_joint_index = self.selected_arm_joint_index(buttons)
        if selected_joint_index is not None and selected_joint_index < len(self.arm_joints):
            value = self.axis_value(axes, self.selected_joint_axis)
            if value != 0.0:
                joint_names.append(self.arm_joints[selected_joint_index])
                velocities.append(value * self.joint_velocity_scale)

        was_manual_active = bool(previous_joint_names)
        self.manual_joint_names = joint_names
        self.manual_velocities = velocities

        if not self.behavior_busy:
            if self.manual_joint_names:
                self.publish_joint_jog(self.manual_joint_names, self.manual_velocities)
            elif was_manual_active:
                self.publish_joint_jog(previous_joint_names, [0.0] * len(previous_joint_names))
            self.update_gripper_hold(buttons, dt)

        if self.dpad_rising(axes, axis_index=self.dpad_horizontal_axis, direction=1):
            self.log_recording_counts()
        if self.dpad_rising(axes, axis_index=self.dpad_horizontal_axis, direction=-1):
            self.clear_recorded_motion()

        if self.rising_edge(buttons, self.button_toggle_dance):
            self.toggle_random_dance()
        if self.rising_edge(buttons, self.button_toggle_pick_task):
            self.toggle_pick_task()
        if self.rising_edge(buttons, self.button_print_status):
            self.print_current_state()

        self.last_buttons = buttons
        self.last_axes = axes

    def selected_arm_joint_index(self, buttons):
        for mapped_index, button_index in enumerate(self.select_joint_buttons):
            if button_index < 0 or button_index >= len(buttons):
                continue
            if buttons[button_index] == 1:
                if mapped_index < len(self.arm_joints):
                    return mapped_index
        return None

    def axis_value(self, axes, index):
        if index < 0 or index >= len(axes):
            return 0.0
        value = float(axes[index])
        if abs(value) < self.deadzone:
            return 0.0
        sign = 1.0 if value > 0.0 else -1.0
        magnitude = (abs(value) - self.deadzone) / max(1e-6, (1.0 - self.deadzone))
        return sign * min(1.0, magnitude)

    def rising_edge(self, buttons, index):
        if index < 0 or index >= len(buttons):
            return False
        previous = self.last_buttons[index] if index < len(self.last_buttons) else 0
        return buttons[index] == 1 and previous == 0

    def dpad_rising(self, axes, axis_index, direction):
        if axis_index < 0 or axis_index >= len(axes):
            return False
        current = axes[axis_index]
        previous = self.last_axes[axis_index] if axis_index < len(self.last_axes) else 0.0
        if direction > 0:
            return current > 0.5 and previous <= 0.5
        return current < -0.5 and previous >= -0.5

    def trigger_pressed(self, buttons, axes, button_index, axis_index):
        button_down = 0 <= button_index < len(buttons) and buttons[button_index] == 1
        if button_down:
            return True
        if axis_index < 0 or axis_index >= len(axes):
            return False
        return float(axes[axis_index]) >= self.trigger_threshold

    def record_trigger_pressed(self, buttons, axes):
        return self.trigger_pressed(
            buttons,
            axes,
            self.button_record_motion,
            self.left_trigger_axis,
        )

    def play_trigger_pressed(self, buttons, axes):
        return self.trigger_pressed(
            buttons,
            axes,
            self.button_play_motion,
            self.right_trigger_axis,
        )

    def debounced_trigger_edge(self, raw_pressed, state, now):
        if raw_pressed:
            state['false_since'] = None
            if not state['down']:
                state['down'] = True
                return True
            return False

        if state['false_since'] is None:
            state['false_since'] = now
        if state['down'] and now - state['false_since'] >= self.trigger_release_debounce_sec:
            state['down'] = False
        return False

    def rising_record_trigger(self, buttons, axes, now):
        return self.debounced_trigger_edge(
            self.record_trigger_pressed(buttons, axes),
            self.record_trigger_state,
            now,
        )

    def rising_play_trigger(self, buttons, axes, now):
        return self.debounced_trigger_edge(
            self.play_trigger_pressed(buttons, axes),
            self.play_trigger_state,
            now,
        )

    def update_gripper_hold(self, buttons, dt):
        if self.behavior_busy:
            return

        open_held = 0 <= self.button_open_gripper < len(buttons) and buttons[self.button_open_gripper] == 1
        close_held = 0 <= self.button_close_gripper < len(buttons) and buttons[self.button_close_gripper] == 1
        if open_held == close_held:
            return

        current_position = self.current_gripper_position()
        if current_position is None:
            return

        span = self.gripper_open_position - self.gripper_closed_position
        step = self.gripper_hold_rate * span * dt
        if open_held:
            target_position = min(current_position + step, self.gripper_open_position)
            if target_position <= current_position + 1e-7:
                return
        else:
            target_position = max(current_position - step, self.gripper_closed_position)
            if target_position >= current_position - 1e-7:
                return

        self.send_gripper_goal_async(target_position)

    def publish_joint_jog(self, joint_names, velocities):
        if self.behavior_busy or not joint_names or len(joint_names) != len(velocities):
            return

        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.joint_names = list(joint_names)
        msg.velocities = list(velocities)
        msg.duration = 1.0 / self.direct_publish_rate
        self.joint_pub.publish(msg)

    def run_worker(self, label, target, *args):
        if self.behavior_busy:
            self.get_logger().warning(f'Cannot start {label}; another behavior is running.')
            return

        def wrapped():
            self.behavior_busy = True
            try:
                target(*args)
            except Exception as exc:  # pragma: no cover - defensive runtime logging
                self.get_logger().error(f'{label} failed: {exc}')
            finally:
                self.behavior_busy = False

        threading.Thread(target=wrapped, daemon=True).start()

    def wait_future(self, future, timeout_sec):
        start = time.monotonic()
        while rclpy.ok() and not future.done():
            if timeout_sec is not None and time.monotonic() - start > timeout_sec:
                return False
            time.sleep(0.02)
        return future.done()

    def default_trajectory_topic(self, action_name):
        if action_name.endswith('/follow_joint_trajectory'):
            return action_name[: -len('/follow_joint_trajectory')] + '/joint_trajectory'
        return '/arm_controller/joint_trajectory'

    def start_servo(self):
        if self.servo_command_type_client is None:
            self.get_logger().warning('MoveIt Servo command type service is unavailable.')
            return False
        if not self.servo_command_type_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('MoveIt Servo command type service is not available yet.')
            return False
        if not self.servo_pause_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('MoveIt Servo pause service is not available yet.')
            return False
        command_type_request = ServoCommandType.Request()
        command_type_request.command_type = ServoCommandType.Request.JOINT_JOG
        command_type_future = self.servo_command_type_client.call_async(command_type_request)
        if not self.wait_future(command_type_future, 2.0):
            self.get_logger().warning('Timed out setting MoveIt Servo command type.')
            return False
        command_type_result = command_type_future.result()
        if command_type_result is None or not command_type_result.success:
            self.get_logger().warning('Could not set MoveIt Servo command type to joint jog.')
            return False
        request = SetBool.Request()
        request.data = False
        future = self.servo_pause_client.call_async(request)
        if not self.wait_future(future, 2.0):
            self.get_logger().warning('Timed out unpausing MoveIt Servo.')
            return False
        result = future.result()
        if result is None or not result.success:
            message = result.message if result is not None else 'no response'
            self.get_logger().warning(f'Could not unpause MoveIt Servo: {message}')
            return False
        self.get_logger().info('MoveIt Servo is ready for joystick control.')
        return True

    def stop_servo(self):
        if not self.servo_pause_client.wait_for_service(timeout_sec=1.0):
            return False
        request = SetBool.Request()
        request.data = True
        future = self.servo_pause_client.call_async(request)
        if not self.wait_future(future, 2.0):
            return False
        result = future.result()
        return bool(result is not None and result.success)

    def current_joint_map(self):
        if self.current_joint_state is None:
            return {}
        return {
            name: position
            for name, position in zip(
                self.current_joint_state.name,
                self.current_joint_state.position,
            )
        }

    def get_current_joint_pose(self):
        joint_map = self.current_joint_map()
        if not joint_map:
            self.get_logger().warning('No /joint_states received yet.')
            return None

        pose = {}
        for joint in self.arm_joints:
            if joint not in joint_map:
                self.get_logger().warning(f'Missing joint in /joint_states: {joint}')
                return None
            pose[joint] = float(joint_map[joint])

        for joint in self.gripper_joints:
            if joint in joint_map:
                pose[joint] = float(joint_map[joint])
                break

        return pose

    def extract_motion_waypoint(self, msg):
        joint_map = {
            name: position
            for name, position in zip(msg.name, msg.position)
        }
        positions_arm = []
        for joint in self.arm_joints:
            if joint not in joint_map:
                return None
            positions_arm.append(float(joint_map[joint]))

        gripper = None
        for joint in self.gripper_joints:
            if joint in joint_map:
                gripper = float(joint_map[joint])
                break

        return {
            'positions_arm': positions_arm,
            'gripper': gripper,
            'time_from_start_sec': 0.0,
        }

    def record_motion_sample(self, msg):
        with self.recording_lock:
            if not self.recording_active:
                return

            now = self.get_clock().now()
            if self.last_record_sample_time is not None:
                dt = (now - self.last_record_sample_time).nanoseconds * 1e-9
                min_interval = 1.0 / max(1.0, self.motion_sample_rate_hz)
                if dt < min_interval:
                    return

            waypoint = self.extract_motion_waypoint(msg)
            if waypoint is None:
                return

            if self.recording_start_time is None:
                self.recording_start_time = now
            self.last_record_sample_time = now
            waypoint['time_from_start_sec'] = (
                now - self.recording_start_time
            ).nanoseconds * 1e-9
            self.recorded_motion.append(waypoint)

    def start_motion_recording(self):
        if self.playback_thread and self.playback_thread.is_alive():
            self.get_logger().warning('Cannot record while playback is running.')
            return
        with self.recording_lock:
            self.recorded_motion.clear()
            self.recording_active = True
            self.recording_start_time = None
            self.last_record_sample_time = None
        self.get_logger().info('Motion recording started. Press LT again to stop and save.')

    def toggle_motion_recording(self):
        if self.recording_active:
            self.stop_motion_recording(save=True)
        else:
            self.start_motion_recording()

    def stop_motion_recording(self, save=True):
        with self.recording_lock:
            if not self.recording_active:
                return
            self.recording_active = False
            waypoints = [dict(wp) for wp in self.recorded_motion]

        if not save:
            self.get_logger().info('Motion recording stopped without saving.')
            return

        if len(waypoints) < 2:
            self.get_logger().warning(
                'Need at least 2 motion samples. Press LT to start recording, move the arm, then press LT again to save.'
            )
            return

        offset = float(waypoints[0]['time_from_start_sec'])
        for waypoint in waypoints:
            waypoint['time_from_start_sec'] = (
                float(waypoint['time_from_start_sec']) - offset
            )
        self.recorded_motion = waypoints
        self.save_recording()
        duration = waypoints[-1]['time_from_start_sec']
        self.get_logger().info(
            f'Saved {len(waypoints)} motion samples over {duration:.2f}s to {self.recording_file}.'
        )

    def clear_recorded_motion(self):
        if self.recording_active:
            self.stop_motion_recording(save=False)
        self.recorded_motion.clear()
        path = Path(self.recording_file)
        if path.exists():
            path.unlink()
        self.get_logger().info('Cleared recorded motion.')

    def toggle_motion_playback(self):
        if self.playback_thread and self.playback_thread.is_alive():
            self.playback_stop.set()
            self.get_logger().info('Playback stop requested.')
            return
        self.playback_stop.clear()
        self.playback_thread = threading.Thread(target=self.play_recorded_motion, daemon=True)
        self.playback_thread.start()

    def play_recorded_motion(self):
        if self.recording_active:
            self.get_logger().warning('Cannot play while recording is active.')
            return False
        if not self.recorded_motion:
            self.load_recording()
        if len(self.recorded_motion) < 2:
            self.get_logger().warning('No recorded motion to play. Hold LT to record first.')
            return False

        self.behavior_busy = True
        try:
            return self.execute_recorded_motion(self.recorded_motion)
        finally:
            self.behavior_busy = False
            self.playback_stop.clear()

    def execute_recorded_motion(self, waypoints):
        trajectory = self.build_recorded_trajectory(waypoints)
        if not trajectory.points:
            self.get_logger().warning('Recorded motion is empty.')
            return False

        self.send_recorded_arm_trajectory(trajectory)
        self.replay_recorded_gripper(waypoints)

        if self.playback_stop.is_set():
            self.get_logger().info('Playback stopped.')
            return False
        self.get_logger().info('Playback complete.')
        return True

    def build_recorded_trajectory(self, waypoints):
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = list(self.arm_joints)

        for waypoint in waypoints:
            positions = waypoint.get('positions_arm', [])
            if len(positions) != len(self.arm_joints):
                self.get_logger().warning('Skipping malformed recorded waypoint.')
                continue
            tfs = (
                max(0.0, float(waypoint.get('time_from_start_sec', 0.0)))
                + self.playback_start_move_duration
            )
            point = JointTrajectoryPoint()
            point.positions = [float(value) for value in positions]
            point.velocities = [0.0] * len(self.arm_joints)
            point.accelerations = [0.0] * len(self.arm_joints)
            point.time_from_start = self.duration_msg(tfs)
            trajectory.points.append(point)

        return trajectory

    def send_recorded_arm_trajectory(self, trajectory):
        if self.traj_client.wait_for_server(timeout_sec=0.5):
            goal_msg = FollowJointTrajectory.Goal()
            goal_msg.trajectory = trajectory
            self.traj_client.send_goal_async(goal_msg)
            self.get_logger().info('Sent recorded arm trajectory to action server.')
            return True

        self.trajectory_pub.publish(trajectory)
        self.get_logger().warning(
            'Trajectory action server unavailable; published recorded motion to controller topic.'
        )
        return True

    def replay_recorded_gripper(self, waypoints):
        start = time.monotonic()
        last_index = -1
        total_duration = (
            float(waypoints[-1].get('time_from_start_sec', 0.0))
            + self.playback_start_move_duration
        )
        while rclpy.ok() and not self.playback_stop.is_set():
            elapsed = time.monotonic() - start
            sent_any = False
            for index, waypoint in enumerate(waypoints):
                if index <= last_index:
                    continue
                target_time = (
                    float(waypoint.get('time_from_start_sec', 0.0))
                    + self.playback_start_move_duration
                )
                if elapsed < target_time:
                    break
                gripper = waypoint.get('gripper')
                if gripper is not None:
                    self.send_gripper_goal_async(float(gripper))
                last_index = index
                sent_any = True
            if elapsed >= total_duration + self.playback_finish_margin_sec:
                return
            if not sent_any:
                time.sleep(0.02)

    def record_joint_pose(self):
        pose = self.get_current_joint_pose()
        if pose is None:
            return
        self.saved_joint_poses.append(pose)
        self.current_joint_motion.append(pose)
        self.save_recordings()
        self.get_logger().info(
            f'Recorded joint pose #{len(self.saved_joint_poses)} '
            f'(current motion length {len(self.current_joint_motion)}).'
        )

    def save_current_joint_motion(self):
        if not self.current_joint_motion:
            self.get_logger().warning('No current joint motion points to save.')
            return
        self.saved_joint_motions.append(list(self.current_joint_motion))
        self.current_joint_motion.clear()
        self.save_recordings()
        self.get_logger().info(f'Saved joint motion #{len(self.saved_joint_motions)}.')

    def record_ee_pose(self):
        pose = self.lookup_ee_pose()
        if pose is None:
            return
        self.saved_ee_poses.append(pose)
        self.current_ee_sequence.append(pose)
        self.save_recordings()
        position = pose['position']
        self.get_logger().info(
            'Recorded EE pose '
            f"x={position['x']:.3f}, y={position['y']:.3f}, z={position['z']:.3f} "
            f'(sequence length {len(self.current_ee_sequence)}).'
        )

    def lookup_ee_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_link,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5),
            )
        except Exception as exc:
            self.get_logger().warning(f'Cannot record EE pose yet: {exc}')
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return {
            'frame_id': self.base_frame,
            'position': {
                'x': float(translation.x),
                'y': float(translation.y),
                'z': float(translation.z),
            },
            'orientation': {
                'x': float(rotation.x),
                'y': float(rotation.y),
                'z': float(rotation.z),
                'w': float(rotation.w),
            },
        }

    def play_last_joint_pose(self):
        if not self.saved_joint_poses:
            self.get_logger().warning('No recorded joint pose to play.')
            return False
        return self.execute_joint_poses([self.saved_joint_poses[-1]], self.motion_point_duration)

    def play_last_joint_motion(self):
        if self.current_joint_motion:
            motion = list(self.current_joint_motion)
        elif self.saved_joint_motions:
            motion = self.saved_joint_motions[-1]
        elif self.saved_joint_poses:
            motion = list(self.saved_joint_poses)
        else:
            self.get_logger().warning('No recorded joint motion to play.')
            return False
        return self.execute_joint_poses(motion, self.motion_point_duration)

    def play_all_joint_motions(self):
        if not self.saved_joint_motions:
            self.get_logger().warning('No saved joint motions to play.')
            return False
        for index, motion in enumerate(self.saved_joint_motions, start=1):
            if not rclpy.ok():
                return False
            self.get_logger().info(f'Playing saved motion {index}/{len(self.saved_joint_motions)}.')
            if not self.execute_joint_poses(motion, self.motion_point_duration):
                return False
        return True

    def play_last_ee_pose(self):
        if not self.saved_ee_poses:
            self.get_logger().warning('No recorded EE pose to play.')
            return False
        return self.move_to_ee_pose(self.saved_ee_poses[-1], self.motion_point_duration)

    def play_ee_sequence(self):
        poses = self.current_ee_sequence if self.current_ee_sequence else self.saved_ee_poses
        if not poses:
            self.get_logger().warning('No recorded EE target sequence to play.')
            return False
        for pose in poses:
            if not self.move_to_ee_pose(pose, self.motion_point_duration):
                return False
        return True

    def execute_joint_poses(self, poses, point_duration):
        points = []
        for pose in poses:
            point_positions = []
            for joint in self.arm_joints:
                if joint not in pose:
                    self.get_logger().error(f'Recorded pose is missing {joint}.')
                    return False
                point_positions.append(float(pose[joint]))
            points.append(point_positions)
        return self.send_joint_trajectory(points, point_duration)

    def build_joint_trajectory(self, points, point_duration):
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = list(self.arm_joints)

        for index, positions in enumerate(points, start=1):
            point = JointTrajectoryPoint()
            point.positions = list(positions)
            point.velocities = [0.0] * len(self.arm_joints)
            point.accelerations = [0.0] * len(self.arm_joints)
            point.time_from_start = self.duration_msg(point_duration * index)
            trajectory.points.append(point)

        return trajectory

    def wait_for_joint_positions(self, target_positions, timeout_sec, tolerance=0.08):
        start = time.monotonic()
        while rclpy.ok():
            joint_map = self.current_joint_map()
            if all(
                joint in joint_map and abs(float(joint_map[joint]) - target) <= tolerance
                for joint, target in zip(self.arm_joints, target_positions)
            ):
                return True
            if timeout_sec is not None and time.monotonic() - start > timeout_sec:
                return False
            time.sleep(0.05)
        return False

    def current_gripper_position(self):
        joint_map = self.current_joint_map()
        joint_name = self.gripper_joints[0]
        if joint_name not in joint_map:
            return None
        return float(joint_map[joint_name])

    def wait_for_gripper_position(self, target_position, timeout_sec):
        start = time.monotonic()
        while rclpy.ok():
            current_position = self.current_gripper_position()
            if current_position is not None and abs(current_position - target_position) <= (
                self.gripper_motion_tolerance
            ):
                return True
            if timeout_sec is not None and time.monotonic() - start > timeout_sec:
                return False
            time.sleep(0.05)
        return False

    def publish_joint_trajectory_fallback(self, trajectory, expected_duration_sec):
        if not trajectory.points:
            self.get_logger().error('Cannot publish an empty joint trajectory.')
            return False

        self.trajectory_pub.publish(trajectory)
        self.get_logger().warning(
            'Published trajectory directly to the controller topic because the '
            'action handshake did not complete.'
        )

        target_positions = list(trajectory.points[-1].positions)
        if self.wait_for_joint_positions(target_positions, expected_duration_sec + 4.0):
            self.get_logger().info('Trajectory execution complete via topic fallback.')
            return True

        self.get_logger().error('Timed out waiting for topic-published trajectory execution.')
        return False

    def send_joint_trajectory(self, points, point_duration):
        trajectory = self.build_joint_trajectory(points, point_duration)
        expected_duration_sec = point_duration * len(points)

        if not self.traj_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warning(
                f'Trajectory action server is not available: {self.trajectory_action_name}'
            )
            return self.publish_joint_trajectory_fallback(trajectory, expected_duration_sec)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        send_future = self.traj_client.send_goal_async(goal_msg)
        if not self.wait_future(send_future, 5.0):
            self.get_logger().warning('Timed out sending trajectory goal.')
            return self.publish_joint_trajectory_fallback(trajectory, expected_duration_sec)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warning('Trajectory goal rejected.')
            return self.publish_joint_trajectory_fallback(trajectory, expected_duration_sec)

        result_future = goal_handle.get_result_async()
        if not self.wait_future(result_future, expected_duration_sec + 8.0):
            self.get_logger().error('Timed out waiting for trajectory execution.')
            return False

        self.get_logger().info('Trajectory execution complete.')
        return True

    def send_gripper_goal(self, position):
        if not self.gripper_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                f'Gripper action server is not available: {self.gripper_action_name}'
            )
            return False

        start_position = self.current_gripper_position()
        closing_motion = start_position is None or float(position) < start_position

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = float(position)
        goal_msg.command.max_effort = 10.0

        send_future = self.gripper_client.send_goal_async(goal_msg)
        if not self.wait_future(send_future, 3.0):
            self.get_logger().error('Timed out sending gripper goal.')
            return False

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected.')
            return False

        result_future = goal_handle.get_result_async()
        result_ready = self.wait_future(result_future, 4.0)
        reached_goal = False
        stalled = False
        if result_ready:
            result_response = result_future.result()
            action_result = None if result_response is None else result_response.result
            if action_result is not None:
                reached_goal = bool(action_result.reached_goal)
                stalled = bool(action_result.stalled)
        else:
            self.get_logger().warning(
                'Timed out waiting for gripper action result; checking joint state instead.'
            )

        position_reached = self.wait_for_gripper_position(position, 2.0)
        if not (position_reached or reached_goal or (closing_motion and stalled)):
            current_position = self.current_gripper_position()
            self.get_logger().error(
                'Gripper did not reach the requested state '
                f'(target={position:.4f}, current={current_position}).'
            )
            return False

        if self.gripper_settle_sec > 0.0:
            time.sleep(self.gripper_settle_sec)
        self.get_logger().info(f'Gripper command sent: {position:.4f}.')
        return True

    def send_gripper_goal_async(self, position):
        if not self.gripper_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warning(
                f'Gripper action server is not available: {self.gripper_action_name}'
            )
            return False

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = float(position)
        goal_msg.command.max_effort = 10.0
        self.gripper_client.send_goal_async(goal_msg)
        return True

    def move_to_ee_pose(self, pose_dict, duration_sec):
        joint_positions = self.compute_ik(pose_dict)
        if joint_positions is None:
            return False
        return self.send_joint_trajectory([joint_positions], duration_sec)

    def compute_ik(self, pose_dict):
        if self.ik_client is None or GetPositionIK is None or RobotState is None:
            self.get_logger().error('MoveIt IK service type is unavailable on this install.')
            return None
        if not self.ik_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('MoveIt /compute_ik service is not available.')
            return None

        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.ik_link_name = self.ik_link_name
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout.sec = 2
        req.ik_request.robot_state = self.seed_robot_state()
        req.ik_request.pose_stamped = self.pose_stamped_from_dict(pose_dict)

        future = self.ik_client.call_async(req)
        if not self.wait_future(future, 4.0):
            self.get_logger().error('Timed out waiting for /compute_ik.')
            return None

        result = future.result()
        if result is None or result.error_code.val != 1:
            error_code = None if result is None else result.error_code.val
            self.get_logger().error(f'IK failed; MoveIt error code: {error_code}.')
            return None

        joint_map = {
            name: position
            for name, position in zip(
                result.solution.joint_state.name,
                result.solution.joint_state.position,
            )
        }
        if any(joint not in joint_map for joint in self.arm_joints):
            self.get_logger().error('IK solution did not include all arm joints.')
            return None
        return [float(joint_map[joint]) for joint in self.arm_joints]

    def seed_robot_state(self):
        robot_state = RobotState()
        if self.current_joint_state is not None:
            robot_state.joint_state = self.current_joint_state
        return robot_state

    def pose_stamped_from_dict(self, pose_dict):
        pose = PoseStamped()
        pose.header.frame_id = str(pose_dict.get('frame_id', self.base_frame))
        pose.header.stamp = self.get_clock().now().to_msg()

        position = pose_dict['position']
        orientation = pose_dict['orientation']
        pose.pose.position.x = float(position['x'])
        pose.pose.position.y = float(position['y'])
        pose.pose.position.z = float(position['z'])
        pose.pose.orientation.x = float(orientation['x'])
        pose.pose.orientation.y = float(orientation['y'])
        pose.pose.orientation.z = float(orientation['z'])
        pose.pose.orientation.w = float(orientation['w'])
        return pose

    def toggle_random_dance(self):
        if self.random_dance_thread and self.random_dance_thread.is_alive():
            self.random_dance_stop.set()
            self.get_logger().info('Stopping random dance after current motion.')
            return

        self.random_dance_stop.clear()
        self.random_dance_thread = threading.Thread(target=self.random_dance_loop, daemon=True)
        self.random_dance_thread.start()
        self.get_logger().info('Started random dance behavior.')

    def random_dance_loop(self):
        self.behavior_busy = True
        try:
            while rclpy.ok() and not self.random_dance_stop.is_set():
                motion = self.generate_random_dance_motion()
                self.saved_joint_motions.append(motion)
                if len(self.saved_joint_motions) > 20:
                    self.saved_joint_motions = self.saved_joint_motions[-20:]
                self.execute_joint_poses(motion, 1.6)
        finally:
            self.behavior_busy = False
            self.random_dance_stop.clear()

    def generate_random_dance_motion(self):
        limits = [
            (-2.4, 2.4),
            (-1.2, 1.1),
            (-1.2, 1.2),
            (-1.5, 1.5),
        ]
        motion = []
        for _ in range(5):
            pose = {}
            for joint, (lower, upper) in zip(self.arm_joints, limits):
                pose[joint] = random.uniform(lower, upper)
            motion.append(pose)
        return motion

    def toggle_pick_task(self):
        if self.pick_task_thread and self.pick_task_thread.is_alive():
            self.pick_task_stop.set()
            self.get_logger().info('Stopping pick-rotate-drop after current cycle.')
            return

        self.pick_cycle_source_sign = 1.0
        self.pick_cycle_initialized = False
        self.pick_task_stop.clear()
        self.pick_task_thread = threading.Thread(target=self.pick_task_loop, daemon=True)
        self.pick_task_thread.start()
        self.get_logger().info('Started autonomous pick-rotate-drop behavior.')

    def pick_task_loop(self):
        self.behavior_busy = True
        try:
            while rclpy.ok() and not self.pick_task_stop.is_set():
                if not self.run_pick_rotate_drop_cycle():
                    self.get_logger().warning(
                        'Pick task cycle used fallback or failed; retrying after pause.'
                    )
                    time.sleep(1.0)
        finally:
            self.behavior_busy = False
            self.pick_task_stop.clear()

    def run_pick_rotate_drop_cycle(self):
        source_x = self.cube_x * self.pick_cycle_source_sign
        target_x = -source_x
        source_yaw = 0.0 if source_x >= 0.0 else math.pi
        target_yaw = math.pi if source_x >= 0.0 else 0.0

        if not self.pick_cycle_initialized:
            self.set_cube_pose(source_x, self.cube_y, self.cube_z, yaw=source_yaw)
            self.pick_cycle_initialized = True

        if not self.send_gripper_goal(self.gripper_open_position):
            return False

        pregrasp = self.make_ee_pose(source_x, self.cube_y, self.pick_pregrasp_z)
        grasp = self.make_ee_pose(source_x, self.cube_y, self.pick_grasp_z)
        drop_pregrasp = self.make_ee_pose(target_x, self.cube_y, self.pick_pregrasp_z)
        drop_pose = self.make_ee_pose(target_x, self.cube_y, self.pick_grasp_z)

        used_ik = self.move_to_ee_pose(pregrasp, 2.0) and self.move_to_ee_pose(grasp, 1.5)
        if not used_ik:
            self.execute_fallback_pick_sequence(open_before_close=False)

        if not self.send_gripper_goal(self.gripper_closed_position):
            return False
        self.set_cube_pose_from_ee()

        self.move_to_ee_pose(pregrasp, 1.2)
        self.set_cube_pose_from_ee()

        current = self.get_current_joint_pose()
        if current is None:
            current = {joint: 0.0 for joint in self.arm_joints}
        rotated = dict(current)
        rotated['joint1'] = clamp(
            rotated.get('joint1', 0.0) + (math.pi * self.pick_cycle_source_sign),
            -2.6,
            2.6,
        )
        self.execute_joint_poses([rotated], 2.0)
        self.set_cube_pose_from_ee()

        self.move_to_ee_pose(drop_pregrasp, 1.5)
        self.set_cube_pose_from_ee()
        if not self.move_to_ee_pose(drop_pose, 1.2):
            self.execute_fallback_pick_sequence(open_before_close=True)
        self.set_cube_pose_from_ee()
        if not self.send_gripper_goal(self.gripper_open_position):
            return False
        self.set_cube_pose(target_x, self.cube_y, self.cube_z, yaw=target_yaw)
        self.move_to_ee_pose(drop_pregrasp, 1.0)

        home = {joint: 0.0 for joint in self.arm_joints}
        self.execute_joint_poses([home], 2.0)
        self.pick_cycle_source_sign *= -1.0
        time.sleep(0.5)
        return True

    def execute_fallback_pick_sequence(self, open_before_close):
        sequence = [
            [0.0, -0.75, 0.95, 0.30],
            [0.0, -0.45, 1.10, -0.35],
            [2.60, -0.45, 1.10, -0.35],
            [2.60, -0.75, 0.95, 0.30],
        ]
        if open_before_close:
            sequence = sequence[2:]
        return self.send_joint_trajectory(sequence, 1.5)

    def make_ee_pose(self, x, y, z):
        return {
            'frame_id': self.base_frame,
            'position': {'x': x, 'y': y, 'z': z},
            'orientation': quaternion_from_euler(0.0, 0.0, 0.0),
        }

    def set_cube_pose_from_ee(self):
        ee_pose = self.lookup_ee_pose()
        if ee_pose is None:
            return False
        position = ee_pose['position']
        return self.set_cube_pose(position['x'], position['y'], position['z'] - 0.03)

    def set_cube_pose(self, x, y, z, yaw=0.0):
        if self.set_entity_pose_client is None or Entity is None:
            return False
        if not self.set_entity_pose_client.wait_for_service(timeout_sec=0.2):
            return False

        req = SetEntityPose.Request()
        req.entity.name = self.cube_name
        req.entity.type = Entity.MODEL
        req.pose = Pose()
        req.pose.position.x = float(x)
        req.pose.position.y = float(y)
        req.pose.position.z = float(z)
        q = yaw_quaternion(yaw)
        req.pose.orientation.x = q['x']
        req.pose.orientation.y = q['y']
        req.pose.orientation.z = q['z']
        req.pose.orientation.w = q['w']
        future = self.set_entity_pose_client.call_async(req)
        self.wait_future(future, 1.0)
        return future.done() and future.result() is not None and future.result().success

    def duration_msg(self, seconds):
        msg = DurationMsg()
        msg.sec = int(seconds)
        msg.nanosec = int((seconds - int(seconds)) * 1e9)
        return msg

    def print_current_state(self):
        joint_map = self.current_joint_map()
        if not joint_map:
            self.get_logger().warning('No /joint_states received yet.')
            return
        values = []
        for joint in self.arm_joints + self.gripper_joints:
            if joint in joint_map:
                values.append(f'{joint}={joint_map[joint]:.3f}')
        self.get_logger().info('Current state: ' + ', '.join(values))
        self.log_recording_counts()

    def log_recording_counts(self):
        status = 'active' if self.recording_active else 'idle'
        self.get_logger().info(
            f'Motion recording is {status}; '
            f'{len(self.recorded_motion)} samples loaded; '
            f'file: {self.recording_file}'
        )

    def clear_current_buffers(self):
        self.current_joint_motion.clear()
        self.current_ee_sequence.clear()
        self.get_logger().info('Cleared current joint motion and EE target buffers.')

    def load_recording(self):
        path = Path(self.recording_file)
        if not path.exists():
            return
        try:
            data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
        except Exception as exc:
            self.get_logger().warning(f'Could not read recording file: {exc}')
            return
        waypoints = list(data.get('waypoints', []))
        if len(waypoints) < 2:
            self.get_logger().warning(f'Recording file has too few waypoints: {path}')
            return
        self.recorded_motion = waypoints
        self.get_logger().info(f'Loaded {len(waypoints)} recorded motion samples from {path}.')

    def save_recording(self):
        path = Path(self.recording_file)
        path.parent.mkdir(parents=True, exist_ok=True)
        data = {
            'version': 1,
            'kind': 'motion',
            'arm_joint_names': list(self.arm_joints),
            'gripper_joint_names': list(self.gripper_joints),
            'sample_rate_hz': self.motion_sample_rate_hz,
            'waypoints': self.recorded_motion,
        }
        path.write_text(yaml.safe_dump(data, sort_keys=False), encoding='utf-8')

    def save_recordings(self):
        self.save_recording()

    def destroy_node(self):
        self.random_dance_stop.set()
        self.pick_task_stop.set()
        self.playback_stop.set()
        self.stop_motion_recording(save=False)
        try:
            self.stop_servo()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FinalProjJoyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
