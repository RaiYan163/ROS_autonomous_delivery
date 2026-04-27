#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoystickTeleop(Node):
    def __init__(self) -> None:
        super().__init__("joystick_teleop")

        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("axis_turn", 0)
        self.declare_parameter("axis_forward", 1)
        self.declare_parameter("stop_button_index", 0)
        self.declare_parameter("linear_step", 0.10)
        self.declare_parameter("angular_step", 0.30)
        self.declare_parameter("max_linear_speed", 0.50)
        self.declare_parameter("max_angular_speed", 1.50)
        self.declare_parameter("axis_trigger_threshold", 0.70)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("deadzone", 0.05)

        self._joy_topic = str(self.get_parameter("joy_topic").value)
        self._cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._axis_turn = int(self.get_parameter("axis_turn").value)
        self._axis_forward = int(self.get_parameter("axis_forward").value)
        self._stop_button_index = int(self.get_parameter("stop_button_index").value)
        self._linear_step = float(self.get_parameter("linear_step").value)
        self._angular_step = float(self.get_parameter("angular_step").value)
        self._max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self._max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self._axis_trigger_threshold = float(
            self.get_parameter("axis_trigger_threshold").value
        )
        self._publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._deadzone = float(self.get_parameter("deadzone").value)

        self._target_linear = 0.0
        self._target_angular = 0.0
        self._prev_forward_trigger = 0
        self._prev_turn_trigger = 0
        self._last_stop_pressed = None

        self._cmd_pub = self.create_publisher(TwistStamped, self._cmd_vel_topic, 10)
        self._joy_sub = self.create_subscription(Joy, self._joy_topic, self._on_joy, 10)
        self._publish_timer = self.create_timer(
            1.0 / max(1.0, self._publish_rate_hz), self._publish_current_cmd
        )

        self.get_logger().info(
            "joystick_teleop incremental mode: axis_turn=%d axis_forward=%d stop_button=%d "
            "linear_step=%.2f angular_step=%.2f (edge-trigger)"
            % (
                self._axis_turn,
                self._axis_forward,
                self._stop_button_index,
                self._linear_step,
                self._angular_step,
            )
        )

    def _apply_deadzone(self, value: float) -> float:
        if abs(value) < self._deadzone:
            return 0.0
        return value

    def _on_joy(self, msg: Joy) -> None:
        stop_pressed = (
            self._stop_button_index < len(msg.buttons)
            and msg.buttons[self._stop_button_index] == 1
        )

        if self._last_stop_pressed != stop_pressed:
            if stop_pressed:
                self.get_logger().warn("STOP button pressed -> publishing zero velocity.")
                self._target_linear = 0.0
                self._target_angular = 0.0
                self._prev_forward_trigger = 0
                self._prev_turn_trigger = 0
            else:
                self.get_logger().info("STOP button released -> joystick teleop enabled.")
            self._last_stop_pressed = stop_pressed

        if stop_pressed:
            return

        raw_forward = (
            msg.axes[self._axis_forward] if self._axis_forward < len(msg.axes) else 0.0
        )
        raw_turn = msg.axes[self._axis_turn] if self._axis_turn < len(msg.axes) else 0.0
        forward = self._apply_deadzone(raw_forward)
        turn = self._apply_deadzone(raw_turn)

        forward_trigger = 0
        if forward >= self._axis_trigger_threshold:
            forward_trigger = 1
        elif forward <= -self._axis_trigger_threshold:
            forward_trigger = -1

        turn_trigger = 0
        if turn >= self._axis_trigger_threshold:
            turn_trigger = 1
        elif turn <= -self._axis_trigger_threshold:
            turn_trigger = -1

        # Edge-triggered stepping: one increment per distinct stick press.
        if forward_trigger != 0 and self._prev_forward_trigger == 0:
            self._target_linear = self._clamp(
                self._target_linear + (forward_trigger * self._linear_step),
                -self._max_linear_speed,
                self._max_linear_speed,
            )
            self.get_logger().info(
                "Linear step: trigger=%+d -> linear.x=%.2f"
                % (forward_trigger, self._target_linear)
            )

        if turn_trigger != 0 and self._prev_turn_trigger == 0:
            self._target_angular = self._clamp(
                self._target_angular + (turn_trigger * self._angular_step),
                -self._max_angular_speed,
                self._max_angular_speed,
            )
            self.get_logger().info(
                "Angular step: trigger=%+d -> angular.z=%.2f"
                % (turn_trigger, self._target_angular)
            )

        self._prev_forward_trigger = forward_trigger
        self._prev_turn_trigger = turn_trigger

    def _publish_current_cmd(self) -> None:
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        cmd.twist.linear.x = self._target_linear
        cmd.twist.angular.z = self._target_angular
        self._cmd_pub.publish(cmd)

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JoystickTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
