import array
import fcntl
import os
import struct
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80

JSIOCGAXES = 0x80016A11
JSIOCGBUTTONS = 0x80016A12
JSIOCGNAME_128 = 0x80806A13


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class FinalProjRawJoyNode(Node):
    def __init__(self):
        super().__init__('joy_node')

        self.device_id = int(self.declare_parameter('device_id', 0).value)
        self.device_name = str(self.declare_parameter('device_name', '').value).strip()
        self.device_path = str(self.declare_parameter('device_path', '').value).strip()
        self.deadzone = float(self.declare_parameter('deadzone', 0.05).value)
        self.autorepeat_rate = float(self.declare_parameter('autorepeat_rate', 20.0).value)
        self.frame_id = str(self.declare_parameter('frame_id', 'joy').value)
        self.num_axes = int(self.declare_parameter('num_axes', 8).value)
        self.num_buttons = int(self.declare_parameter('num_buttons', 16).value)

        self.joy_pub = self.create_publisher(Joy, '/joy', 10)
        self.fd: Optional[int] = None
        self.opened_path = ''
        self.axes = [0.0] * self.num_axes
        self.buttons = [0] * self.num_buttons
        self.waiting_logged = False

        self.poll_timer = self.create_timer(0.01, self.poll_device)
        self.repeat_timer = self.create_timer(
            1.0 / max(self.autorepeat_rate, 1.0),
            self.publish_current,
        )

    def resolved_device_path(self):
        if self.device_path:
            return self.device_path
        return f'/dev/input/js{self.device_id}'

    def ensure_open(self):
        if self.fd is not None:
            return True

        device_path = self.resolved_device_path()
        try:
            self.fd = os.open(device_path, os.O_RDONLY | os.O_NONBLOCK)
        except OSError as exc:
            if not self.waiting_logged:
                self.get_logger().warning(f'Waiting for joystick device {device_path}: {exc}')
                self.waiting_logged = True
            return False

        self.opened_path = device_path
        self.waiting_logged = False
        self.refresh_device_info()
        self.get_logger().info(f'Reading joystick events from {self.opened_path}.')
        return True

    def refresh_device_info(self):
        if self.fd is None:
            return

        axes_buf = array.array('B', [0])
        buttons_buf = array.array('B', [0])
        name_buf = array.array('B', [0] * 128)

        try:
            fcntl.ioctl(self.fd, JSIOCGAXES, axes_buf, True)
            if axes_buf[0] > 0:
                self.num_axes = int(axes_buf[0])
        except OSError:
            pass

        try:
            fcntl.ioctl(self.fd, JSIOCGBUTTONS, buttons_buf, True)
            if buttons_buf[0] > 0:
                self.num_buttons = int(buttons_buf[0])
        except OSError:
            pass

        try:
            fcntl.ioctl(self.fd, JSIOCGNAME_128, name_buf, True)
            raw_name = bytes(name_buf).split(b'\x00', 1)[0]
            if raw_name:
                self.get_logger().info(f'Joystick name: {raw_name.decode(errors="replace")}')
        except OSError:
            pass

        self.axes = [0.0] * self.num_axes
        self.buttons = [0] * self.num_buttons

    def close_device(self):
        if self.fd is None:
            return
        os.close(self.fd)
        self.fd = None
        self.opened_path = ''

    def poll_device(self):
        if not self.ensure_open():
            return

        while rclpy.ok() and self.fd is not None:
            try:
                event = os.read(self.fd, 8)
            except BlockingIOError:
                break
            except OSError as exc:
                self.get_logger().warning(f'Joystick device disconnected: {exc}')
                self.close_device()
                return

            if not event:
                break

            _, value, event_type, number = struct.unpack('<IhBB', event)
            event_type &= ~JS_EVENT_INIT

            if event_type == JS_EVENT_AXIS:
                self.ensure_axis_capacity(number + 1)
                normalized = clamp(float(value) / 32767.0, -1.0, 1.0)
                if abs(normalized) < self.deadzone:
                    normalized = 0.0
                self.axes[number] = normalized
                self.publish_current()
            elif event_type == JS_EVENT_BUTTON:
                self.ensure_button_capacity(number + 1)
                self.buttons[number] = 1 if value else 0
                self.publish_current()

    def ensure_axis_capacity(self, size):
        if size <= len(self.axes):
            return
        self.axes.extend([0.0] * (size - len(self.axes)))

    def ensure_button_capacity(self, size):
        if size <= len(self.buttons):
            return
        self.buttons.extend([0] * (size - len(self.buttons)))

    def publish_current(self):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.axes = list(self.axes)
        msg.buttons = list(self.buttons)
        self.joy_pub.publish(msg)

    def destroy_node(self):
        self.close_device()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FinalProjRawJoyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
