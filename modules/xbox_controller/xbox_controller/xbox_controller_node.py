#!/usr/bin/env python3
import math
import threading
from dataclasses import dataclass, field
from typing import Dict

import rclpy
from rclpy.node import Node

from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import GearCommand


@dataclass
class ControllerState:
    axes: Dict[str, float] = field(default_factory=dict)
    buttons: Dict[str, int] = field(default_factory=dict)
    connected: bool = False


class XboxInputReader:
    MAX_TRIGGER_VALUE = 255.0
    MAX_JOYSTICK_VALUE = 32768.0

    def __init__(self, device_index: int, logger):
        self._logger = logger
        self._lock = threading.Lock()
        self._state = ControllerState()
        self._running = True

        try:
            import inputs
            from inputs import get_gamepad
        except ImportError as exc:
            raise RuntimeError(
                "Python package 'inputs' is not installed. Install it in the runtime "
                "environment, for example: python3 -m pip install inputs"
            ) from exc

        if device_index >= len(inputs.devices.gamepads):
            raise RuntimeError(
                f"Xbox controller index {device_index} not found; "
                f"detected {len(inputs.devices.gamepads)} gamepad(s)."
            )

        self._get_gamepad = get_gamepad
        self._gamepad = inputs.devices.gamepads[device_index]
        self._thread = threading.Thread(target=self._monitor, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False

    def vibrate(self, duration_ms: int):
        try:
            self._gamepad.set_vibration(1, 1, duration_ms)
        except Exception:
            self._logger.debug('Controller vibration is not available')

    def snapshot(self) -> ControllerState:
        with self._lock:
            return ControllerState(
                axes=dict(self._state.axes),
                buttons=dict(self._state.buttons),
                connected=self._state.connected,
            )

    def _monitor(self):
        while self._running:
            try:
                events = self._get_gamepad()
            except Exception as exc:
                with self._lock:
                    self._state.connected = False
                self._logger.warn(f'Xbox controller read failed: {exc}')
                continue

            with self._lock:
                self._state.connected = True
                for event in events:
                    if event.code in ('ABS_X', 'ABS_Y', 'ABS_RX', 'ABS_RY'):
                        self._state.axes[event.code] = self._normalize_joystick(event.state)
                    elif event.code in ('ABS_Z', 'ABS_RZ'):
                        self._state.axes[event.code] = self._normalize_trigger(event.state)
                    elif event.code.startswith('BTN_'):
                        self._state.buttons[event.code] = int(event.state)

    @classmethod
    def _normalize_joystick(cls, value: int) -> float:
        return max(-1.0, min(1.0, float(value) / cls.MAX_JOYSTICK_VALUE))

    @classmethod
    def _normalize_trigger(cls, value: int) -> float:
        return max(0.0, min(1.0, float(value) / cls.MAX_TRIGGER_VALUE))


class XboxControllerNode(Node):
    def __init__(self):
        super().__init__('xbox_controller')

        self.control_topic = self.declare_parameter(
            'control_topic', '/control/command/control_cmd').value
        self.gear_topic = self.declare_parameter(
            'gear_topic', '/control/command/gear_cmd').value
        self.publish_rate_hz = float(self.declare_parameter('publish_rate_hz', 25.0).value)
        self.device_index = int(self.declare_parameter('device_index', 0).value)

        self.require_deadman = bool(self.declare_parameter('require_deadman', True).value)
        self.deadman_button = self.declare_parameter('deadman_button', 'BTN_TR').value
        self.arm_button = self.declare_parameter('arm_button', 'BTN_SOUTH').value
        self.disarm_button = self.declare_parameter('disarm_button', 'BTN_SELECT').value
        self.arm_hold_sec = float(self.declare_parameter('arm_hold_sec', 1.5).value)
        self.neutral_on_disarm = bool(self.declare_parameter('neutral_on_disarm', True).value)
        self.neutral_on_deadman_release = bool(
            self.declare_parameter('neutral_on_deadman_release', True).value)
        self.allow_reverse = bool(self.declare_parameter('allow_reverse', False).value)

        self.steering_axis = self.declare_parameter('steering_axis', 'ABS_X').value
        self.throttle_axis = self.declare_parameter('throttle_axis', 'ABS_RZ').value
        self.brake_axis = self.declare_parameter('brake_axis', 'ABS_Z').value
        self.neutral_button = self.declare_parameter('neutral_button', 'BTN_WEST').value
        self.drive_button = self.declare_parameter('drive_button', 'BTN_EAST').value
        self.reverse_button = self.declare_parameter('reverse_button', 'BTN_NORTH').value

        self.max_steering_angle_rad = float(
            self.declare_parameter('max_steering_angle_rad', 0.5236).value)
        self.max_accel_mps2 = float(self.declare_parameter('max_accel_mps2', 2.0).value)
        self.max_decel_mps2 = float(self.declare_parameter('max_decel_mps2', 5.0).value)
        self.max_velocity_mps = float(self.declare_parameter('max_velocity_mps', 3.0).value)
        self.steering_deadzone = float(self.declare_parameter('steering_deadzone', 0.10).value)
        self.trigger_deadzone = float(self.declare_parameter('trigger_deadzone', 0.02).value)

        self.control_pub = self.create_publisher(Control, self.control_topic, 10)
        self.gear_pub = self.create_publisher(GearCommand, self.gear_topic, 10)

        self.armed = False
        self.current_gear = GearCommand.NEUTRAL
        self._arm_started_at = None
        self._reverse_warned = False

        self.reader = None
        self._last_reader_error = ''
        self._last_reader_retry = self.get_clock().now()
        self._reader_retry_period_sec = 2.0
        try:
            self._create_reader()
        except RuntimeError as exc:
            self._last_reader_error = str(exc)
            self.get_logger().error(self._last_reader_error)

        timer_period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.timer = self.create_timer(timer_period, self._on_timer)

        self.get_logger().info(
            f'xbox_controller publishing {self.control_topic} and {self.gear_topic}; '
            f'require_deadman={self.require_deadman}; raw CAN is not used'
        )

    def destroy_node(self):
        if self.reader is not None:
            self.reader.stop()
        super().destroy_node()

    def _on_timer(self):
        if self.reader is None:
            self._retry_create_reader()
            self._publish_safe_stop()
            return

        state = self.reader.snapshot()
        if not state.connected:
            self._set_armed(False, 'controller disconnected')
            self._publish_safe_stop()
            return

        self._update_arm_state(state)
        self._update_gear_state(state)

        deadman_ok = (not self.require_deadman) or self._button(state, self.deadman_button)
        if not self.armed or not deadman_ok:
            self._publish_safe_stop(
                force_neutral=self.neutral_on_disarm or self.neutral_on_deadman_release)
            return

        steering = self._axis(state, self.steering_axis)
        steering = self._apply_deadzone(steering, self.steering_deadzone)
        throttle = self._apply_deadzone(self._axis(state, self.throttle_axis), self.trigger_deadzone)
        brake = self._apply_deadzone(self._axis(state, self.brake_axis), self.trigger_deadzone)

        accel = throttle * self.max_accel_mps2
        velocity = throttle * self.max_velocity_mps
        if brake > 0.0:
            accel = -brake * self.max_decel_mps2
            velocity = 0.0

        self._publish_control(
            steering_tire_angle=steering * self.max_steering_angle_rad,
            acceleration=accel,
            velocity=velocity,
        )
        self._publish_gear(self.current_gear)

    def _update_arm_state(self, state: ControllerState):
        now = self.get_clock().now()

        if self._button(state, self.disarm_button):
            self._set_armed(False, 'disarm button pressed')
            self._arm_started_at = None
            return

        if self._button(state, self.arm_button):
            if self._arm_started_at is None:
                self._arm_started_at = now
            elif not self.armed and (now - self._arm_started_at).nanoseconds / 1e9 >= self.arm_hold_sec:
                self._set_armed(True, 'arm button hold accepted')
                self.reader.vibrate(500)
        else:
            self._arm_started_at = None

    def _create_reader(self):
        self.reader = XboxInputReader(self.device_index, self.get_logger())
        self.reader.vibrate(250)
        self._last_reader_error = ''
        self.get_logger().info(f'Xbox controller connected at device_index={self.device_index}')

    def _retry_create_reader(self):
        now = self.get_clock().now()
        if (now - self._last_reader_retry).nanoseconds / 1e9 < self._reader_retry_period_sec:
            return
        self._last_reader_retry = now
        try:
            self._create_reader()
        except RuntimeError as exc:
            message = str(exc)
            if message != self._last_reader_error:
                self._last_reader_error = message
                self.get_logger().error(message)

    def _update_gear_state(self, state: ControllerState):
        if self._button(state, self.neutral_button):
            self._set_gear(GearCommand.NEUTRAL)
        elif self._button(state, self.drive_button):
            self._set_gear(GearCommand.DRIVE)
        elif self._button(state, self.reverse_button):
            if self.allow_reverse:
                self._set_gear(GearCommand.REVERSE)
            elif not self._reverse_warned:
                self.get_logger().warn('Reverse button ignored because allow_reverse=false')
                self._reverse_warned = True

    def _publish_safe_stop(self, force_neutral: bool = True):
        self._publish_control(steering_tire_angle=0.0, acceleration=-self.max_decel_mps2, velocity=0.0)
        if force_neutral:
            self.current_gear = GearCommand.NEUTRAL
        self._publish_gear(self.current_gear)

    def _publish_control(self, steering_tire_angle: float, acceleration: float, velocity: float):
        msg = Control()
        stamp = self.get_clock().now().to_msg()
        msg.stamp = stamp
        msg.lateral.stamp = stamp
        msg.lateral.steering_tire_angle = float(steering_tire_angle)
        msg.longitudinal.stamp = stamp
        msg.longitudinal.velocity = float(velocity)
        msg.longitudinal.acceleration = float(acceleration)
        msg.longitudinal.is_defined_acceleration = True
        self.control_pub.publish(msg)

    def _publish_gear(self, command: int):
        msg = GearCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.command = command
        self.gear_pub.publish(msg)

    def _set_armed(self, armed: bool, reason: str):
        if self.armed == armed:
            return
        self.armed = armed
        state = 'armed' if armed else 'disarmed'
        self.get_logger().warn(f'Xbox controller {state}: {reason}')

    def _set_gear(self, gear: int):
        if self.current_gear == gear:
            return
        self.current_gear = gear
        self.get_logger().info(f'Xbox controller gear command: {gear}')

    @staticmethod
    def _axis(state: ControllerState, code: str) -> float:
        return float(state.axes.get(code, 0.0))

    @staticmethod
    def _button(state: ControllerState, code: str) -> bool:
        return bool(state.buttons.get(code, 0))

    @staticmethod
    def _apply_deadzone(value: float, deadzone: float) -> float:
        if math.fabs(value) <= deadzone:
            return 0.0
        sign = 1.0 if value > 0.0 else -1.0
        return sign * ((math.fabs(value) - deadzone) / max(1.0 - deadzone, 1e-6))


def main(args=None):
    rclpy.init(args=args)
    node = XboxControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
