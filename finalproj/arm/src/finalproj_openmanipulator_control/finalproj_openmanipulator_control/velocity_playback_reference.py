"""Reference implementation for recorded-motion playback with JointJog velocities.

This module is not used by the final-project controller. It preserves the
velocity-streaming experiment so it can be revisited without keeping that
behavior active in runtime playback.
"""


def build_recorded_velocity_commands(controller, waypoints):
    commands = []
    previous_velocities = [0.0] * len(controller.arm_joints)
    smoothing = max(0.0, min(1.0, controller.playback_velocity_smoothing))
    for current, next_waypoint in zip(waypoints, waypoints[1:]):
        current_positions = current.get('positions_arm', [])
        next_positions = next_waypoint.get('positions_arm', [])
        if len(current_positions) != len(controller.arm_joints) or len(next_positions) != len(
            controller.arm_joints
        ):
            continue

        current_time = float(current.get('time_from_start_sec', 0.0))
        next_time = float(next_waypoint.get('time_from_start_sec', current_time))
        duration = next_time - current_time
        if duration <= 1e-3:
            continue

        velocities = []
        for joint_index, (start_position, end_position) in enumerate(
            zip(current_positions, next_positions)
        ):
            velocity = (
                (float(end_position) - float(start_position))
                / duration
                * controller.playback_velocity_scale
            )
            smoothed_velocity = (
                smoothing * previous_velocities[joint_index]
                + (1.0 - smoothing) * velocity
            )
            velocities.append(
                max(
                    -controller.playback_max_joint_velocity,
                    min(
                        controller.playback_max_joint_velocity,
                        smoothed_velocity,
                    ),
                )
            )
        previous_velocities = velocities
        commands.append(
            {
                'duration': duration,
                'velocities': velocities,
                'gripper': next_waypoint.get('gripper'),
            }
        )
    return commands
