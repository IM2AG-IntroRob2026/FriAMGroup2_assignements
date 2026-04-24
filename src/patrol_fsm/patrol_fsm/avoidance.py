"""
avoidance.py — Avoidance maneuver helper for the Security Patrol Robot.

Bump-reaction sequence (odom-controlled, non-blocking):
  1. Stop        — publish zero Twist, wait 0.5 s for bump reflex to clear
  2. Back up     — 0.2 m at 0.15 m/s (odom-based, 3 s timeout)
  3. Rotate away — 90° away from bump side (odom-based, 8 s timeout)
                   LEFT/CENTER → turn right  |  RIGHT → turn left
  4. Stop        — publish zero Twist, call done_callback

After step 4 the robot is clear of the obstacle; patrol_node navigates it
back to the saved lap-start position before resuming the patrol square.
"""

import math
import time as _time

BUMP_LEFT   = 'LEFT'
BUMP_RIGHT  = 'RIGHT'
BUMP_CENTER = 'CENTER'

_BACKUP_DISTANCE  = 0.20
_BACKUP_SPEED     = 0.15

_ROTATE_ANGLE     = math.pi / 2
_ROTATE_SPEED     = 0.5
_ROTATE_MIN_SPEED = 0.3
_ROTATE_THRESH    = 0.05          # rad (~3°)

_POLL_PERIOD      = 0.05          # s — 20 Hz

_BACKUP_TIMEOUT   = 3.0           # s
_ROTATE_TIMEOUT   = 8.0           # s

# Exposed for unit tests
_BACKUP_DURATION  = _BACKUP_DISTANCE / _BACKUP_SPEED
_ROTATE_DURATION  = _ROTATE_ANGLE   / _ROTATE_SPEED


def rotation_direction(bump_side: str) -> float:
    """LEFT/CENTER → rotate right (-1.0)  |  RIGHT → rotate left (+1.0)"""
    return 1.0 if bump_side == BUMP_RIGHT else -1.0


def _get_yaw(pose) -> float:
    q = pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _normalize_angle(a: float) -> float:
    while a >  math.pi: a -= 2.0 * math.pi
    while a < -math.pi: a += 2.0 * math.pi
    return a


def _xy_dist(x1, y1, x2, y2) -> float:
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


class AvoidanceManeuver:
    """
    Executes the bump-reaction maneuver (non-blocking, timer-driven).

    node.current_pose  — geometry_msgs/Pose updated by an odom subscriber.
    done_callback()    — called with no arguments when the maneuver finishes.
    callback_group     — node's ReentrantCallbackGroup (avoids mutex with odom).
    """

    def __init__(self, node, bump_side: str, done_callback,
                 callback_group=None, original_heading=None):
        self._node          = node
        self._bump_side     = bump_side
        self._done_callback = done_callback
        self._cb_group      = callback_group
        self._avoid_dir     = rotation_direction(bump_side)

    def start(self):
        self._node.get_logger().info(
            f'[AVOIDANCE] Starting — bump: {self._bump_side} '
            f'(rotate {"left" if self._avoid_dir > 0 else "right"})')
        # 0.5 s lets the Create3 bump-reflex firmware lock expire before moving
        self._stop_then(self._start_backup, delay=0.5)

    def _stop_then(self, next_fn, delay=0.1):
        from geometry_msgs.msg import Twist
        self._node.cmd_pub.publish(Twist())
        self._node.get_logger().info('[AVOIDANCE] STOP')
        self._once(delay, next_fn)

    def _once(self, delay: float, fn):
        # Guard prevents re-entry; timer is never cancelled from inside its own
        # callback — that triggers RCLError in rclpy Iron's MultiThreadedExecutor.
        fired = [False]

        def _cb():
            if fired[0]:
                return
            fired[0] = True
            fn()

        self._node.create_timer(delay, _cb, callback_group=self._cb_group)

    def _poll(self, tick_fn, timeout_secs=None, timeout_fn=None):
        """20 Hz poll. tick_fn() returns True to finish; timeout_fn() on expiry."""
        done    = [False]
        t_start = _time.monotonic()

        def _cb():
            if done[0]:
                return
            if timeout_secs is not None:
                if _time.monotonic() - t_start >= timeout_secs:
                    self._node.get_logger().warn(
                        f'[AVOIDANCE] Phase timeout after {timeout_secs:.0f} s '
                        f'— forcing next phase')
                    done[0] = True
                    if timeout_fn:
                        timeout_fn()
                    return
            if tick_fn() is True:
                done[0] = True

        self._node.create_timer(_POLL_PERIOD, _cb, callback_group=self._cb_group)

    def _rotate(self, direction: float, label: str, next_fn):
        """90° relative rotation. direction: +1.0=CCW/left, -1.0=CW/right."""
        pose = self._node.current_pose
        if pose is None:
            self._node.get_logger().warn(f'[AVOIDANCE] No odom — skipping {label}')
            next_fn()
            return
        current_yaw      = _get_yaw(pose)
        self._yaw_target = _normalize_angle(current_yaw + direction * _ROTATE_ANGLE)
        side = 'left' if direction > 0 else 'right'
        self._node.get_logger().info(
            f'[AVOIDANCE] ROTATE {label} {side} '
            f'(cur={math.degrees(current_yaw):.1f}° '
            f'tgt={math.degrees(self._yaw_target):.1f}°)')
        self._poll(
            lambda: self._tick_rotate(next_fn),
            timeout_secs=_ROTATE_TIMEOUT,
            timeout_fn=lambda: self._stop_then(next_fn),
        )

    def _tick_rotate(self, next_fn):
        from geometry_msgs.msg import Twist
        pose = self._node.current_pose
        if pose is None:
            return
        current_yaw = _get_yaw(pose)
        error       = _normalize_angle(self._yaw_target - current_yaw)
        if abs(error) < _ROTATE_THRESH:
            self._node.get_logger().info(
                f'[AVOIDANCE] Rotate done (yaw={math.degrees(current_yaw):.1f}°)')
            self._stop_then(next_fn)
            return True
        speed = max(_ROTATE_MIN_SPEED, min(_ROTATE_SPEED, abs(error) * 3.0))
        twist = Twist()
        twist.angular.z = math.copysign(speed, error)
        self._node.cmd_pub.publish(twist)

    def _start_backup(self):
        pose = self._node.current_pose
        if pose is None:
            self._node.get_logger().warn('[AVOIDANCE] No odom — skipping backup')
            self._start_rotate_avoid()
            return
        p = pose.position
        self._backup_ox = p.x
        self._backup_oy = p.y
        self._node.get_logger().info(
            f'[AVOIDANCE] BACKUP {_BACKUP_DISTANCE} m @ {_BACKUP_SPEED} m/s '
            f'(origin x={p.x:.3f} y={p.y:.3f})')
        self._poll(
            self._tick_backup,
            timeout_secs=_BACKUP_TIMEOUT,
            timeout_fn=lambda: self._stop_then(self._start_rotate_avoid),
        )

    def _tick_backup(self):
        from geometry_msgs.msg import Twist
        pose = self._node.current_pose
        if pose is None:
            return
        p    = pose.position
        dist = _xy_dist(p.x, p.y, self._backup_ox, self._backup_oy)
        if dist >= _BACKUP_DISTANCE:
            self._node.get_logger().info(f'[AVOIDANCE] Backup done (dist={dist:.3f} m)')
            self._stop_then(self._start_rotate_avoid)
            return True
        twist = Twist()
        twist.linear.x = -_BACKUP_SPEED
        self._node.cmd_pub.publish(twist)

    def _start_rotate_avoid(self):
        self._rotate(self._avoid_dir, 'AVOID', self._finish)

    def _finish(self):
        self._node.get_logger().info(
            '[AVOIDANCE] Clear of obstacle — handing off to return-to-start')
        self._done_callback()
