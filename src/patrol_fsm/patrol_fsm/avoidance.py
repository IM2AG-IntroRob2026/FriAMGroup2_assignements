"""
avoidance.py — Avoidance maneuver helper for the Security Patrol Robot.

Sequence (odometry-controlled, non-blocking):
  1. Stop    — publish zero Twist
  2. Back up — 0.2 m at 0.15 m/s, stop when odom distance >= 0.2 m
  3. Rotate  — 90° away from bump side using proportional yaw control
               LEFT  → rotate right (negative angular.z)
               RIGHT → rotate left  (positive angular.z)
               CENTER → rotate right (negative angular.z, default)
  4. Forward — 0.3 m at 0.2 m/s, stop when odom distance >= 0.3 m
  5. Stop    — publish zero Twist, call done_callback

Each motion phase uses a 20 Hz polling timer that checks odometry and cancels
itself when the target is reached. _once() is strictly single-fire.
No blocking calls.
"""

import math

BUMP_LEFT   = 'LEFT'
BUMP_RIGHT  = 'RIGHT'
BUMP_CENTER = 'CENTER'

_BACKUP_DISTANCE  = 0.20          # m
_BACKUP_SPEED     = 0.15          # m/s

_ROTATE_ANGLE     = math.pi / 2   # 90°
_ROTATE_SPEED     = 0.5           # rad/s (max)
_ROTATE_THRESH    = 0.02          # rad  (~1°)

_FORWARD_DISTANCE = 0.30          # m
_FORWARD_SPEED    = 0.20          # m/s

_POLL_PERIOD      = 0.05          # s — 20 Hz

# Exposed for unit tests
_BACKUP_DURATION  = _BACKUP_DISTANCE  / _BACKUP_SPEED
_ROTATE_DURATION  = _ROTATE_ANGLE     / _ROTATE_SPEED
_FORWARD_DURATION = _FORWARD_DISTANCE / _FORWARD_SPEED 


# ── geometry helpers ───────────────────────────────────────────────────────────

def rotation_direction(bump_side: str) -> float:
    """LEFT/CENTER → rotate right → -1.0 | RIGHT → rotate left → +1.0"""
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


def _xy_dist(p1, p2) -> float:
    return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)


# ── maneuver ───────────────────────────────────────────────────────────────────

class AvoidanceManeuver:
    """
    Executes the 4-phase avoidance maneuver.

    node.current_pose must be a geometry_msgs/Pose updated by an odom subscriber.
    done_callback is called with no arguments when the maneuver finishes.
    """

    def __init__(self, node, bump_side: str, done_callback):
        self._node          = node
        self._bump_side     = bump_side
        self._done_callback = done_callback

    # ── public ────────────────────────────────────────────────────────────────

    def start(self):
        self._node.get_logger().info(
            f'[AVOIDANCE] Starting — bump side: {self._bump_side}')
        self._stop_then(self._start_backup)

    # ── helpers ───────────────────────────────────────────────────────────────

    def _stop_then(self, next_fn):
        """Publish zero Twist, then call next_fn once after 0.1 s."""
        from geometry_msgs.msg import Twist
        self._node.cmd_pub.publish(Twist())
        self._node.get_logger().info('[AVOIDANCE] STOP')
        self._once(0.1, next_fn)

    def _once(self, delay: float, fn):
        """Create a strictly single-fire timer."""
        fired = [False]

        def _cb():
            if fired[0]:
                return
            fired[0] = True
            t.cancel()
            fn()

        t = self._node.create_timer(delay, _cb)

    def _poll(self, tick_fn):
        """Create a 20 Hz timer; tick_fn must return True to stop polling."""
        done = [False]

        def _cb():
            if done[0]:
                t.cancel()
                return
            if tick_fn() is True:
                done[0] = True
                t.cancel()

        t = self._node.create_timer(_POLL_PERIOD, _cb)

    # ── phase 2: back up ──────────────────────────────────────────────────────

    def _start_backup(self):
        pose = self._node.current_pose
        if pose is None:
            self._node.get_logger().warn('[AVOIDANCE] No odom — skipping backup')
            self._start_rotate()
            return
        self._backup_origin = pose.position
        self._node.get_logger().info(
            f'[AVOIDANCE] BACKUP {_BACKUP_DISTANCE} m @ {_BACKUP_SPEED} m/s')
        self._poll(self._tick_backup)

    def _tick_backup(self):
        from geometry_msgs.msg import Twist
        pose = self._node.current_pose
        if pose is None:
            return
        if _xy_dist(pose.position, self._backup_origin) >= _BACKUP_DISTANCE:
            self._node.get_logger().info('[AVOIDANCE] Backup done')
            self._stop_then(self._start_rotate)
            return True
        twist = Twist()
        twist.linear.x = -_BACKUP_SPEED
        self._node.cmd_pub.publish(twist)

    # ── phase 3: rotate ───────────────────────────────────────────────────────

    def _start_rotate(self):
        pose = self._node.current_pose
        if pose is None:
            self._node.get_logger().warn('[AVOIDANCE] No odom — skipping rotate')
            self._start_forward()
            return
        direction = rotation_direction(self._bump_side)
        self._yaw_target = _normalize_angle(_get_yaw(pose) + direction * _ROTATE_ANGLE)
        side = 'right' if direction < 0 else 'left'
        self._node.get_logger().info(
            f'[AVOIDANCE] ROTATE {side} 90° (target {math.degrees(self._yaw_target):.1f}°)')
        self._poll(self._tick_rotate)

    def _tick_rotate(self):
        from geometry_msgs.msg import Twist
        pose = self._node.current_pose
        if pose is None:
            return
        error = _normalize_angle(self._yaw_target - _get_yaw(pose))
        if abs(error) < _ROTATE_THRESH:
            self._node.get_logger().info('[AVOIDANCE] Rotate done')
            self._stop_then(self._start_forward)
            return True
        speed = max(0.2, min(_ROTATE_SPEED, abs(error) * 3.0))
        twist = Twist()
        twist.angular.z = math.copysign(speed, error)
        self._node.cmd_pub.publish(twist)

    # ── phase 4: forward ──────────────────────────────────────────────────────

    def _start_forward(self):
        pose = self._node.current_pose
        if pose is None:
            self._node.get_logger().warn('[AVOIDANCE] No odom — skipping forward')
            self._finish()
            return
        self._forward_origin = pose.position
        self._node.get_logger().info(
            f'[AVOIDANCE] FORWARD {_FORWARD_DISTANCE} m @ {_FORWARD_SPEED} m/s')
        self._poll(self._tick_forward)

    def _tick_forward(self):
        from geometry_msgs.msg import Twist
        pose = self._node.current_pose
        if pose is None:
            return
        if _xy_dist(pose.position, self._forward_origin) >= _FORWARD_DISTANCE:
            self._node.get_logger().info('[AVOIDANCE] Forward done')
            self._stop_then(self._finish)
            return True
        twist = Twist()
        twist.linear.x = _FORWARD_SPEED
        self._node.cmd_pub.publish(twist)

    # ── phase 5: finish ───────────────────────────────────────────────────────

    def _finish(self):
        self._node.get_logger().info('[AVOIDANCE] Maneuver complete')
        self._done_callback()
