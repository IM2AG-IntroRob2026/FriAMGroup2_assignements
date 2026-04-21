"""
avoidance.py — Avoidance maneuver helper for the Security Patrol Robot.

Sequence (odometry-controlled, non-blocking):
  1. Stop      — publish zero Twist, wait 0.5 s for bump reflex to clear
  2. Back up   — 0.2 m at 0.15 m/s (odom-based, 3 s timeout)
  3. Rotate    — 90° away from bump side (odom-based, 8 s timeout)
  4. Forward   — 0.3 m at 0.2 m/s  (odom-based, 3 s timeout)
  5. Stop      — publish zero Twist, call done_callback

Timeouts ensure the FSM never gets stuck if odom freezes or
the robot is physically blocked.
"""

import math
import time as _time

BUMP_LEFT   = 'LEFT'
BUMP_RIGHT  = 'RIGHT'
BUMP_CENTER = 'CENTER'

_BACKUP_DISTANCE  = 0.20          # m
_BACKUP_SPEED     = 0.15          # m/s

_ROTATE_ANGLE     = math.pi / 2   # 90°
_ROTATE_SPEED     = 0.5           # rad/s (max)
_ROTATE_MIN_SPEED = 0.3           # rad/s (min, avoid stalling near target)
_ROTATE_THRESH    = 0.05          # rad  (~3°)

_FORWARD_DISTANCE = 0.30          # m
_FORWARD_SPEED    = 0.20          # m/s

_POLL_PERIOD      = 0.05          # s — 20 Hz

_BACKUP_TIMEOUT   = 3.0           # s — give up backup after this
_ROTATE_TIMEOUT   = 8.0           # s — give up rotation after this
_FORWARD_TIMEOUT  = 3.0           # s — give up forward after this

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


def _xy_dist(x1, y1, x2, y2) -> float:
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


# ── maneuver ───────────────────────────────────────────────────────────────────

class AvoidanceManeuver:
    """
    Executes the 4-phase avoidance maneuver (non-blocking, timer-driven).

    node.current_pose must be a geometry_msgs/Pose updated by an odom subscriber.
    done_callback() is called with no arguments when the maneuver finishes.
    callback_group should be the node's ReentrantCallbackGroup.
    """

    def __init__(self, node, bump_side: str, done_callback, callback_group=None):
        self._node          = node
        self._bump_side     = bump_side
        self._done_callback = done_callback
        self._cb_group      = callback_group

    # ── public ────────────────────────────────────────────────────────────────

    def start(self):
        self._node.get_logger().info(
            f'[AVOIDANCE] Starting — bump side: {self._bump_side}')
        # Longer stop (0.5 s) lets the Create3 bump reflex expire before backup
        self._stop_then(self._start_backup, delay=0.5)

    # ── helpers ───────────────────────────────────────────────────────────────

    def _stop_then(self, next_fn, delay=0.1):
        from geometry_msgs.msg import Twist
        self._node.cmd_pub.publish(Twist())
        self._node.get_logger().info('[AVOIDANCE] STOP')
        self._once(delay, next_fn)

    def _once(self, delay: float, fn):
        """Strictly single-fire timer."""
        fired = [False]

        def _cb():
            if fired[0]:
                return
            fired[0] = True
            t.cancel()
            fn()

        t = self._node.create_timer(delay, _cb, callback_group=self._cb_group)

    def _poll(self, tick_fn, timeout_secs=None, timeout_fn=None):
        """
        20 Hz timer calling tick_fn every tick.
        tick_fn() must return True to stop polling normally.
        If timeout_secs elapses first, timeout_fn() is called instead.
        """
        done = [False]
        t_start = _time.monotonic()

        def _cb():
            if done[0]:
                t.cancel()
                return
            # Timeout guard — prevents infinite loop when odom is frozen
            if timeout_secs is not None:
                elapsed = _time.monotonic() - t_start
                if elapsed >= timeout_secs:
                    self._node.get_logger().warn(
                        f'[AVOIDANCE] Phase timeout after {elapsed:.1f} s '
                        f'— forcing next phase')
                    done[0] = True
                    t.cancel()
                    if timeout_fn is not None:
                        timeout_fn()
                    return
            if tick_fn() is True:
                done[0] = True
                t.cancel()

        t = self._node.create_timer(
            _POLL_PERIOD, _cb, callback_group=self._cb_group)

    # ── phase 2: back up ──────────────────────────────────────────────────────

    def _start_backup(self):
        pose = self._node.current_pose
        if pose is None:
            self._node.get_logger().warn('[AVOIDANCE] No odom — skipping backup')
            self._start_rotate()
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
            timeout_fn=lambda: self._stop_then(self._start_rotate),
        )

    def _tick_backup(self):
        from geometry_msgs.msg import Twist
        pose = self._node.current_pose
        if pose is None:
            return
        p = pose.position
        dist = _xy_dist(p.x, p.y, self._backup_ox, self._backup_oy)
        if dist >= _BACKUP_DISTANCE:
            self._node.get_logger().info(
                f'[AVOIDANCE] Backup done (dist={dist:.3f} m '
                f'pos x={p.x:.3f} y={p.y:.3f})')
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
        current_yaw = _get_yaw(pose)
        self._yaw_target = _normalize_angle(current_yaw + direction * _ROTATE_ANGLE)
        side = 'right' if direction < 0 else 'left'
        self._node.get_logger().info(
            f'[AVOIDANCE] ROTATE {side} 90° '
            f'(current={math.degrees(current_yaw):.1f}° '
            f'target={math.degrees(self._yaw_target):.1f}°)')
        self._poll(
            self._tick_rotate,
            timeout_secs=_ROTATE_TIMEOUT,
            timeout_fn=lambda: self._stop_then(self._start_forward),
        )

    def _tick_rotate(self):
        from geometry_msgs.msg import Twist
        pose = self._node.current_pose
        if pose is None:
            return
        current_yaw = _get_yaw(pose)
        error = _normalize_angle(self._yaw_target - current_yaw)
        if abs(error) < _ROTATE_THRESH:
            self._node.get_logger().info(
                f'[AVOIDANCE] Rotate done (yaw={math.degrees(current_yaw):.1f}°)')
            self._stop_then(self._start_forward)
            return True
        speed = max(_ROTATE_MIN_SPEED, min(_ROTATE_SPEED, abs(error) * 3.0))
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
        p = pose.position
        self._forward_ox = p.x
        self._forward_oy = p.y
        self._node.get_logger().info(
            f'[AVOIDANCE] FORWARD {_FORWARD_DISTANCE} m @ {_FORWARD_SPEED} m/s '
            f'(origin x={p.x:.3f} y={p.y:.3f})')
        self._poll(
            self._tick_forward,
            timeout_secs=_FORWARD_TIMEOUT,
            timeout_fn=lambda: self._stop_then(self._finish),
        )

    def _tick_forward(self):
        from geometry_msgs.msg import Twist
        pose = self._node.current_pose
        if pose is None:
            return
        p = pose.position
        dist = _xy_dist(p.x, p.y, self._forward_ox, self._forward_oy)
        if dist >= _FORWARD_DISTANCE:
            self._node.get_logger().info(
                f'[AVOIDANCE] Forward done (dist={dist:.3f} m '
                f'pos x={p.x:.3f} y={p.y:.3f})')
            self._stop_then(self._finish)
            return True
        twist = Twist()
        twist.linear.x = _FORWARD_SPEED
        self._node.cmd_pub.publish(twist)

    # ── phase 5: finish ───────────────────────────────────────────────────────

    def _finish(self):
        self._node.get_logger().info('[AVOIDANCE] Maneuver complete — resuming patrol')
        self._done_callback()
