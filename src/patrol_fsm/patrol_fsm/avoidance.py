"""
avoidance.py : Avoidance maneuver helper for the Security Patrol Robot.

Sequence (all time-based, no blocking):
  1. Stop   : publish zero Twist
  2. Back up : 0.2 m at 0.15 m/s  → duration = 0.2 / 0.15 ≈ 1.33 s
  3. Rotate  : 90° at 0.5 rad/s   → duration = (π/2) / 0.5 ≈ 3.14 s
              direction: away from the bump side
              LEFT  → rotate right (negative angular.z)
              RIGHT → rotate left  (positive angular.z)
              CENTER → rotate right (negative angular.z, default)
  4. Forward : 0.3 m at 0.2 m/s  → duration = 0.3 / 0.2 = 1.5 s
  5. Stop   : publish zero Twist, call done_callback

"""

import math


# ---------------------------------------------------------------------------
# Bump side constants  (mirror of irobot_create_msgs HazardDetection.type
# field strings — compared by string to keep this module import-free)
# ---------------------------------------------------------------------------
BUMP_LEFT = 'LEFT'
BUMP_RIGHT = 'RIGHT'
BUMP_CENTER = 'CENTER'

# Maneuver parameters
_BACKUP_DISTANCE = 0.20      # metres
_BACKUP_SPEED = 0.15         # m/s
_BACKUP_DURATION = _BACKUP_DISTANCE / _BACKUP_SPEED 

_ROTATE_ANGLE = math.pi / 2  # 90deg
_ROTATE_SPEED = 0.5           # rad/s
_ROTATE_DURATION = _ROTATE_ANGLE / _ROTATE_SPEED

_FORWARD_DISTANCE = 0.30     # metres
_FORWARD_SPEED = 0.20        # m/s
_FORWARD_DURATION = _FORWARD_DISTANCE / _FORWARD_SPEED


def rotation_direction(bump_side: str) -> float:
    """
    Return the sign of angular.z for the avoidance rotation.

    LEFT  bump → rotate right → negative angular.z  → returns -1.0
    RIGHT bump → rotate left  → positive angular.z  → returns +1.0
    CENTER/unknown → rotate right → returns -1.0
    """
    if bump_side == BUMP_RIGHT:
        return 1.0
    return -1.0   # LEFT or CENTER


class AvoidanceManeuver:
    """
    Executes the 4-phase avoidance maneuver using one-shot timers.

    Usage::

        maneuver = AvoidanceManeuver(node, bump_side, done_callback)
        maneuver.start()

    ``done_callback`` is called with no arguments when the maneuver finishes.
    The caller (patrol_node) is responsible for driving the FSM transition.
    """

    def __init__(self, node, bump_side: str, done_callback):
        self._node = node
        self._bump_side = bump_side
        self._done_callback = done_callback
        self._timers = []   # keep references so they are not GC'd

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self):
        """Begin phase 1: stop."""
        self._node.get_logger().info(
            f'[AVOIDANCE] Starting maneuver — bump side: {self._bump_side}'
        )
        self._phase_stop(next_phase=self._phase_backup)

    # ------------------------------------------------------------------
    # Phases
    # ------------------------------------------------------------------

    def _phase_stop(self, next_phase):
        """Publish zero Twist, then schedule next_phase immediately."""
        from geometry_msgs.msg import Twist
        self._node.cmd_pub.publish(Twist())
        self._node.get_logger().info('[AVOIDANCE] Phase: STOP')
        # Schedule next phase on the next timer tick (0.05 s)
        self._schedule(0.05, next_phase)

    def _phase_backup(self):
        """Publish backward Twist for BACKUP_DURATION, then rotate."""
        from geometry_msgs.msg import Twist
        twist = Twist()
        twist.linear.x = -_BACKUP_SPEED
        self._node.cmd_pub.publish(twist)
        self._node.get_logger().info(
            f'[AVOIDANCE] Phase: BACKUP ({_BACKUP_DISTANCE} m @ {_BACKUP_SPEED} m/s '
            f'= {_BACKUP_DURATION:.2f} s)'
        )
        self._schedule(_BACKUP_DURATION, self._phase_rotate)

    def _phase_rotate(self):
        """Publish rotation Twist for ROTATE_DURATION, then move forward."""
        from geometry_msgs.msg import Twist
        direction = rotation_direction(self._bump_side)
        twist = Twist()
        twist.angular.z = direction * _ROTATE_SPEED
        self._node.cmd_pub.publish(twist)
        side_label = 'right' if direction < 0 else 'left'
        self._node.get_logger().info(
            f'[AVOIDANCE] Phase: ROTATE {side_label} 90° '
            f'({_ROTATE_DURATION:.2f} s, bump={self._bump_side})'
        )
        self._schedule(_ROTATE_DURATION, self._phase_forward)

    def _phase_forward(self):
        """Publish forward Twist for FORWARD_DURATION, then stop and finish."""
        from geometry_msgs.msg import Twist
        twist = Twist()
        twist.linear.x = _FORWARD_SPEED
        self._node.cmd_pub.publish(twist)
        self._node.get_logger().info(
            f'[AVOIDANCE] Phase: FORWARD ({_FORWARD_DISTANCE} m @ {_FORWARD_SPEED} m/s '
            f'= {_FORWARD_DURATION:.2f} s)'
        )
        self._schedule(_FORWARD_DURATION, self._phase_finish)

    def _phase_finish(self):
        """Stop the robot and call the done callback."""
        from geometry_msgs.msg import Twist
        self._node.cmd_pub.publish(Twist())
        self._node.get_logger().info('[AVOIDANCE] Maneuver complete')
        self._done_callback()

    # ------------------------------------------------------------------
    # Timer helper
    # ------------------------------------------------------------------

    def _schedule(self, delay: float, callback):
        """Create a one-shot timer that cancels itself after firing."""
        def _wrapper():
            timer.cancel()
            self._timers.remove(timer)
            callback()

        timer = self._node.create_timer(delay, _wrapper)
        self._timers.append(timer)