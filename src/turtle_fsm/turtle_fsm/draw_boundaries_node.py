#!/usr/bin/env python3
"""FSM-based boundary drawing controller for turtlesim."""
import math
from enum import Enum

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from turtlesim.msg import Pose
from turtlesim.srv import SetPen, TeleportAbsolute

WORLD_MIN = 4
WORLD_MAX = 7
SPEED = 2.0
ANGLE_TOL = 0.05
TURN_KP = 4.0
# How far inside the boundary the turtle aims to stay
WALL_MARGIN = 0.3


class State(Enum):
    """FSM states."""

    MOVING_TO_BOUNDARY = 'moving_to_boundary'
    FOLLOWING_BOUNDARY = 'following_boundary'
    RETURNING_HOME = 'returning_home'
    IDLE = 'idle'
    MANUAL = 'manual'


class Wall(Enum):
    """Wall identifier."""

    NONE = 'none'
    RIGHT = 'right'
    TOP = 'top'
    LEFT = 'left'
    BOTTOM = 'bottom'


# Clockwise: right -> top -> left -> bottom -> right
WALL_HEADING = {
    Wall.RIGHT: math.pi / 2,
    Wall.TOP: math.pi,
    Wall.LEFT: -math.pi / 2,
    Wall.BOTTOM: 0.0,
}


class DrawBoundariesNode(Node):
    """FSM node that draws the turtlesim boundary and supports manual interruption."""

    def __init__(self):
        """Initialize subscribers, publishers, service client and timers."""
        super().__init__('draw_boundaries_node')

        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_subscription(Pose, '/turtle1/pose', self._on_pose, 10)
        self.create_subscription(Twist, '/manual_cmd_vel', self._on_manual_cmd, 10)
        self.create_subscription(Empty, '/toggle_manual', self._on_toggle, 10)
        self._pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self._teleport_client = self.create_client(
            TeleportAbsolute, '/turtle1/teleport_absolute')

        self._pose = None
        self._home = None
        self._state = State.MOVING_TO_BOUNDARY
        self._state_before_manual = State.MOVING_TO_BOUNDARY
        self._wall = Wall.NONE
        self._boundary_start = None
        self._boundary_moved_away = False
        self._manual_twist = Twist()

        for client, name in [
            (self._pen_client, '/turtle1/set_pen'),
            (self._teleport_client, '/turtle1/teleport_absolute'),
        ]:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {name}...')

        self._set_pen(off=True)
        self._reset_pose()

        self.create_timer(0.05, self._tick)
        self.get_logger().info(
            'Draw Boundaries started. '
            'Run keyboard_node in another terminal to toggle manual mode.')

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _on_pose(self, msg):
        """Store turtle pose and record home position on first message."""
        self._pose = msg
        if self._home is None:
            self._home = msg

    def _on_manual_cmd(self, msg):
        """Store incoming manual velocity (used only in MANUAL state)."""
        self._manual_twist = msg

    def _on_toggle(self, _msg):
        """Toggle between MANUAL and the last auto state."""
        if self._state != State.MANUAL:
            self._state_before_manual = self._state
            self._manual_twist = Twist()
            self._transition(State.MANUAL)
            self.get_logger().info('MANUAL -- keyboard_node now controls the turtle.')
        else:
            self._transition(self._state_before_manual)
            self._manual_twist = Twist()
            self.get_logger().info('AUTO resumed.')

    # ------------------------------------------------------------------
    # Service helpers
    # ------------------------------------------------------------------

    def _set_pen(self, r=255, g=255, b=255, width=2, off=False):
        """Call set_pen service asynchronously."""
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = off
        self._pen_client.call_async(req)

    def _reset_pose(self):
        """Teleport turtle to center facing east for a deterministic start."""
        req = TeleportAbsolute.Request()
        req.x = 5.544
        req.y = 5.544
        req.theta = 0.0
        self._teleport_client.call_async(req)

    # ------------------------------------------------------------------
    # FSM tick (50 Hz timer)
    # ------------------------------------------------------------------

    def _tick(self):
        """Run one FSM cycle."""
        if self._pose is None:
            return
        if self._state == State.MANUAL:
            self.cmd_pub.publish(self._manual_twist)
        elif self._state == State.MOVING_TO_BOUNDARY:
            self._tick_moving()
        elif self._state == State.FOLLOWING_BOUNDARY:
            self._tick_following()
        elif self._state == State.RETURNING_HOME:
            self._tick_returning()
        elif self._state == State.IDLE:
            self._stop()

    def _tick_moving(self):
        """Move east until hitting the boundary, then start tracing it."""
        wall = self._detect_wall()
        if wall != Wall.NONE:
            self._stop()
            self._wall = wall
            self._boundary_start = (self._pose.x, self._pose.y)
            self._boundary_moved_away = False
            self._set_pen(r=255, g=200, b=0, width=3, off=False)
            self._transition(State.FOLLOWING_BOUNDARY)
            self.get_logger().info(f'Boundary reached ({wall.value}), pen down.')
        else:
            self._steer_toward(0.0)

    def _tick_following(self):
        """Follow boundary clockwise with pen down; detect when loop is closed."""
        wall = self._detect_wall()
        if wall != Wall.NONE and wall != self._wall:
            self._stop()
            self._wall = wall
            self.get_logger().info(f'Corner: now following {wall.value} wall.')

        self._steer_toward(WALL_HEADING[self._wall], wall=self._wall)

        if self._boundary_start is not None:
            dx = self._pose.x - self._boundary_start[0]
            dy = self._pose.y - self._boundary_start[1]
            dist = math.sqrt(dx ** 2 + dy ** 2)
            if not self._boundary_moved_away and dist > 2.0:
                self._boundary_moved_away = True
            if self._boundary_moved_away and dist < 0.4:
                self._set_pen(off=True)
                self._transition(State.RETURNING_HOME)
                self.get_logger().info('Boundary closed! Returning home.')

    def _tick_returning(self):
        """Navigate back to the home position."""
        dx = self._home.x - self._pose.x
        dy = self._home.y - self._pose.y
        dist = math.sqrt(dx ** 2 + dy ** 2)
        if dist < 0.3:
            self._stop()
            self._transition(State.IDLE)
            self.get_logger().info('Home reached. Done.')
            return
        self._steer_toward(math.atan2(dy, dx))

    # ------------------------------------------------------------------
    # Motion helpers
    # ------------------------------------------------------------------

    def _detect_wall(self):
        """Return which wall the turtle is near, or Wall.NONE."""
        x, y = self._pose.x, self._pose.y
        if x >= WORLD_MAX:
            return Wall.RIGHT
        if y >= WORLD_MAX:
            return Wall.TOP
        if x <= WORLD_MIN:
            return Wall.LEFT
        if y <= WORLD_MIN:
            return Wall.BOTTOM
        return Wall.NONE

    def _normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _steer_toward(self, heading, wall=None):
        """Steer toward heading; turn in place for large errors, arc for small ones."""
        error = self._normalize_angle(heading - self._pose.theta)
        twist = Twist()
        if abs(error) > 0.3:
            twist.angular.z = max(-3.0, min(3.0, TURN_KP * error))
        elif abs(error) > ANGLE_TOL:
            twist.angular.z = max(-3.0, min(3.0, TURN_KP * error))
            twist.linear.x = SPEED * 0.3
        else:
            twist.linear.x = SPEED
            twist.angular.z = self._wall_correction(wall)
        self.cmd_pub.publish(twist)

    def _wall_correction(self, wall):
        """Return a small angular correction to keep the turtle from drifting into the wall."""
        if wall is None:
            return 0.0
        x, y = self._pose.x, self._pose.y
        if wall == Wall.RIGHT:
            # heading north: drift east is bad, correct CCW (positive)
            return max(-1.0, min(1.0, (WORLD_MAX - WALL_MARGIN - x) * 0.5))
        if wall == Wall.LEFT:
            # heading south: drift west is bad, correct CW (negative)
            return max(-1.0, min(1.0, (x - WORLD_MIN - WALL_MARGIN) * -0.5))
        if wall == Wall.TOP:
            # heading west: drift north is bad, correct CW (negative)
            return max(-1.0, min(1.0, (WORLD_MAX - WALL_MARGIN - y) * 0.5))
        if wall == Wall.BOTTOM:
            # heading east: drift south is bad, correct CCW (positive)
            return max(-1.0, min(1.0, (y - WORLD_MIN - WALL_MARGIN) * -0.5))
        return 0.0

    def _stop(self):
        """Publish zero velocity."""
        self.cmd_pub.publish(Twist())

    def _transition(self, new_state):
        """Apply FSM state transition."""
        self.get_logger().info(
            f'FSM: {self._state.value} -> {new_state.value}')
        self._state = new_state


def main(args=None):
    """Entry point."""
    rclpy.init(args=args)
    node = DrawBoundariesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
