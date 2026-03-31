"""
boundary_fsm.py – Autonomous turtle boundary tracer.

FSM states
----------
INIT       : waiting for the first pose message
EXPLORING  : moving straight (pen up), watching for the domain edge
CORNERING  : wall detected; driving to the nearest corner with pen still up,
             so the boundary trace starts exactly at a corner
FOLLOWING  : tracing boundary corners in order (pen down)
RETURNING  : all corners visited; driving back to the start position (pen up)
DONE       : turtle is back at start, stopped

MANUAL mode can interrupt any state except DONE:
  /manual_mode = True  → pause autonomy, publish nothing (teleop drives)
  /manual_mode = False → resume autonomy from the interrupted state
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from std_msgs.msg import Bool


# ── Domain boundaries ────────────────────────────────────────────────────────
X_MIN, X_MAX = 1.0, 10.0
Y_MIN, Y_MAX = 1.0, 10.0

# How close to the domain edge before we switch to CORNERING
WALL_DETECT = 0.6

# The four domain corners, clockwise starting from bottom-left:
#   0=BL  1=BR  2=TR  3=TL
CORNERS = [
    (X_MIN, Y_MIN),
    (X_MAX, Y_MIN),
    (X_MAX, Y_MAX),
    (X_MIN, Y_MAX),
]

# ── Motion tuning ─────────────────────────────────────────────────────────────
MOVE_SPEED  = 2.0    # full linear speed (m/s)
TURN_SPEED  = 2.0    # max angular speed (rad/s)
ARRIVE_DIST = 0.12   # arrival threshold (m)


class BoundaryFSM(Node):

    def __init__(self):
        super().__init__('boundary_fsm')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('pen_color_r', 255)
        self.declare_parameter('pen_color_g', 0)
        self.declare_parameter('pen_color_b', 0)
        self.declare_parameter('pen_width', 3)

        self._pen_rgb = (
            self.get_parameter('pen_color_r').value,
            self.get_parameter('pen_color_g').value,
            self.get_parameter('pen_color_b').value,
        )
        self._pen_width = self.get_parameter('pen_width').value

        # ── ROS interfaces ────────────────────────────────────────────────────
        self._vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_subscription(Pose, '/turtle1/pose', self._on_pose, 10)
        self.create_subscription(Bool, '/manual_mode', self._on_manual_mode, 10)
        self._pen_cli = self.create_client(SetPen, '/turtle1/set_pen')

        # ── FSM bookkeeping ───────────────────────────────────────────────────
        self._pose: Pose | None = None
        self._start_pose: Pose | None = None
        self._state = 'INIT'
        self._pre_manual = 'EXPLORING'   # state to restore after manual mode

        # Built in EXPLORING when a wall is detected:
        #   _waypoints[0]   – nearest corner (driven pen-up in CORNERING)
        #   _waypoints[1..] – remaining corners + return to first (driven pen-down)
        self._waypoints: list[tuple[float, float]] = []
        self._wp_idx = 0

        self.create_timer(0.05, self._loop)   # 20 Hz control loop
        self.get_logger().info('BoundaryFSM ready – waiting for pose…')

    # ── ROS callbacks ─────────────────────────────────────────────────────────

    def _on_pose(self, msg: Pose):
        self._pose = msg
        if self._state == 'INIT':
            self._start_pose = msg
            self._set_pen(down=False)
            self._state = 'EXPLORING'
            self.get_logger().info(
                f'Start: ({msg.x:.2f}, {msg.y:.2f}). Moving…')

    def _on_manual_mode(self, msg: Bool):
        if msg.data and self._state != 'MANUAL':
            self._pre_manual = self._state
            self._state = 'MANUAL'
            self._stop()
            self.get_logger().info('→ MANUAL  (SPACE to resume)')
        elif not msg.data and self._state == 'MANUAL':
            self._state = self._pre_manual
            self.get_logger().info(f'→ AUTO    (resuming {self._state})')

    # ── Control loop ──────────────────────────────────────────────────────────

    def _loop(self):
        if self._pose is None or self._state in ('INIT', 'MANUAL', 'DONE'):
            return

        dispatch = {
            'EXPLORING': self._exploring,
            'CORNERING': self._cornering,
            'FOLLOWING': self._following,
            'RETURNING': self._returning,
        }
        dispatch[self._state]()

    # ── State handlers ────────────────────────────────────────────────────────

    def _exploring(self):
        """Move straight until we're near the domain edge."""
        p = self._pose
        near = (p.x < X_MIN + WALL_DETECT or p.x > X_MAX - WALL_DETECT or
                p.y < Y_MIN + WALL_DETECT or p.y > Y_MAX - WALL_DETECT)
        if near:
            idx = self._nearest_corner(p.x, p.y)
            # 5 waypoints: [corner_0, corner_1, corner_2, corner_3, corner_0]
            # – first one is approached pen-up (CORNERING)
            # – the rest are traced pen-down (FOLLOWING)
            self._waypoints = [CORNERS[(idx + i) % 4] for i in range(5)]
            self._wp_idx = 0
            self._state = 'CORNERING'
            self.get_logger().info(
                f'Wall detected – heading to corner {idx} {CORNERS[idx]}')
        else:
            self._drive(MOVE_SPEED, 0.0)

    def _cornering(self):
        """Reach the first corner with the pen up, then switch to FOLLOWING."""
        if self._drive_toward(*self._waypoints[0]):
            self._wp_idx = 1
            self._set_pen(down=True)
            self._state = 'FOLLOWING'
            self.get_logger().info('Pen DOWN – tracing boundary')

    def _following(self):
        """Trace each corner in order; close the loop and lift the pen."""
        if self._wp_idx >= len(self._waypoints):
            self._set_pen(down=False)
            self._state = 'RETURNING'
            self.get_logger().info('Boundary closed. Pen UP – returning to start')
            return

        tx, ty = self._waypoints[self._wp_idx]
        if self._drive_toward(tx, ty):
            self.get_logger().info(
                f'  corner {self._wp_idx}/{len(self._waypoints) - 1}'
                f' ({tx:.1f}, {ty:.1f}) ✓')
            self._wp_idx += 1

    def _returning(self):
        """Go back to the start position and stop."""
        if self._drive_toward(self._start_pose.x, self._start_pose.y):
            self._state = 'DONE'
            self.get_logger().info('Done – back at start.')

    # ── Motion helpers ─────────────────────────────────────────────────────────

    def _drive_toward(self, tx: float, ty: float) -> bool:
        """Proportional heading + speed control toward (tx, ty).
        Returns True once the turtle is within ARRIVE_DIST."""
        p = self._pose
        dx, dy = tx - p.x, ty - p.y
        dist = math.hypot(dx, dy)

        if dist < ARRIVE_DIST:
            self._stop()
            return True

        angle_err = _wrap(math.atan2(dy, dx) - p.theta)
        angular   = max(-TURN_SPEED, min(TURN_SPEED, 2.5 * angle_err))

        # Slow down when the heading is far off or when very close to the target
        if dist < 0.5 or abs(angle_err) > 0.3:
            linear = MOVE_SPEED * 0.25
        else:
            linear = MOVE_SPEED

        self._drive(linear, angular)
        return False

    def _nearest_corner(self, x: float, y: float) -> int:
        return min(range(4),
                   key=lambda i: math.hypot(CORNERS[i][0] - x, CORNERS[i][1] - y))

    def _drive(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x  = linear
        msg.angular.z = angular
        self._vel_pub.publish(msg)

    def _stop(self):
        self._drive(0.0, 0.0)

    # ── SetPen service ─────────────────────────────────────────────────────────

    def _set_pen(self, down: bool):
        if not self._pen_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('set_pen service not available')
            return
        req       = SetPen.Request()
        req.r     = self._pen_rgb[0]
        req.g     = self._pen_rgb[1]
        req.b     = self._pen_rgb[2]
        req.width = self._pen_width
        req.off   = 0 if down else 1
        self._pen_cli.call_async(req)


# ── Module-level helpers ───────────────────────────────────────────────────────

def _wrap(angle: float) -> float:
    """Wrap angle to [-π, π]."""
    while angle >  math.pi: angle -= 2 * math.pi
    while angle < -math.pi: angle += 2 * math.pi
    return angle


def main(args=None):
    rclpy.init(args=args)
    node = BoundaryFSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
