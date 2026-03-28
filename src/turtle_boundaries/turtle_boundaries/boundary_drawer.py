import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute, SetPen
from std_msgs.msg import Bool
import math



DOMAIN_MIN = 1.0
DOMAIN_MAX = 10.0
MOVE_SPEED = 2.0
TURN_SPEED = 1.5
CLOSE_ENOUGH = 0.15
WALL_THRESHOLD = 0.5


ALL_CORNERS = [
    (DOMAIN_MIN, DOMAIN_MIN),   
    (DOMAIN_MAX, DOMAIN_MIN),   
    (DOMAIN_MAX, DOMAIN_MAX),   
    (DOMAIN_MIN, DOMAIN_MAX),  
]


class BoundaryDrawer(Node):

    def __init__(self):
        super().__init__('boundary_drawer')

     
        self.declare_parameter('pen_color_r', 255)
        self.declare_parameter('pen_color_g', 0)
        self.declare_parameter('pen_color_b', 0)
        self.declare_parameter('pen_width', 3)

        r = self.get_parameter('pen_color_r').value
        g = self.get_parameter('pen_color_g').value
        b = self.get_parameter('pen_color_b').value
        self.pen_width = self.get_parameter('pen_width').value
        self.pen_color = (r, g, b)

        # Publisher
        self.vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Subscriber:turtle position
        self.pose_sub = self.create_subscription(
            Pose, '/turtle1/pose', self._pose_callback, 10)

        # Subscriber:manual mode toggle
        self.mode_sub = self.create_subscription(
            Bool, '/manual_mode', self._mode_callback, 10)

        # Service clients
        self.teleport_client = self.create_client(
            TeleportAbsolute, '/turtle1/teleport_absolute')
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')

      
        self.pose= None
        self.state = 'INIT'
        self.manual_mode = False
        self.pre_manual_state = 'EXPLORING'
        self.start_pose = None

        # These are built when wall is first hit
        self.visit_list  = []   
        self.visit_index = 0  

      
        self.timer = self.create_timer(0.05, self._control_loop)

        self.get_logger().info('BoundaryDrawer started. Waiting for pose...')

    #CALLBACKS 
    def _pose_callback(self, msg: Pose):
        self.pose = msg
        if self.state == 'INIT':
            self.start_pose = msg
            self.state = 'EXPLORING'
            self.get_logger().info(
                f'Start position saved: ({msg.x:.2f}, {msg.y:.2f})')
            self._set_pen(up=True)

    def _mode_callback(self, msg: Bool):
        if msg.data and not self.manual_mode:
            self.manual_mode = True
            self.pre_manual_state = self.state
            self.state = 'MANUAL'
            self._stop_turtle()
            self.get_logger().info('MANUAL MODE — use arrow keys. SPACE to resume.')
        elif not msg.data and self.manual_mode:
            self.manual_mode = False
            self.state = self.pre_manual_state
            self.get_logger().info(f'Resuming autonomous (state={self.state})')

    #MAIN CONTROL LOOP 

    def _control_loop(self):
        if self.pose is None:
            return

        if self.state == 'INIT':
            pass
        elif self.state == 'EXPLORING':
            self._do_exploring()
        elif self.state == 'FOLLOWING':
            self._do_following()
        elif self.state == 'RETURNING':
            self._do_returning()
        elif self.state == 'MANUAL':
            pass
        elif self.state == 'DONE':
            self._stop_turtle()

    #STATE BEHAVIORS 

    def _do_exploring(self):
        p = self.pose

        near_wall = (
            p.x <= DOMAIN_MIN + WALL_THRESHOLD or
            p.x >= DOMAIN_MAX - WALL_THRESHOLD or
            p.y <= DOMAIN_MIN + WALL_THRESHOLD or
            p.y >= DOMAIN_MAX - WALL_THRESHOLD
        )

        if near_wall:
            # find which corner is nearest to where we hit the wall
            start_idx = self._nearest_corner_idx(p.x, p.y)

            # build a visit list
            # wrapping around,then back to the start corner to close the square
            self.visit_list = []
            for i in range(5):  
                idx = (start_idx + i) % 4
                self.visit_list.append(ALL_CORNERS[idx])

            self.visit_index = 0
            self._set_pen(up=False)   
            self.state = 'FOLLOWING'
            self.get_logger().info(
                f'Wall hit! Pen down. Starting from corner {start_idx}.')
        else:
            self._publish_vel(linear=MOVE_SPEED, angular=0.0)

    def _do_following(self):
        if self.visit_index >= len(self.visit_list):
            self.get_logger().info('Boundary closed! Returning to start.')
            self._set_pen(up=True)
            self.state = 'RETURNING'
            return

        target_x, target_y = self.visit_list[self.visit_index]
        arrived = self._drive_toward(target_x, target_y)

        if arrived:
            self.get_logger().info(
                f'Reached waypoint {self.visit_index}: ({target_x}, {target_y})')
            self.visit_index += 1

    def _do_returning(self):
        arrived = self._drive_toward(self.start_pose.x, self.start_pose.y)
        if arrived:
            self._stop_turtle()
            self.state = 'DONE'
            self.get_logger().info('Done, Turtle is back at start.')

    # HELPERS 

    def _nearest_corner_idx(self, x: float, y: float) -> int:
        best_idx  = 0
        best_dist = float('inf')
        for i, (cx, cy) in enumerate(ALL_CORNERS):
            d = math.hypot(cx - x, cy - y)
            if d < best_dist:
                best_dist = d
                best_idx  = i
        return best_idx

    def _drive_toward(self, tx: float, ty: float) -> bool:
        p = self.pose
        dx = tx - p.x
        dy = ty - p.y
        dist = math.hypot(dx, dy)

        if dist < CLOSE_ENOUGH:
            self._stop_turtle()
            return True

        target_angle = math.atan2(dy, dx)
        angle_error  = self._normalize_angle(target_angle - p.theta)
        angular      = min(TURN_SPEED, max(-TURN_SPEED, 2.5 * angle_error))

        if dist < 1.0:
            linear = MOVE_SPEED * 0.3
        elif abs(angle_error) < 0.3:
            linear = MOVE_SPEED
        else:
            linear = MOVE_SPEED * 0.3

        self._publish_vel(linear=linear, angular=angular)
        return False

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle >  math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle

    def _publish_vel(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x  = linear
        msg.angular.z = angular
        self.vel_pub.publish(msg)

    def _stop_turtle(self):
        self._publish_vel(0.0, 0.0)

    #SERVICE CALLS

    def _set_pen(self, up: bool):
        if not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('set_pen service not available')
            return

        req       = SetPen.Request()
        req.r     = self.pen_color[0]
        req.g     = self.pen_color[1]
        req.b     = self.pen_color[2]
        req.width = self.pen_width
        req.off   = 1 if up else 0

        self.pen_client.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = BoundaryDrawer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
