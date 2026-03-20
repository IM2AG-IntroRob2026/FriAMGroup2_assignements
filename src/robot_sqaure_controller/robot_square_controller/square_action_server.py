#!/usr/bin/env python3
import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from turtle_square_interfaces.action import DrawSquare


class SquareActionServer(Node):

    def __init__(self):
        super().__init__('square_action_server')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # Parameters
        self.declare_parameter('side_length', 2.0)
        self.declare_parameter('speed', 1.0)

        # ReentrantCallbackGroup allows odom_callback to run concurrently
        # with the action execute_callback
        self.cb_group = ReentrantCallbackGroup()

        # Publisher and Subscriber
        self.cmd_pub = self.create_publisher(Twist, '/Robot2/cmd_vel', 10)
        self.pose_sub = self.create_subscription(
            Odometry,
            '/Robot2/odom',
            self.odom_callback,
            qos_profile,
            callback_group=self.cb_group)

        # Current pose
        self.current_pose = None

        # Action Server (with cancel_callback to support Ctrl+C)
        self._action_server = ActionServer(
            self,
            DrawSquare,
            '/draw_square',
            self.execute_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info('Square Action Server started!')

    def cancel_callback(self, goal_handle):
        """Accept cancel requests."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def odom_callback(self, msg):
        """Store current robot position."""
        self.current_pose = msg.pose.pose
        pos = self.current_pose.position
        ori = self.current_pose.orientation
        self.get_logger().info(
            f'Position: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}')
        self.get_logger().info(
            f'Orientation: x={ori.x:.2f}, y={ori.y:.2f}, '
            f'z={ori.z:.2f}, w={ori.w:.2f}')

    def execute_callback(self, goal_handle):
        """Execute the square drawing action."""
        self.get_logger().info('Executing goal...')

        # Get parameters (goal overrides defaults)
        req = goal_handle.request
        side_length = (req.side_length if req.side_length > 0
                       else self.get_parameter('side_length').value)
        speed = (req.speed if req.speed > 0
                 else self.get_parameter('speed').value)

        feedback_msg = DrawSquare.Feedback()

        # Draw 4 sides
        for side in range(4):
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled!')
                self.stop()
                goal_handle.canceled()
                result = DrawSquare.Result()
                result.success = False
                return result

            self.get_logger().info(f'Drawing side {side + 1}/4')

            # Move forward
            if not self.move_forward(side_length, speed, feedback_msg, goal_handle, side):
                goal_handle.abort()
                result = DrawSquare.Result()
                result.success = False
                return result

            # Rotate 90 degrees
            if not self.rotate(90.0, goal_handle):
                self.stop()
                goal_handle.canceled()
                result = DrawSquare.Result()
                result.success = False
                return result

        # Success
        goal_handle.succeed()
        result = DrawSquare.Result()
        result.success = True
        return result

    def move_forward(self, distance, speed, feedback_msg, goal_handle, side):
        """Move robot forward by distance."""
        if self.current_pose is None:
            return False

        # Record start position
        start_x = self.current_pose.position.x
        start_y = self.current_pose.position.y

        twist = Twist()
        twist.linear.x = speed

        while rclpy.ok():
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                self.stop()
                return False

            # Calculate distance traveled
            dx = self.current_pose.position.x - start_x
            dy = self.current_pose.position.y - start_y
            traveled = math.sqrt(dx**2 + dy**2)

            # Send feedback
            feedback_msg.remaining_distance = distance * (4 - side) - traveled
            goal_handle.publish_feedback(feedback_msg)

            if traveled >= distance:
                break

            self.cmd_pub.publish(twist)
            time.sleep(0.1)
        # Stop
        self.stop()
        return True

    def get_yaw(self):
        """Extract yaw from current_pose quaternion."""
        q = self.current_pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def rotate(self, angle_degrees, goal_handle):
        """Rotate robot by angle in degrees."""
        if self.current_pose is None:
            return False

        # Compute exact target angle
        target_theta = self.normalize_angle(
            self.get_yaw() + angle_degrees * math.pi / 180.0
        )

        twist = Twist()

        while rclpy.ok():
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                return False

            # self-corrects if overshoot
            error = self.normalize_angle(target_theta - self.get_yaw())

            if abs(error) < 0.005:  # ~0.3 degrees
                break

            # Proportional control: direction auto-corrects via sign of error
            speed = max(0.2, min(2.0, abs(error) * 3.0))
            if error > 0:
                twist.angular.z = speed
            else:
                twist.angular.z = -speed
            self.cmd_pub.publish(twist)
            time.sleep(0.02)

        self.stop()
        return True

    def stop(self):
        """Stop the robot."""
        twist = Twist()
        self.cmd_pub.publish(twist)
        time.sleep(0.1)

    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    server = SquareActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(server)
    executor.spin()
    server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
