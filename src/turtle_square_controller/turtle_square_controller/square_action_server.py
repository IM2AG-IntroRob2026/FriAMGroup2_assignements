#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtle_square_interfaces.action import DrawSquare
import math
import time


class SquareActionServer(Node):
    def __init__(self):
        super().__init__('square_action_server')

        # Parameters
        self.declare_parameter('side_length', 2.0)
        self.declare_parameter('speed', 1.0)

        # ReentrantCallbackGroup allows pose_callback to run concurrently
        # with the action execute_callback
        self.cb_group = ReentrantCallbackGroup()

        # Publisher and Subscriber
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose',
                                                  self.pose_callback, 10,
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
        """Accept cancel requests"""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT


    def pose_callback(self, msg):
        """Store current turtle position"""
        self.current_pose = msg

    def execute_callback(self, goal_handle):
        """Execute the square drawing action"""
        self.get_logger().info('Executing goal...')

        # Get parameters (goal overrides defaults)
        side_length = goal_handle.request.side_length if goal_handle.request.side_length > 0 \
                      else self.get_parameter('side_length').value
        speed = goal_handle.request.speed if goal_handle.request.speed > 0 \
                else self.get_parameter('speed').value

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
            if not self.move_forward(side_length, speed, feedback_msg, goal_handle):
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


    def move_forward(self, distance, speed, feedback_msg, goal_handle):
        """Move turtle forward by distance"""
        if self.current_pose is None:
            return False

        # Record start position
        start_x = self.current_pose.x
        start_y = self.current_pose.y

        twist = Twist()
        twist.linear.x = speed

        while rclpy.ok():
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                self.stop()
                return False

            # Calculate distance traveled
            dx = self.current_pose.x - start_x
            dy = self.current_pose.y - start_y
            traveled = math.sqrt(dx**2 + dy**2)

            # Send feedback
            feedback_msg.remaining_distance = (distance * 4) - traveled
            goal_handle.publish_feedback(feedback_msg)

            if traveled >= distance:
                break

            self.cmd_pub.publish(twist)
            time.sleep(0.1)
        # Stop
        self.stop()
        return True


    def rotate(self, angle_degrees, goal_handle):
        """Rotate turtle by angle in degrees"""
        if self.current_pose is None:
            return False

        # Compute exact target angle
        target_theta = self.normalize_angle(
            self.current_pose.theta + angle_degrees * math.pi / 180.0
        )

        twist = Twist()

        while rclpy.ok():
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                return False

            # self-corrects if overshoot
            error = self.normalize_angle(target_theta - self.current_pose.theta)

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
        """Stop the turtle"""
        twist = Twist()
        self.cmd_pub.publish(twist)
        time.sleep(0.1)

    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to [-pi, pi]"""
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

