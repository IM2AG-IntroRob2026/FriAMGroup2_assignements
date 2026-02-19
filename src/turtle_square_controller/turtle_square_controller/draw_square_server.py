
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from turtle_square_interfaces.action import DrawSquare
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

import math
import time


class TurtleSquareActionServer(Node):
    def __init__(self):
        super().__init__('turtle_square_action_server')
        
     
        self.declare_parameter('side_length', 2.0)
        self.declare_parameter('speed', 1.0)
        
     
        self.callback_group = ReentrantCallbackGroup()
        
   
        self.current_pose = None
        
      
        self.pose_subscriber = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10,
            callback_group=self.callback_group
        )
        
        
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )
        
      
        self.action_server = ActionServer(
            self,
            DrawSquare,
            '/draw_square',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )
        
        self.get_logger().info('Turtle Square Action Server started')
    
    def pose_callback(self, msg):
        """Update current pose of the turtle"""
        self.current_pose = msg
    
    
    def goal_callback(self, goal_request):
        """Accept or reject goal requests"""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT
        
    
    def cancel_callback(self, goal_handle):
        """Handle cancellation requests"""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    def execute_callback(self, goal_handle):
        """Execute the square drawing action"""
        self.get_logger().info('Executing goal...')
        
   
        side_length = goal_handle.request.side_length
        speed = goal_handle.request.speed
      
        if side_length == 0.0:
            side_length = self.get_parameter('side_length').value
        if speed == 0.0:
            speed = self.get_parameter('speed').value
        
        self.get_logger().info(f'Drawing square: side_length={side_length}, speed={speed}')
        
        
        while self.current_pose is None and rclpy.ok():
            self.get_logger().info('Waiting for turtle pose...')
            time.sleep(0.1)
        
      
        total_distance = 4 * side_length
        completed_distance = 0.0
        
        feedback_msg = DrawSquare.Feedback()
        
        for side in range(4):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = DrawSquare.Result()
                result.success = False
                return result
            
            self.get_logger().info(f'Drawing side {side + 1}/4')
            
           
            if not self.move_forward(side_length, speed, goal_handle, feedback_msg, completed_distance, total_distance):
                goal_handle.canceled()
                result = DrawSquare.Result()
                result.success = False
                return result
            
            completed_distance += side_length
            
          
            if not self.rotate(math.pi / 2, goal_handle):
                goal_handle.canceled()
                result = DrawSquare.Result()
                result.success = False
                return result
        
       
        goal_handle.succeed()
        result = DrawSquare.Result()
        result.success = True
        
        self.get_logger().info('Square completed successfully!')
        return result
    
    def move_forward(self, distance, speed, goal_handle, feedback_msg, completed_distance, total_distance):
        """Move the turtle forward by the specified distance"""
       
        start_x = self.current_pose.x
        start_y = self.current_pose.y
    
        vel_msg = Twist()
        vel_msg.linear.x = speed
        vel_msg.angular.z = 0.0
        
    
        rate = self.create_rate(10)  
        
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                vel_msg.linear.x = 0.0
                self.cmd_vel_publisher.publish(vel_msg)
                return False
            
           
            dx = self.current_pose.x - start_x
            dy = self.current_pose.y - start_y
            distance_traveled = math.sqrt(dx * dx + dy * dy)
            
        
            remaining = total_distance - (completed_distance + distance_traveled)
            feedback_msg.remaining_distance = max(0.0, remaining)
            goal_handle.publish_feedback(feedback_msg)
            
           
            if distance_traveled >= distance:
                break
            
         
            self.cmd_vel_publisher.publish(vel_msg)
            rate.sleep()
        
    
        vel_msg.linear.x = 0.0
        self.cmd_vel_publisher.publish(vel_msg)
        time.sleep(0.1)
        
        return True
    
    def rotate(self, angle, goal_handle):
     
        start_theta = self.current_pose.theta
        
     
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 1.0  
        
   
        rate = self.create_rate(10) 
        
        while rclpy.ok():
          
            if goal_handle.is_cancel_requested:
                vel_msg.angular.z = 0.0
                self.cmd_vel_publisher.publish(vel_msg)
                return False
            
            
            angle_rotated = abs(self.normalize_angle(self.current_pose.theta - start_theta))
            
          
            if angle_rotated >= angle - 0.05:  
                break
            
           
            self.cmd_vel_publisher.publish(vel_msg)
            rate.sleep()
        
    
        vel_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(vel_msg)
        time.sleep(0.1)
        
        return True
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    
    action_server = TurtleSquareActionServer()
    
    executor = MultiThreadedExecutor()
    executor.add_node(action_server)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()