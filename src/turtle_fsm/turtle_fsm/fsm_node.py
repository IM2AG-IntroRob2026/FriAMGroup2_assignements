import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile
import time
from turtlesim.msg import Pose
from turtlesim.msg import Color

from enum import Enum

# Enumerating the states of the FSM
class TurtleState(Enum):
    FORWARD = 1
    BACKWARD = 2
    TURN = 3

class TurtleFSMNode(Node):
    def __init__(self):
        super().__init__('turtle_fsm1_node')

        # Initial state
        self.state = TurtleState.FORWARD        
        
        # The FSM input is a Pose  
        qos_profile = QoSProfile(depth=10)
        self.in_sub_pose = self.create_subscription(Pose, '/turtle1/pose', self.read_pose, qos_profile)
        self.in_pose = Pose()
        
        #Subscribe to the color sensor
        self.in_sub_color = self.create_subscription(Color, '/turtle1/color_sensor', self.read_color, qos_profile)
        self.in_color = Color()
        # The output of the FSM is cmd_vel
        self.out_pub_vel = self.create_publisher(Twist, '/turtle1/cmd_vel', qos_profile)

 
        # Keeping track of time based on number of cycles
        self.cycle_dt = 0.05   # 20Hz 
        self.cycle_current = 0
        self.cycle_last_transition = 0
        self.time_since_last_transition = 0

        # Timer to control state transitions and publishing
        self.timer = self.create_timer(self.cycle_dt, self.control_cycle)

        
    # Callback for input
    def read_pose(self,msg):
        self.in_pose = msg
    
    # Callback for color
    def read_color(self,msg):
        self.in_color = msg
    
    # Color detection event
    def evt_sees_red(self):
        return self.in_color.r > 200 and self.in_color.g < 50 and self.in_color.b < 50
    

    # Main function, computes output and calls transition function for next cycle
    def control_cycle(self):
        
        # Publish output depending on state
        if self.state == TurtleState.FORWARD:
            self.output_forward()

        elif self.state == TurtleState.BACKWARD:
            self.output_backward()

        elif self.state == TurtleState.TURN:
            self.output_turn()

        self.next_state()   

    def output_forward(self):
        msg = Twist()
        speed = 10.0 if self.evt_sees_red() else 5.0
        msg.linear.x = speed
        msg.angular.z = 0.0
        self.out_pub_vel.publish(msg)


    def output_backward(self):
        msg = Twist()
        msg.linear.x = -1.0  # Move back
        msg.angular.z = 0.0
        self.out_pub_vel.publish(msg)

    def output_turn(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 1.0  # Turn 
        self.out_pub_vel.publish(msg)

    # Transition function
    def next_state(self):
        transition = False #no transition by default
        
        # Handle possible transitions
        if self.state == TurtleState.FORWARD:
            if self.evt_sees_red():
                transition = True
                print("RED DETECTED — GO BACKWARD")
                self.state = TurtleState.BACKWARD
            elif self.evt_out_of_bound():
                transition = True
                print("GO BACKWARD")
                self.state = TurtleState.BACKWARD


        elif self.state == TurtleState.BACKWARD:
            if self.evt_enough_time_spent(1):
                transition = True
                print("GO TURN")                  
                self.state = TurtleState.TURN
        
        elif self.state == TurtleState.TURN:
            if self.evt_enough_time_spent(1):
                transition = True
                print("GO FORWARD")                  
                self.state = TurtleState.FORWARD
        
        # Update cycle count
        self.cycle_current +=1
        if transition:
            self.cycle_last_transition = self.cycle_current
            self.time_since_last_transition = 0
        else:
            self.time_since_last_transition += self.cycle_dt
       
    # Events: return booleans
    def evt_out_of_bound(self):
        x = self.in_pose.x
        y = self.in_pose.y
        x_is_out = x<1 or x>10
        y_is_out = y<1 or y>10
        return x_is_out or y_is_out

    def evt_enough_time_spent(self, dur):
        return self.time_since_last_transition > dur

def main(args=None):
    rclpy.init(args=args)
    node = TurtleFSMNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()