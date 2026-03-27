#!/usr/bin/env python3
"""Keyboard teleop node: publishes manual commands and mode toggle."""
import sys
import termios
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

TELEOP_SPEED = 2.0
TELEOP_TURN = 2.0

BANNER = """
-- keyboard_node --
  SPACE      toggle manual / auto
  Arrow Up   forward
  Arrow Down backward
  Arrow Left turn left
  Arrow Right turn right
  Ctrl+C     quit
"""


class KeyboardNode(Node):
    """Reads keyboard input and publishes to /toggle_manual and /manual_cmd_vel."""

    def __init__(self):
        """Initialize publishers."""
        super().__init__('keyboard_node')
        self._toggle_pub = self.create_publisher(Empty, '/toggle_manual', 10)
        self._cmd_pub = self.create_publisher(Twist, '/manual_cmd_vel', 10)
        self.get_logger().info(BANNER)

    def run(self):
        """Block and process keyboard input until Ctrl+C."""
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while rclpy.ok():
                ch = sys.stdin.read(1)
                if ch == ' ':
                    self._toggle_pub.publish(Empty())
                elif ch == '\x03':
                    break
                elif ch == '\x1b':
                    seq = sys.stdin.read(2)
                    twist = Twist()
                    if seq == '[A':
                        twist.linear.x = TELEOP_SPEED
                    elif seq == '[B':
                        twist.linear.x = -TELEOP_SPEED
                    elif seq == '[C':
                        twist.angular.z = -TELEOP_TURN
                    elif seq == '[D':
                        twist.angular.z = TELEOP_TURN
                    self._cmd_pub.publish(twist)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main(args=None):
    """Entry point."""
    rclpy.init(args=args)
    node = KeyboardNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
