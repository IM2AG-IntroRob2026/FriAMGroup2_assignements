import sys
import tty
import termios
import select
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


TELEOP_LINEAR = 2.0
TELEOP_ANGULAR = 1.5

KEY_UP     = '\x1b[A'
KEY_DOWN   = '\x1b[B'
KEY_LEFT   = '\x1b[D'
KEY_RIGHT  = '\x1b[C'
KEY_SPACE  = ' '
KEY_CTRL_C = '\x03'


class TeleopHandler(Node):

    def __init__(self):
        super().__init__('teleop_handler')

        self.manual_mode_pub = self.create_publisher(Bool, '/manual_mode', 10)
        self.vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.is_manual = False

        # Current velocity command, reset each tick (requires key held for continuous motion)
        self.current_linear = 0.0
        self.current_angular = 0.0

        # Publish velocity at 10 Hz when in manual mode
        self.create_timer(0.1, self._publish_vel_if_manual)

        self.get_logger().info(
            'TeleopHandler ready.\n'
            '  SPACE     : toggle manual / autonomous\n'
            '  Arrow keys: move turtle (manual mode only)'
        )

    def _publish_vel_if_manual(self):
        if not self.is_manual:
            return
        twist = Twist()
        twist.linear.x = self.current_linear
        twist.angular.z = self.current_angular
        self.vel_pub.publish(twist)
        # Reset so the turtle stops when the key is released
        self.current_linear = 0.0
        self.current_angular = 0.0

    def handle_key(self, key: str):
        if key == KEY_SPACE:
            self.is_manual = not self.is_manual
            msg = Bool()
            msg.data = self.is_manual
            self.manual_mode_pub.publish(msg)
            mode = 'MANUAL' if self.is_manual else 'AUTONOMOUS'
            self.get_logger().info(f'Switched to {mode} mode')

        elif self.is_manual:
            if key == KEY_UP:
                self.current_linear = TELEOP_LINEAR
                self.current_angular = 0.0
            elif key == KEY_DOWN:
                self.current_linear = -TELEOP_LINEAR
                self.current_angular = 0.0
            elif key == KEY_LEFT:
                self.current_linear = 0.0
                self.current_angular = TELEOP_ANGULAR
            elif key == KEY_RIGHT:
                self.current_linear = 0.0
                self.current_angular = -TELEOP_ANGULAR


def _read_key() -> str:
    """Read one key or escape sequence from stdin."""
    ch = sys.stdin.read(1)
    if ch == '\x1b':
        ch2 = sys.stdin.read(1)
        if ch2 == '[':
            ch3 = sys.stdin.read(1)
            return '\x1b[' + ch3
        return ch + ch2
    return ch


def main(args=None):
    rclpy.init(args=args)
    node = TeleopHandler()

    # Spin in a background thread so callbacks (timer) keep firing
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while rclpy.ok():
            # Non-blocking read with 100 ms timeout
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = _read_key()
                if key == KEY_CTRL_C:
                    break
                node.handle_key(key)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
