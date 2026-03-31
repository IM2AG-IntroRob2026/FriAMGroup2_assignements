"""
keyboard_teleop.py – Keyboard interface for the boundary controller.

Controls
--------
  SPACE      : toggle between MANUAL and AUTONOMOUS mode
  ↑ / ↓      : move forward / backward (manual mode only)
  ← / →      : rotate left / right     (manual mode only)
  Ctrl-C     : quit

Communication
-------------
Publishes /manual_mode (std_msgs/Bool)
  True  when entering manual mode, False when leaving.

Publishes /turtle1/cmd_vel (geometry_msgs/Twist)
  Only while in manual mode, at 10 Hz. Velocity resets to zero each tick so
  the turtle stops as soon as the key is released.
"""

import select
import sys
import termios
import threading
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool


# ── Velocity settings ─────────────────────────────────────────────────────────
LINEAR_SPEED  = 2.0   # m/s
ANGULAR_SPEED = 1.5   # rad/s

# ── Key codes ─────────────────────────────────────────────────────────────────
KEY_SPACE  = ' '
KEY_UP     = '\x1b[A'
KEY_DOWN   = '\x1b[B'
KEY_LEFT   = '\x1b[D'
KEY_RIGHT  = '\x1b[C'
KEY_CTRL_C = '\x03'


class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')

        self._mode_pub = self.create_publisher(Bool,  '/manual_mode',       10)
        self._vel_pub  = self.create_publisher(Twist, '/turtle1/cmd_vel',   10)

        self._manual = False
        self._lin    = 0.0
        self._ang    = 0.0

        # Publish velocity at 10 Hz; reset after each publish so the turtle
        # stops when no key is held.
        self.create_timer(0.1, self._publish_vel)

        self.get_logger().info(
            '\n'
            '  SPACE  : toggle manual / autonomous\n'
            '  Arrows : move turtle (manual mode only)\n'
            '  Ctrl-C : quit\n')

    def _publish_vel(self):
        if not self._manual:
            return
        msg = Twist()
        msg.linear.x  = self._lin
        msg.angular.z = self._ang
        self._vel_pub.publish(msg)
        self._lin = self._ang = 0.0   # reset → turtle stops when key released

    def handle_key(self, key: str):
        if key == KEY_SPACE:
            self._manual = not self._manual
            m = Bool()
            m.data = self._manual
            self._mode_pub.publish(m)
            label = 'MANUAL' if self._manual else 'AUTONOMOUS'
            self.get_logger().info(f'Mode → {label}')

        elif self._manual:
            if   key == KEY_UP:    self._lin =  LINEAR_SPEED
            elif key == KEY_DOWN:  self._lin = -LINEAR_SPEED
            elif key == KEY_LEFT:  self._ang =  ANGULAR_SPEED
            elif key == KEY_RIGHT: self._ang = -ANGULAR_SPEED


def _read_key() -> str:
    """Read one key or escape sequence (blocking)."""
    ch = sys.stdin.read(1)
    if ch == '\x1b':
        # Arrow keys send ESC [ A/B/C/D
        rest = sys.stdin.read(2)
        return ch + rest
    return ch


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()

    # Spin ROS callbacks in a daemon thread so the main thread can own stdin.
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fd  = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
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
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
