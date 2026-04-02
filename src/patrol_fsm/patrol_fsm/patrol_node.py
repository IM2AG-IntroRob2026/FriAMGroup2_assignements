import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

from patrol_fsm.fsm_states import PatrolFSM


class PatrolNode(Node):

    def __init__(self):
        super().__init__('patrol_node')

        # Parameters
        self.declare_parameter('num_laps', 3)
        self.declare_parameter('side_length', 2.0)
        self.declare_parameter('patrol_speed', 0.3)
        self.declare_parameter('siren_duration', 3.0)

        # FSM
        num_laps = self.get_parameter('num_laps').value
        self.fsm = PatrolFSM(num_laps=num_laps)

        # Shared callback group for all action clients (added in later steps)
        self.cb_group = ReentrantCallbackGroup()

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/Robot2/cmd_vel', 10)

        # Services
        self.start_srv = self.create_service(
            Trigger,
            '/patrol/start',
            self._handle_start,
            callback_group=self.cb_group,
        )
        self.stop_srv = self.create_service(
            Trigger,
            '/patrol/stop',
            self._handle_stop,
            callback_group=self.cb_group,
        )

        # 10 Hz state-logging timer
        self.create_timer(0.1, self._timer_cb)

        self.get_logger().info('PatrolNode started. State: DOCKED')

    # ── Service handlers ───────────────────────────────────────────────────────

    def _handle_start(self, request, response):
        try:
            self.fsm.on_start()
            self.get_logger().info(f'START → state: {self.fsm.state.name}')
            response.success = True
            response.message = f'State: {self.fsm.state.name}'
        except ValueError as e:
            self.get_logger().warn(str(e))
            response.success = False
            response.message = str(e)
        return response

    def _handle_stop(self, request, response):
        self.fsm.on_stop_service()
        # Publish zero Twist to halt motion
        self.cmd_pub.publish(Twist())
        self.get_logger().info(f'STOP → state: {self.fsm.state.name}')
        response.success = True
        response.message = f'State: {self.fsm.state.name}'
        return response

    # ── Timer ─────────────────────────────────────────────────────────────────

    def _timer_cb(self):
        self.get_logger().info(
            f'[FSM] state={self.fsm.state.name}  laps={self.fsm.laps_done}/{self.fsm.num_laps}',
            throttle_duration_sec=1.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
