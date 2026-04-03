import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from irobot_create_msgs.action import Dock, Undock

from patrol_fsm.fsm_states import PatrolFSM, PatrolState


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

        # Shared callback group for all action clients
        self.cb_group = ReentrantCallbackGroup()

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/Robot2/cmd_vel', 10)

        # Action clients
        self._undock_client = ActionClient(
            self, Undock, '/Robot2/undock', callback_group=self.cb_group)
        self._dock_client = ActionClient(
            self, Dock, '/Robot2/dock', callback_group=self.cb_group)

        # Guards to prevent re-sending goals
        self._undock_sent = False
        self._dock_sent = False

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
            self._dispatch_state()
            response.success = True
            response.message = f'State: {self.fsm.state.name}'
        except ValueError as e:
            self.get_logger().warn(str(e))
            response.success = False
            response.message = str(e)
        return response

    def _handle_stop(self, request, response):
        self.fsm.on_stop_service()
        self.cmd_pub.publish(Twist())
        self.get_logger().info(f'STOP → state: {self.fsm.state.name}')
        response.success = True
        response.message = f'State: {self.fsm.state.name}'
        return response

    # ── State dispatcher ───────────────────────────────────────────────────────

    def _dispatch_state(self):
        state = self.fsm.state
        if state == PatrolState.UNDOCKING:
            self._enter_undocking()
        elif state == PatrolState.DOCKING:
            self._enter_docking()

    # ── UNDOCKING ──────────────────────────────────────────────────────────────

    def _enter_undocking(self):
        if self._undock_sent:
            return
        if not self._undock_client.server_is_ready():
            self.get_logger().warn('/Robot2/undock server not ready yet — retrying next tick')
            self._undock_sent = False
            return
        self._undock_sent = True
        self.get_logger().info('Sending Undock goal...')
        future = self._undock_client.send_goal_async(Undock.Goal())
        future.add_done_callback(self._undock_goal_response_cb)

    def _undock_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Undock goal rejected')
            self._undock_sent = False
            return
        self.get_logger().info('Undock goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._undock_result_cb)

    def _undock_result_cb(self, future):
        self._undock_sent = False
        self.get_logger().info('Undock complete')
        self.fsm.on_undock_done()
        self.get_logger().info(f'State → {self.fsm.state.name}')
        self._enter_patrolling()

    # ── PATROLLING placeholder (implemented in Step 5) ─────────────────────────

    def _enter_patrolling(self):
        self.get_logger().info(
            f'[PATROLLING] laps={self.fsm.laps_done}/{self.fsm.num_laps} '
            '— DrawSquare action client added in Step 5'
        )

    # ── DOCKING ────────────────────────────────────────────────────────────────

    def _enter_docking(self):
        if self._dock_sent:
            return
        if not self._dock_client.server_is_ready():
            self.get_logger().warn('/Robot2/dock server not ready yet — retrying next tick')
            self._dock_sent = False
            return
        self._dock_sent = True
        self.get_logger().info('Sending Dock goal...')
        future = self._dock_client.send_goal_async(Dock.Goal())
        future.add_done_callback(self._dock_goal_response_cb)

    def _dock_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Dock goal rejected')
            self._dock_sent = False
            return
        self.get_logger().info('Dock goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._dock_result_cb)

    def _dock_result_cb(self, future):
        self._dock_sent = False
        self.fsm.on_dock_done()
        self.get_logger().info(f'Dock complete — state → {self.fsm.state.name}')

    # ── Timer ─────────────────────────────────────────────────────────────────

    def _timer_cb(self):
        self.get_logger().info(
            f'[FSM] state={self.fsm.state.name}  laps={self.fsm.laps_done}/{self.fsm.num_laps}',
            throttle_duration_sec=1.0,
        )
        # Retry dispatch if a goal hasn't been sent yet (server may not have been ready)
        state = self.fsm.state
        if state == PatrolState.UNDOCKING and not self._undock_sent:
            self._enter_undocking()
        elif state == PatrolState.DOCKING and not self._dock_sent:
            self._enter_docking()


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
