import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
import rclpy.duration

from irobot_create_msgs.action import AudioNoteSequence, Dock, Undock
from irobot_create_msgs.msg import AudioNote, HazardDetectionVector
from turtle_square_interfaces.action import DrawSquare

from patrol_fsm.fsm_states import PatrolFSM, PatrolState
from patrol_fsm.avoidance import AvoidanceManeuver
from patrol_fsm.siren import build_siren_goal


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

        # Shared callback group for all action clients + subscribers
        self.cb_group = ReentrantCallbackGroup()

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/Robot2/cmd_vel', 10)

        # Action clients
        self._undock_client = ActionClient(
            self, Undock, '/Robot2/undock', callback_group=self.cb_group)
        self._dock_client = ActionClient(
            self, Dock, '/Robot2/dock', callback_group=self.cb_group)
        self._draw_square_client = ActionClient(
            self, DrawSquare, '/draw_square', callback_group=self.cb_group)
        self._audio_client = ActionClient(
            self, AudioNoteSequence, '/Robot2/audio_note_sequence',
            callback_group=self.cb_group)

        # Guards to prevent re-sending goals
        self._undock_sent = False
        self._dock_sent = False
        self._patrol_sent = False
        self._patrol_goal_handle = None

        # Bump side stored at hazard detection time
        self._bump_side = 'CENTER'

        # Hazard subscriber
        self.create_subscription(
            HazardDetectionVector,
            '/Robot2/hazard_detection',
            self._hazard_cb,
            10,
            callback_group=self.cb_group,
        )

        # Services
        self.start_srv = self.create_service(
            Trigger, '/patrol/start', self._handle_start,
            callback_group=self.cb_group)
        self.stop_srv = self.create_service(
            Trigger, '/patrol/stop', self._handle_stop,
            callback_group=self.cb_group)

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
        self._cancel_patrol()
        self.get_logger().info(f'STOP → state: {self.fsm.state.name}')
        response.success = True
        response.message = f'State: {self.fsm.state.name}'
        return response

    # ── Hazard subscriber ─────────────────────────────────────────────────────

    def _hazard_cb(self, msg):
        self.get_logger().info(f'Hazard detections received: {len(msg.detections)}')
        if self.fsm.state != PatrolState.PATROLLING:
            return

        # Look for any BUMP-type detection
        for detection in msg.detections:
            # HazardDetection.type == 1 is BUMP in irobot_create_msgs
            if detection.type == 1:
                # Determine bump side from sensor header frame_id
                # frame_id examples: "bump_left", "bump_right", "bump_front_center"
                frame = detection.header.frame_id.lower()
                if 'left' in frame:
                    self._bump_side = 'LEFT'
                elif 'right' in frame:
                    self._bump_side = 'RIGHT'
                else:
                    self._bump_side = 'CENTER'

                self.get_logger().info(
                    f'[HAZARD] BUMP detected — side: {self._bump_side}')
                self._cancel_patrol()
                self.fsm.on_hazard()
                self.get_logger().info(f'State → {self.fsm.state.name}')
                self._enter_intruder_alert()
                return  # process only the first bump per message

    # ── State dispatcher ───────────────────────────────────────────────────────

    def _dispatch_state(self):
        state = self.fsm.state
        if state == PatrolState.UNDOCKING:
            self._enter_undocking()
        elif state == PatrolState.PATROLLING:
            self._enter_patrolling()
        elif state == PatrolState.DOCKING:
            self._enter_docking()

    # ── UNDOCKING ──────────────────────────────────────────────────────────────

    def _enter_undocking(self):
        if self._undock_sent:
            return
        if not self._undock_client.server_is_ready():
            self.get_logger().warn('/Robot2/undock server not ready yet — retrying next tick')
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
        goal_handle.get_result_async().add_done_callback(self._undock_result_cb)

    def _undock_result_cb(self, future):
        self._undock_sent = False
        self.get_logger().info('Undock complete')
        self.fsm.on_undock_done()
        self.get_logger().info(f'State → {self.fsm.state.name}')
        self._enter_patrolling()

    # ── PATROLLING ─────────────────────────────────────────────────────────────

    def _enter_patrolling(self):
        if self._patrol_sent:
            return
        if not self._draw_square_client.server_is_ready():
            self.get_logger().warn('/draw_square server not ready yet — retrying next tick')
            return
        side_length = self.get_parameter('side_length').value
        patrol_speed = self.get_parameter('patrol_speed').value

        goal = DrawSquare.Goal()
        goal.side_length = float(side_length)
        goal.speed = float(patrol_speed)

        self._patrol_sent = True
        self.get_logger().info(
            f'Sending DrawSquare goal — lap {self.fsm.laps_done + 1}/{self.fsm.num_laps} '
            f'(side={side_length} m, speed={patrol_speed} m/s)'
        )
        future = self._draw_square_client.send_goal_async(
            goal, feedback_callback=self._patrol_feedback_cb)
        future.add_done_callback(self._patrol_goal_response_cb)

    def _patrol_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('DrawSquare goal rejected')
            self._patrol_sent = False
            return
        self._patrol_goal_handle = goal_handle
        self.get_logger().info('DrawSquare goal accepted')
        goal_handle.get_result_async().add_done_callback(self._patrol_result_cb)

    def _patrol_feedback_cb(self, feedback_msg):
        self.get_logger().info(
            f'[PATROLLING] remaining distance: {feedback_msg.feedback.remaining_distance:.2f} m',
            throttle_duration_sec=1.0,
        )

    def _patrol_result_cb(self, future):
        self._patrol_sent = False
        self._patrol_goal_handle = None

        if self.fsm.state != PatrolState.PATROLLING:
            self.get_logger().info(
                f'DrawSquare result ignored — state is {self.fsm.state.name}')
            return

        self.fsm.on_lap_done()
        self.get_logger().info(
            f'Lap done — laps={self.fsm.laps_done}/{self.fsm.num_laps} '
            f'state → {self.fsm.state.name}'
        )
        if self.fsm.state == PatrolState.RETURNING:
            self._enter_returning()
        else:
            self._enter_patrolling()

    def _cancel_patrol(self):
        if self._patrol_goal_handle is not None:
            self.get_logger().info('Cancelling DrawSquare goal...')
            self._patrol_goal_handle.cancel_goal_async()
            self._patrol_goal_handle = None
        self._patrol_sent = False

    # ── INTRUDER_ALERT ────────────────────────────────────────────────────────

    def _enter_intruder_alert(self):
        self.get_logger().info('[INTRUDER_ALERT] Sending siren goal...')
        if not self._audio_client.server_is_ready():
            self.get_logger().warn('/Robot2/audio_note_sequence not ready — skipping siren')
            self.fsm.on_siren_done()
            self._enter_avoiding()
            return

        def _dur(ms):
            return rclpy.duration.Duration(seconds=ms / 1000.0).to_msg()

        goal = build_siren_goal(AudioNoteSequence, AudioNote, _dur)
        future = self._audio_client.send_goal_async(goal)
        future.add_done_callback(self._siren_goal_response_cb)

    def _siren_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Siren goal rejected — going straight to AVOIDING')
            if self.fsm.state == PatrolState.INTRUDER_ALERT:
                self.fsm.on_siren_done()
                self._enter_avoiding()
            return
        self.get_logger().info('Siren goal accepted')
        goal_handle.get_result_async().add_done_callback(self._siren_result_cb)

    def _siren_result_cb(self, future):
        self.get_logger().info('Siren complete')
        if self.fsm.state != PatrolState.INTRUDER_ALERT:
            return
        self.fsm.on_siren_done()
        self.get_logger().info(f'State → {self.fsm.state.name}')
        self._enter_avoiding()

    # ── AVOIDING ─────────────────────────────────────────────────────────────

    def _enter_avoiding(self):
        self.get_logger().info(
            f'[AVOIDING] Starting avoidance maneuver — bump side: {self._bump_side}')
        maneuver = AvoidanceManeuver(
            node=self,
            bump_side=self._bump_side,
            done_callback=self._avoidance_done_cb,
        )
        maneuver.start()

    def _avoidance_done_cb(self):
        if self.fsm.state != PatrolState.AVOIDING:
            return
        self.fsm.on_avoidance_done()
        self.get_logger().info(f'Avoidance done → state: {self.fsm.state.name}')
        self._enter_patrolling()

    # ── RETURNING placeholder (Step 8) ────────────────────────────────────────

    def _enter_returning(self):
        self.get_logger().info(
            '[RETURNING] — navigation toward dock implemented in Step 8')

    # ── DOCKING ────────────────────────────────────────────────────────────────

    def _enter_docking(self):
        if self._dock_sent:
            return
        if not self._dock_client.server_is_ready():
            self.get_logger().warn('/Robot2/dock server not ready yet — retrying next tick')
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
        goal_handle.get_result_async().add_done_callback(self._dock_result_cb)

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
        state = self.fsm.state
        if state == PatrolState.UNDOCKING and not self._undock_sent:
            self._enter_undocking()
        elif state == PatrolState.PATROLLING and not self._patrol_sent:
            self._enter_patrolling()
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