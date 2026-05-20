import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from std_msgs.msg import String, Bool
from interfaces_pkg.msg import PathPlanningResult, DetectionArray, MotionCommand
from .lib import decision_making_func_lib as DMFL

#---------------Variable Setting---------------
SUB_DETECTION_TOPIC_NAME = "detections"
SUB_PATH_TOPIC_NAME = "path_planning_result"
SUB_TRAFFIC_LIGHT_TOPIC_NAME = "yolov8_traffic_light_info"
SUB_LIDAR_OBSTACLE_TOPIC_NAME = "lidar_obstacle_info"
PUB_TOPIC_NAME = "topic_control_signal"

# 제어 모드 ('event' = path 들어올 때 즉시 발행 + watchdog 정지 폴백,
#            'periodic' = 고정 주기 발행)
CONTROL_MODE = 'periodic'

# periodic 모드 발행 주기 (초) - 소수점 필요
PERIODIC_PUBLISH_PERIOD = 0.01

# event 모드 watchdog 체크 주기 (초)
WATCHDOG_PERIOD = 0.1

# event 모드에서 path 입력이 끊겼다고 판단하는 timeout (초)
PATH_TIMEOUT_SEC = 0.5

# 정상 주행 시 좌/우 속도 (0~255)
DEFAULT_LEFT_SPEED = 80
DEFAULT_RIGHT_SPEED = 80

# 실험 중 control publish path에 불필요한 callback/log 부하를 넣지 않기 위한 기본값
ENABLE_AUX_SUBSCRIPTIONS = False
LOG_COMMANDS = False

# 종료(SIGINT/SIGTERM) 시 정지 명령을 도배할 횟수와 간격 (초)
# CAN/DDS 누락 대비용. 총 SHUTDOWN_STOP_REPEATS * SHUTDOWN_STOP_INTERVAL 초간 정지 명령 송출
SHUTDOWN_STOP_REPEATS = 10
SHUTDOWN_STOP_INTERVAL = 0.05

#----------------------------------------------


class MotionPlanningNode(Node):
    def __init__(self):
        super().__init__('motion_planner_node')

        # 토픽 이름 설정
        self.sub_detection_topic = self.declare_parameter('sub_detection_topic', SUB_DETECTION_TOPIC_NAME).value
        self.sub_path_topic = self.declare_parameter('sub_lane_topic', SUB_PATH_TOPIC_NAME).value
        self.sub_traffic_light_topic = self.declare_parameter('sub_traffic_light_topic', SUB_TRAFFIC_LIGHT_TOPIC_NAME).value
        self.sub_lidar_obstacle_topic = self.declare_parameter('sub_lidar_obstacle_topic', SUB_LIDAR_OBSTACLE_TOPIC_NAME).value
        self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value

        # 제어 모드 파라미터
        self.control_mode = self.declare_parameter('control_mode', CONTROL_MODE).value
        self.periodic_period = self.declare_parameter('periodic_period', PERIODIC_PUBLISH_PERIOD).value
        self.watchdog_period = self.declare_parameter('watchdog_period', WATCHDOG_PERIOD).value
        self.path_timeout_sec = self.declare_parameter('path_timeout_sec', PATH_TIMEOUT_SEC).value
        self.enable_aux_subscriptions = self.declare_parameter(
            'enable_aux_subscriptions', ENABLE_AUX_SUBSCRIPTIONS).value
        self.log_commands = self.declare_parameter('log_commands', LOG_COMMANDS).value

        if self.control_mode not in ('event', 'periodic'):
            self.get_logger().warn(
                f"Unknown control_mode '{self.control_mode}', falling back to 'event'")
            self.control_mode = 'event'

        # QoS 설정
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # 변수 초기화
        self.detection_data = None
        self.path_data = None
        self.traffic_light_data = None
        self.lidar_data = None

        self.last_path_time = None  # event 모드 watchdog용

        self.steering_command = 0
        self.left_speed_command = 0
        self.right_speed_command = 0
        self.last_frame_id = 0   # E2E latency 측정용 frame_id pass-through

        # 서브스크라이버 설정
        self.path_sub = self.create_subscription(
            PathPlanningResult, self.sub_path_topic, self.path_callback, self.qos_profile)
        self.detection_sub = None
        self.traffic_light_sub = None
        self.lidar_sub = None
        if self.enable_aux_subscriptions:
            self.detection_sub = self.create_subscription(
                DetectionArray, self.sub_detection_topic, self.detection_callback, self.qos_profile)
            self.traffic_light_sub = self.create_subscription(
                String, self.sub_traffic_light_topic, self.traffic_light_callback, self.qos_profile)
            self.lidar_sub = self.create_subscription(
                Bool, self.sub_lidar_obstacle_topic, self.lidar_callback, self.qos_profile)

        # 퍼블리셔 설정
        self.publisher = self.create_publisher(MotionCommand, self.pub_topic, self.qos_profile)

        # 모드별 타이머 설정
        if self.control_mode == 'periodic':
            self.timer = self.create_timer(self.periodic_period, self.periodic_callback)
            self.get_logger().info(
                f"Motion planner started in PERIODIC mode "
                f"(period={self.periodic_period}s, aux_subscriptions={self.enable_aux_subscriptions})")
        else:
            # event 모드: path_callback이 메인 트리거, 별도 watchdog 타이머만 가동
            self.watchdog_timer = self.create_timer(self.watchdog_period, self.watchdog_callback)
            self.get_logger().info(
                f"Motion planner started in EVENT mode "
                f"(watchdog={self.watchdog_period}s, path_timeout={self.path_timeout_sec}s, "
                f"aux_subscriptions={self.enable_aux_subscriptions})")

        # 종료 시그널(SIGINT/SIGTERM) 발생 시 정지 명령 broadcast 후 종료하기 위한 플래그
        # rclpy 기본 SIGINT 핸들러 대신 우리 핸들러를 등록
        self._shutting_down = False
        signal.signal(signal.SIGINT, self._shutdown_handler)
        signal.signal(signal.SIGTERM, self._shutdown_handler)

    # ---------- Subscriber callbacks ----------
    def detection_callback(self, msg: DetectionArray):
        self.detection_data = msg

    def path_callback(self, msg: PathPlanningResult):
        self.path_data = list(zip(msg.x_points, msg.y_points))
        self.last_path_time = time.monotonic()
        self.last_frame_id = msg.frame_id   # E2E latency 측정용 pass-through
        self.compute_motion_command()

        # event 모드일 때만 즉시 명령 계산/발행
        if self.control_mode == 'event':
            self._publish_current_command()

    def traffic_light_callback(self, msg: String):
        self.traffic_light_data = msg

    def lidar_callback(self, msg: Bool):
        self.lidar_data = msg

    # ---------- Periodic mode ----------
    def periodic_callback(self):
        # # 라이다가 장애물을 감지한 경우 (현 구현에서 라이다 미사용)
        # if self.lidar_data is not None and self.lidar_data.data is True:
        #     self.publish_stop()
        #     return
        #
        # # 빨간색 신호등을 감지한 경우 (현 구현에서 신호등 미사용)
        # if (self.traffic_light_data is not None
        #         and self.traffic_light_data.data == 'Red'
        #         and self.detection_data is not None):
        #     for detection in self.detection_data.detections:
        #         if detection.class_name == 'traffic_light':
        #             y_max = int(detection.bbox.center.position.y + detection.bbox.size.y / 2)
        #             if y_max < 150:
        #                 self.publish_stop()
        #                 return

        self._publish_current_command()

    # ---------- Event mode watchdog ----------
    def watchdog_callback(self):
        # path가 한 번도 안 왔거나, 마지막 path 수신 후 timeout 경과 → 정지 명령 발행
        if (self.last_path_time is None
                or (time.monotonic() - self.last_path_time) > self.path_timeout_sec):
            self.publish_stop()

    # ---------- Core motion computation ----------
    def compute_motion_command(self):
        if self.path_data is None or len(self.path_data) < 10:
            # 경로가 없거나 너무 짧으면 정지
            self.set_stop_command()
            return

        target_slope = DMFL.calculate_slope_between_points(
            self.path_data[-10], self.path_data[-1])

        self.steering_command = convert_steeringangle2command(52, target_slope)
        self.left_speed_command = DEFAULT_LEFT_SPEED
        self.right_speed_command = DEFAULT_RIGHT_SPEED

    def compute_and_publish_motion(self):
        self.compute_motion_command()
        self._publish_current_command()

    def set_stop_command(self):
        self.steering_command = 0
        self.left_speed_command = 0
        self.right_speed_command = 0

    def publish_stop(self):
        self.set_stop_command()
        self._publish_current_command()

    def _publish_current_command(self):
        if self.log_commands:
            self.get_logger().info(
                f"[{self.control_mode}] steering: {self.steering_command}, "
                f"left_speed: {self.left_speed_command}, "
                f"right_speed: {self.right_speed_command}")

        motion_command_msg = MotionCommand()
        motion_command_msg.frame_id = self.last_frame_id   # E2E latency 측정용 pass-through
        motion_command_msg.steering = self.steering_command
        motion_command_msg.left_speed = self.left_speed_command
        motion_command_msg.right_speed = self.right_speed_command
        self.publisher.publish(motion_command_msg)

    # ---------- Graceful shutdown ----------
    def _shutdown_handler(self, signum, frame):
        """SIGINT/SIGTERM 수신 시: 정지 명령을 여러 번 발행한 뒤 rclpy 종료."""
        if self._shutting_down:
            # 두 번째 신호가 와도 무시 (이미 종료 절차 진행 중)
            return
        self._shutting_down = True

        self.get_logger().warn(
            f"Signal {signum} received - broadcasting stop commands before shutdown")

        # 정지 명령 도배 (CAN/DDS 누락 대비)
        for _ in range(SHUTDOWN_STOP_REPEATS):
            try:
                self.publish_stop()
            except Exception as e:
                # 종료 중에는 publisher가 이미 닫혔을 수 있음 - 그 경우 그냥 break
                self.get_logger().error(f"Failed to publish stop during shutdown: {e}")
                break
            time.sleep(SHUTDOWN_STOP_INTERVAL)

        self.get_logger().warn("Stop broadcast complete. Shutting down rclpy.")

        # rclpy가 spin을 빠져나가도록 종료 요청
        if rclpy.ok():
            rclpy.shutdown()


def convert_steeringangle2command(max_target_angle, target_angle):
    f = lambda x: 7 / (max_target_angle ** 3) * (x ** 3)
    ret_direction = round(f(target_angle))

    ret_direction = 7 if ret_direction >= 7 else ret_direction
    ret_direction = -7 if ret_direction <= -7 else ret_direction
    return ret_direction


def main(args=None):
    # rclpy 기본 시그널 핸들러를 비활성화하고, MotionPlanningNode가 SIGINT/SIGTERM을 직접 처리
    # SignalHandlerOptions API가 없는 구버전 ROS2에서는 일반 init으로 폴백
    try:
        rclpy.init(args=args, signal_handler_options=rclpy.SignalHandlerOptions.NO)
    except (AttributeError, TypeError):
        rclpy.init(args=args)

    node = MotionPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # _shutdown_handler가 이미 처리했으므로 여기까지 오는 경우는 거의 없음
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
