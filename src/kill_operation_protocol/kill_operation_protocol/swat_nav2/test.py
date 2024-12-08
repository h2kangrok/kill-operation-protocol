import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math

from swat_interface.msg import TargetStatus  # 필요 없는 TogetherStatus는 제거

class AMRCtrl(Node):
    def __init__(self):
        super().__init__('amr_ctrl')

        # 초기 상태
        self.current_mode = "IDLE"  # IDLE, MOVING, FINDING_TARGET, RETURNING_HOME

        # 기지에서 출발 할때 이동 좌표
        self.goal_list = [
            {'x': 0.3, 'y': -0.0, 'z': 0.0, 'orientation': {'x': 0.0, 'y': 0.0, 'z':  0.0, 'w': 0.9}},
            {'x': 0.1, 'y': -0.6, 'z': 0.0, 'orientation': {'x': 0.0, 'y': 0.0, 'z':  0.9, 'w': 0.0}},
            {'x': -0.6, 'y': -0.2, 'z': 0.0, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.9, 'w': 0.1}}
        ]

        # 타겟을 놓쳤을 때 탐색 좌표
        self.find_goal_list = [
            {'x': -0.9, 'y': 0.1, 'z': 0.0, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.9, 'w': 0.1}},
            {'x': -1.0, 'y': -0.4, 'z': 0.0, 'orientation': {'x': 0.0, 'y': 0.0, 'z': -0.9, 'w': 0.4}},
            {'x': -0.4, 'y': 0.0, 'z': 0.0, 'orientation': {'x': 0.0, 'y': 0.0, 'z': -0.8, 'w': 0.4}},
            {'x': -0.4, 'y': -0.2, 'z': 0.0, 'orientation': {'x': 0.0, 'y': 0.0, 'z': -0.1, 'w': 0.9}},
            {'x': 0.1, 'y': -0.5, 'z': 0.0, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.5, 'w': 0.8}},
            {'x': 0.0, 'y': 0.0, 'z': 0.0, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.9}}
        ]

        # 집으로 복귀 좌표
        self.home_list = [
            {'x': -0.9, 'y': 0.1, 'z': 0.0, 'orientation': {'x': 0.0, 'y': 0.0, 'z':  0.9, 'w': 0.1}},
            {'x': -0.4, 'y': -0.2, 'z': 0.0, 'orientation': {'x': 0.0, 'y': 0.0, 'z': -0.1, 'w': 0.9}},
            {'x': 0.1, 'y': -0.5, 'z': 0.0, 'orientation': {'x': 0.0, 'y': 0.0, 'z':  0.5, 'w': 0.8}},
            {'x': 0.0, 'y': 0.0, 'z': 0.0, 'orientation': {'x': 0.0, 'y': 0.0, 'z':  0.0, 'w': 0.9}}
        ]
        
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 타겟 상태 구독
        self.target_subscription = self.create_subscription(
            TargetStatus,
            '/world_view/target_status',
            self.target_listener_callback,
            10
        )

        self.is_target_active = False
        self.is_moving = False
        self.is_goal_status = False

        self._goal_handle = None
        self.goal_index = 0
        self.find_goal_index = 0
        self.home_index = 0

        self.timer = self.create_timer(1.0, self.check_sub_and_server) # 상태 체크 타이머

    def check_sub_and_server(self):
        # 액션 서버 및 토픽 구독자 상태 확인
        sub_init_count = self.init_pub.get_subscription_count()
        sub_cmd_vel = self.cmd_vel_pub.get_subscription_count()
        init_sub = sub_init_count > 0
        cmd_vel_sub = sub_cmd_vel > 0

        if not init_sub and not cmd_vel_sub:
            self.get_logger().warn("/initialpose 및 /cmd_vel 구독 노드 없음.")
        else:
            self.get_logger().info(f"/initialpose 구독 수: {sub_init_count}, /cmd_vel 구독 수: {sub_cmd_vel}")

        is_action_server_connected = self.action_client.wait_for_server(timeout_sec=1.0)
        if not is_action_server_connected:
            self.get_logger().error("navigate_to_pose 액션 서버와 연결되지 않음.")
        else:
            self.get_logger().info("navigate_to_pose 액션 서버와 연결됨.")

        if init_sub and is_action_server_connected and cmd_vel_sub:
            self.get_logger().info("모두 연결됨. 타이머 종료. SWAT AMR 출동 준비 완료")
            self.timer.cancel()

    def publish_initial_pose(self):
        # 초기 위치 설정
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()

        initial_pose.pose.pose.position.x = 0.031250059604644775
        initial_pose.pose.pose.position.y = 0.015624867752194405
        initial_pose.pose.pose.position.z = 0.0

        initial_pose.pose.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=-0.041560396205254015, w=0.9991359934799978
        )

        initial_pose.pose.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.1,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.01,0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01
        ]

        self.init_pub.publish(initial_pose)
        self.get_logger().info('initial pose 설정 완료')

    def send_goal(self):
        if self.goal_index >= len(self.goal_list):
            self.get_logger().info('모든 목표 지점 도달 완료')
            self.is_moving = False
            self.is_goal_status = False
            self.current_mode = "IDLE"
            return

        current_goal = self.goal_list[self.goal_index]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = current_goal['x']
        goal_msg.pose.pose.position.y = current_goal['y']
        goal_msg.pose.pose.position.z = current_goal['z']

        goal_msg.pose.pose.orientation.x = current_goal['orientation']['x']
        goal_msg.pose.pose.orientation.y = current_goal['orientation']['y']
        goal_msg.pose.pose.orientation.z = current_goal['orientation']['z']
        goal_msg.pose.pose.orientation.w = current_goal['orientation']['w']
        
        self.action_client.wait_for_server()
        self.get_logger().info(f'{self.goal_index+1}번째 목표 전송: ({current_goal["x"]}, {current_goal["y"]})')
        
        self._send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('목표 거부됨')
            return

        self.get_logger().info('목표 수락됨, 결과 대기 중...')
        self._goal_handle = goal_handle 
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'현재 위치: {feedback_msg.feedback.current_pose.pose}')

    def goal_result_callback(self, future):
        try:
            result = future.result()
            if result.status == 4:
                self.get_logger().info(f'{self.goal_index+1}번째 목표 도달')
                self.goal_index += 1
                self.send_goal()
            else:
                self.get_logger().info(f'{self.goal_index+1}번째 목표 실패, 상태 코드: {result.status}')
        except Exception as e:
            self.get_logger().error(f'목표 결과 처리 중 오류: {e}')

    def start_with_delay(self):
        self.publish_initial_pose()
        self.get_logger().info('5초 후 목표 전송 시작...')
        # 한 번만 실행하기 위해 타이머를 변수에 담고 콜백 내에서 취소
        self.delay_timer = self.create_timer(5.0, self.start_goal_after_delay)

    def start_goal_after_delay(self):
        # 타이머 한 번 실행 후 취소
        self.delay_timer.cancel()
        self.send_goal()

    def cancel_goal(self):
        if self._goal_handle is not None:
            self.get_logger().info('목표 취소 시도 중...')
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        else:
            self.get_logger().info('취소할 활성 목표 없음')

    def cancel_done_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('목표 취소 완료, 기지 복귀 시작...')
            # 복귀 모드일 경우 go_home 호출
            if self.current_mode == "RETURNING_HOME":
                self.go_home()
            else:
                # RETURNING_HOME 상태가 아닐 경우, 여기서 종료할 수도 있음
                # 필요 시 로직 추가 가능
                self.get_logger().info('현재 RETURNING_HOME 상태가 아니므로 별도 동작 없음.')
        else:
            self.get_logger().info('목표 취소 실패 또는 활성 목표 없음')

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2)*math.cos(pitch/2)*math.cos(yaw/2) - math.cos(roll/2)*math.sin(pitch/2)*math.sin(yaw/2)
        qy = math.cos(roll/2)*math.sin(pitch/2)*math.cos(yaw/2) + math.sin(roll/2)*math.cos(pitch/2)*math.sin(yaw/2)
        qz = math.cos(roll/2)*math.cos(pitch/2)*math.sin(yaw/2) - math.sin(roll/2)*math.sin(pitch/2)*math.cos(yaw/2)
        qw = math.cos(roll/2)*math.cos(pitch/2)*math.cos(yaw/2) + math.sin(roll/2)*math.sin(pitch/2)*math.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)
    
    def send_find_target_car_goal(self):
        # FINDING_TARGET 모드일 때 호출
        if self.find_goal_index >= len(self.find_goal_list):
            self.get_logger().info('탐색 목표 지점 모두 도달')
            self.current_mode = "IDLE"
            return

        current_goal = self.find_goal_list[self.find_goal_index]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = current_goal['x']
        goal_msg.pose.pose.position.y = current_goal['y']
        goal_msg.pose.pose.position.z = current_goal['z']
        goal_msg.pose.pose.orientation.x = current_goal['orientation']['x']
        goal_msg.pose.pose.orientation.y = current_goal['orientation']['y']
        goal_msg.pose.pose.orientation.z = current_goal['orientation']['z']
        goal_msg.pose.pose.orientation.w = current_goal['orientation']['w']
        
        self.action_client.wait_for_server()
        self.get_logger().info(f'탐색 {self.find_goal_index+1}번째 목표 전송: ({current_goal["x"]}, {current_goal["y"]})')
        
        self._send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.find_goal_response_callback)
        
    def find_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('탐색 목표 거부됨')
            return

        self.get_logger().info('탐색 목표 수락됨, 결과 대기 중...')
        self._goal_handle = goal_handle
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self.find_goal_result_callback)

    def find_goal_result_callback(self, future):
        try:
            result = future.result()
            if result.status == 4:
                self.get_logger().info(f'탐색 {self.find_goal_index+1}번째 목표 도달')
                self.find_goal_index += 1
                self.send_find_target_car_goal()
            else:
                self.get_logger().info(f'탐색 {self.find_goal_index+1}번째 목표 실패, 상태 코드: {result.status}')
        except Exception as e:
            self.get_logger().error(f'탐색 목표 결과 오류: {e}')

    def go_home(self):
        self.get_logger().info('기지 복귀 시작')
        self.home_index = 0
        self.send_home_goal()

    def send_home_goal(self):
        if self.home_index >= len(self.home_list):
            self.get_logger().info('기지 복귀 완료')
            self.current_mode = "IDLE"
            return

        current_goal = self.home_list[self.home_index]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = current_goal['x']
        goal_msg.pose.pose.position.y = current_goal['y']
        goal_msg.pose.pose.position.z = current_goal['z']
        goal_msg.pose.pose.orientation.x = current_goal['orientation']['x']
        goal_msg.pose.pose.orientation.y = current_goal['orientation']['y']
        goal_msg.pose.pose.orientation.z = current_goal['orientation']['z']
        goal_msg.pose.pose.orientation.w = current_goal['orientation']['w']

        self.action_client.wait_for_server()
        self.get_logger().info(f'기지 복귀 {self.home_index+1}번째 목표 전송: ({current_goal["x"]}, {current_goal["y"]})')
        self._send_goal_future = self.action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.home_goal_response_callback)

    def home_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('기지 복귀 목표 거부됨')
            return

        self.get_logger().info('기지 복귀 목표 수락됨, 결과 대기 중...')
        self._goal_handle = goal_handle
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self.home_goal_result_callback)

    def home_goal_result_callback(self, future):
        try:
            result = future.result()
            if result.status == 4:
                self.get_logger().info(f'기지 복귀 {self.home_index+1}번째 목표 도달')
                self.home_index += 1
                self.send_home_goal()
            else:
                self.get_logger().info(f'기지 복귀 {self.home_index+1}번째 목표 실패, 상태 코드: {result.status}')
        except Exception as e:
            self.get_logger().error(f'기지 복귀 결과 처리 오류: {e}')

    def target_listener_callback(self, msg):
        if msg.target_status:
            # 타겟 감지
            if not self.is_target_active:
                self.get_logger().info('타겟 차량 발견! 출동 시작.')
                self.is_target_active = True
                self.is_moving = True
                self.is_goal_status = True
                self.current_mode = "MOVING"
                self.start_with_delay()
            else:
                self.get_logger().info('타겟 차량 추적 중...')
        
        elif msg.target_status is None:
            self.get_logger().error('타겟 상태 확인 불가, 시스템 점검 필요.')
            self.cancel_goal()
            return

        else:
            # 타겟 상실
            if self.is_moving and self.is_target_active and self.is_goal_status:
                # 이동 중 타겟 상실
                if self.current_mode == "MOVING":
                    self.get_logger().info('이동 중 타겟 상실, 복귀 모드 전환.')
                    self.is_target_active = False
                    self.is_moving = False
                    self.is_goal_status = False
                    self.current_mode = "RETURNING_HOME"
                    self.cancel_goal()
            elif not self.is_moving and self.is_target_active:
                # 목표 도착 후 타겟 상실
                if self.current_mode != "FINDING_TARGET":
                    self.get_logger().info('타겟 상실, 탐색 모드 전환.')
                    self.current_mode = "FINDING_TARGET"
                    # 5초 뒤 탐색 시작
                    self.find_timer = self.create_timer(5.0, self.start_find_after_delay)

    def start_find_after_delay(self):
        self.find_timer.cancel()
        self.send_find_target_car_goal()

def main(args=None):
    rclpy.init(args=args)
    node = AMRCtrl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
