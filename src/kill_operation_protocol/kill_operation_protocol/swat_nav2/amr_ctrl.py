import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import threading
import sys
import math


from swat_interface.msg import TargetStatus, TogetherStatus, RobotStatus, TrackingMsg  # Import custom message types

class AMRCtrl(Node):
    def __init__(self):
        super().__init__('amr_ctrl')
        # 기지에서 출발 할때 이동 좌표
        self.goal_list = [
            {'x': 0.3, 'y': -0.0, 'z': 0.0, 'orientation': {'x': 0.0, 'y': 0.0, 'z': -0.0, 'w': 0.9}},
            {'x': 0.1, 'y': -0.6, 'z': 0.0, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.9, 'w': 0.0}},
            {'x': -0.6, 'y': -0.2, 'z': 0.0, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.9, 'w': 0.1}}
        ]

        # 타켓을 놓쳤을 때
        self.find_goal_list = [
            {'x': -0.9, 'y': 0.0, 'z': 0.0, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.9, 'w': 0.1}},
            {'x': -0.5, 'y': 0.0, 'z': 0.0, 'orientation': {'x': 0.0, 'y': 0.0, 'z': -0.9, 'w': 0.3}},
            {'x': -0.5, 'y': -0.5, 'z': 0.0, 'orientation': {'x': 0.0, 'y': 0.0, 'z': -0.9, 'w': 0.3}},
            {'x': 0.1, 'y': -0.5, 'z': 0.0, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.5, 'w': 0.8}},
            {'x': 0.0, 'y': 0.0, 'z': 0.0, 'orientation': {'x': 0.0, 'y': 0.0, 'z': -0.0, 'w': 0.9}}
        ]

        # 집으로 복귀
        # self.home_list = [
        #     {'x': -0.9, 'y': 0.1, 'z': 0.0, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.9, 'w': 0.1}},
        #     {'x': -0.4, 'y': -0.2, 'z': 0.0, 'orientation': {'x': 0.0, 'y': 0.0, 'z': -0.1, 'w': 0.9}},
        #     {'x': 0.1, 'y': -0.5, 'z': 0.0, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.5, 'w': 0.8}},
        #     {'x': 0.0, 'y': 0.0, 'z': 0.0, 'orientation': {'x': 0.0, 'y': 0.0, 'z': -0.0, 'w': 0.9}}
        # ]
        

        self.move_status_pub = self.create_publisher(RobotStatus, 'robot_status', 10) # 웹페이지로 동작 전송
        self.tracking_msg_pub = self.create_publisher(TrackingMsg, 'tracking_msg_pub', 10)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscription to TargetStatus
        self.target_subscription = self.create_subscription(
            TargetStatus,
            '/world_view/target_status',  # Topic to subscribe to
            self.target_listener_callback,
            10  # Queue size
        )

        self.is_target_active = False  # 현재 타겟 상태 추적
        self.is_moving = False  # 출동 중인지 확인
        
        self.is_goal_status = False # 목표 지점 이동 여부

        # Subscription to TogetherStatus
        self.together_subscription = self.create_subscription(
            TogetherStatus,
            '/world_view/together_status',  # Topic to subscribe to
            self.target_listener_callback,
            10  # Queue size
        )

        self._goal_handle = None
        self.goal_index = 0
        self.find_goal_index = 0

        self.timer = self.create_timer(1.0, self.check_sub_and_server) # 1초마다 상태를 체크하기 위한 타이머 생성
        
        self.move_msg = RobotStatus()

    # bringup이랑 nav2연결 확인 부분
    def check_sub_and_server(self):
        """
        액션 서버와 구독자 연결 상태를 확인하고, 두 조건이 충족되면 타이머를 중단
        """
        
        sub_init_count = self.init_pub.get_subscription_count() # 구독자 상태 확인
        sub_cmd_vel = self.cmd_vel_pub.get_subscription_count() # 구독자 상태 확인
        init_sub = sub_init_count > 0
        cmd_vel_sub = sub_cmd_vel > 0

        if not init_sub and not cmd_vel_sub:
            self.get_logger().warn("/initialpose 토픽 및 /cmd_vel을 구독 중인 노드가 없습니다.")
        elif cmd_vel_sub or init_sub:
            self.get_logger().warn(f"/initialpose : {sub_init_count}, /cmd_vel : {cmd_vel_sub}")
        else:
            self.get_logger().info(f"/initialpose 토픽을 구독 중인 노드: {sub_init_count}개")
        
        # 액션 서버 상태 확인
        is_action_server_connected = self.action_client.wait_for_server(timeout_sec=1.0)
        if not is_action_server_connected:
            self.get_logger().error("navigate_to_pose 액션 서버와 연결되지 않았습니다.")
        else:
            self.get_logger().info("navigate_to_pose 액션 서버와 연결되었습니다.")

        # 두 조건이 모두 충족되면 타이머 중단
        if init_sub and is_action_server_connected and cmd_vel_sub:
            self.get_logger().info("모두 연결됨. 타이머를 중단합니다.")
            self.get_logger().info('SWAT AMR 출동 준비 완료')
            self.timer.cancel()

    # 초기 위치 잡는 부분
    def publish_initial_pose(self):
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()

        initial_pose.pose.pose.position.x = 0.031250059604644775
        initial_pose.pose.pose.position.y = 0.015624867752194405
        initial_pose.pose.pose.position.z = 0.0

        initial_pose.pose.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=-0.041560396205254015,
            w=0.9991359934799978
        )

        initial_pose.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.1,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.01]

        self.init_pub.publish(initial_pose)
        self.get_logger().info('initial pose success')

    # 목표 상태
    def send_goal(self):
        if self.goal_index >= len(self.goal_list):
            self.get_logger().info('모든 목표 지점에 도달했습니다.')
            self.is_moving = False # 목적지에 도착해서 안움직임
            self.move_msg.move_status = self.is_moving
            self.move_status_pub.publish(self.move_msg)

            self.is_goal_status = False
            
            track_msg = TrackingMsg() #트래킹 전송
            track_msg.tracking_msg = "start"
            self.tracking_msg_pub.publish(track_msg)

            # self.goal_index = 0
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        current_goal = self.goal_list[self.goal_index]
        goal_msg.pose.pose.position.x = current_goal['x']
        goal_msg.pose.pose.position.y = current_goal['y']
        goal_msg.pose.pose.position.z = current_goal['z']

        goal_msg.pose.pose.orientation.x = current_goal['orientation']['x']
        goal_msg.pose.pose.orientation.y = current_goal['orientation']['y']
        goal_msg.pose.pose.orientation.z = current_goal['orientation']['z']
        goal_msg.pose.pose.orientation.w = current_goal['orientation']['w']
        
        self.action_client.wait_for_server()
        self.get_logger().info(f'{self.goal_index + 1}번째 목표를 전송 중: ({current_goal["x"]}, {current_goal["y"]})')
        
        self._send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    # 목표 승인 여부
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('목표가 거부되었습니다.')
            return

        self.get_logger().info('목표가 수락되었습니다.')
        self._goal_handle = goal_handle 

        self.get_logger().info('결과를 기다리는 중...')
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    # 현재 위치 피드백
    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'현재 위치: {feedback_msg.feedback.current_pose.pose}')

    # 몇 번째 목표 지점 도달 완료 알려주는 부분
    def goal_result_callback(self, future):
        try:
            result = future.result()
            if result.status == 4:
                self.get_logger().info(f'{self.goal_index + 1}번째 목표에 성공적으로 도달했습니다.')
                self.goal_index += 1
                self.send_goal()
            else:
                self.get_logger().info(f'{self.goal_index + 1}번째 목표에 실패했습니다. 상태 코드: {result.status}')
        except Exception as e:
            self.get_logger().error(f'목표 결과 처리 중 오류 발생: {e}')

    # 목표 지점 이동
    def start_with_delay(self):
        self.publish_initial_pose()
        self.get_logger().info('5초 대기 후 목표를 전송합니다...')
        self.create_timer(5.0, self.send_goal) 
    
    # 목표 지점 취소
    def cancel_goal(self):
        print('목표취소: ',self._goal_handle)
        if self._goal_handle is not None:
            self.get_logger().info('목표 취소를 시도하는 중...')
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        else:
            self.get_logger().info('취소할 활성 목표가 없습니다.')

    # 목표 취소 하고 프로그램 종료
    def cancel_done_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('목표가 취소되었습니다. 프로그램을 종료합니다...')
            self.destroy_node()
            rclpy.shutdown()
            sys.exit(0)
        else:
            self.get_logger().info('목표 취소가 실패했거나 활성 목표가 없습니다.')

# =====================================================================================
    def euler_to_quaternion(self, roll, pitch, yaw):
        # Convert Euler angles to a quaternion
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)
    
    def send_find_target_car_goal(self):
        if self.find_goal_index >= len(self.find_goal_list):
            self.get_logger().info('기지에 복귀 완료')
            # self.find_goal_index = 0
            self.destroy_node()
            rclpy.shutdown()
            sys.exit(0)
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        current_goal = self.find_goal_list[self.find_goal_index]
        goal_msg.pose.pose.position.x = current_goal['x']
        goal_msg.pose.pose.position.y = current_goal['y']
        goal_msg.pose.pose.position.z = current_goal['z']

        goal_msg.pose.pose.orientation.x = current_goal['orientation']['x']
        goal_msg.pose.pose.orientation.y = current_goal['orientation']['y']
        goal_msg.pose.pose.orientation.z = current_goal['orientation']['z']
        goal_msg.pose.pose.orientation.w = current_goal['orientation']['w']
        
        self.action_client.wait_for_server()
        self.get_logger().info(f'{self.find_goal_index + 1}번째 목표를 전송 중: ({current_goal["x"]}, {current_goal["y"]})')
        
        self._send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.home_goal_response_callback)
        

    # 목표 승인 여부
    def home_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('목표가 거부되었습니다.')
            return

        self.get_logger().info('목표가 수락되었습니다.')
        self._goal_handle = goal_handle 

        self.get_logger().info('결과를 기다리는 중...')
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self.home_goal_result_callback)

    def home_goal_result_callback(self, future):
        try:
            result = future.result()
            if result.status == 4:
                self.get_logger().info(f'{self.find_goal_index + 1}번째 목표에 성공적으로 도달했습니다.')
                self.find_goal_index += 1
                self.send_find_target_car_goal()
            else:
                self.get_logger().info(f'{self.find_goal_index + 1}번째 목표에 실패했습니다. 상태 코드: {result.status}')
        except Exception as e:
            self.get_logger().error(f'목표 결과 처리 중 오류 발생: {e}')

 # ======================================================================   


    # 적이 작전 지역으로 들어올 경우
    def target_listener_callback(self, msg):
        if msg.target_status:  # 타겟이 감지된 경우
            if not self.is_target_active:  # 새로운 타겟 감지 시에만 처리
                self.get_logger().info('타겟 차량 발견! 출동 시작.')
                self.is_target_active = True
                self.is_moving = True
                self.move_msg.move_status = self.is_moving
                self.move_status_pub.publish(self.move_msg)
                self.is_goal_status = True
                self.start_with_delay()
            else:
                self.get_logger().info('타겟 차량을 계속 추적 중...')
        
        elif msg.target_status is None:
            self.get_logger().error('타겟 상태 확인 불가! 시스템 점검 필요.')
            self.cancel_goal()
            return

        elif not msg.together_status:  # 타겟이 감지되지 않은 경우
            if self.is_moving and self.is_target_active and self.is_goal_status: # 목표지점으로 이동하다 타겟이 사라짐
                self.get_logger().info('이동 중 타겟 차량이 사라짐. 복귀 모드 전환.')
                self.is_target_active = False
                self.is_moving = False
                self.move_msg.move_status = self.is_moving
                self.move_status_pub.publish(self.move_msg)
                # self.cancel_goal() # 이 부분을 복귀모드로 변경
            elif not self.is_moving and self.is_target_active: # 목표지점으로 이동 완료하였는데 타겟이 작전지역에서 벗어남
                self.get_logger().info('타겟 차량이 사라짐. 탐색 모드로 전환.')
                # 이 부분에서 찾는 알고리즘 작성

                track_msg = TrackingMsg() #트래킹 전송
                track_msg.tracking_msg = "stop"
                self.tracking_msg_pub.publish(track_msg)
                
                self.create_timer(5.0, self.send_find_target_car_goal) 

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = AMRCtrl()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('강제 종료')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()