import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import sys
import math

from swat_interface.msg import TargetStatus, RobotStatus, TrackingMsg  # TogetherStatus는 사용하지 않아 제거


class AMRCtrl(Node):
    def __init__(self):
        super().__init__('amr_ctrl')
        # 목표 지점을 튜플로 관리 (px, py, pz, ox, oy, oz, ow)
        self.goal_list = [
            (0.3, 0.0, 0.0, 0.0, 0.0, -0.0, 0.9),
            (0.1, -0.6, 0.0, 0.0, 0.0, 0.9, 0.0),
            (-0.6, -0.2, 0.0, 0.0, 0.0, 0.9, 0.1)
        ]

        self.find_goal_list = [
            (-1.2, 0.0, 0.0, 0.0, 0.0, 0.9, 0.1),
            (-0.5, 0.0, 0.0, 0.0, 0.0, -0.9, 0.3),
            (-0.5, -0.5, 0.0, 0.0, 0.0, -0.9, 0.3),
            (0.1, -0.5, 0.0, 0.0, 0.0, 0.5, 0.8),
            (0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.9)
        ]

        # 퍼블리셔 및 액션 클라이언트 설정
        self.move_status_pub = self.create_publisher(RobotStatus, 'robot_status', 10)
        self.tracking_msg_pub = self.create_publisher(TrackingMsg, 'tracking_msg_pub', 10)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 서브스크립션
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

        # 재사용할 메시지 객체 미리 생성
        self.move_msg = RobotStatus()
        self.tracking_msg = TrackingMsg()

        self.move_msg.move_status = self.is_moving
        self.move_status_pub.publish(self.move_msg)

        # 상태 체크용 타이머
        self.timer = self.create_timer(1.0, self.check_sub_and_server)

    def check_sub_and_server(self):
        sub_init_count = self.init_pub.get_subscription_count()
        sub_cmd_vel = self.cmd_vel_pub.get_subscription_count()
        init_sub = sub_init_count > 0
        cmd_vel_sub = sub_cmd_vel > 0

        if not init_sub and not cmd_vel_sub:
            self.get_logger().warn("/initialpose 토픽 및 /cmd_vel을 구독 중인 노드가 없습니다.")
        elif cmd_vel_sub or init_sub:
            self.get_logger().warn(f"/initialpose : {sub_init_count}, /cmd_vel : {sub_cmd_vel}")
        else:
            self.get_logger().info(f"/initialpose 토픽을 구독 중인 노드: {sub_init_count}개")

        is_action_server_connected = self.action_client.wait_for_server(timeout_sec=1.0)
        if not is_action_server_connected:
            self.get_logger().error("navigate_to_pose 액션 서버와 연결되지 않았습니다.")
        else:
            self.get_logger().info("navigate_to_pose 액션 서버와 연결되었습니다.")

        if init_sub and is_action_server_connected and cmd_vel_sub:
            self.get_logger().info("모두 연결됨. 타이머를 중단합니다.")
            self.get_logger().info('SWAT AMR 출동 준비 완료')
            self.timer.cancel()

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

    def send_goal(self):
        if self.goal_index >= len(self.goal_list):
            self.get_logger().info('모든 목표 지점에 도달했습니다.')
            self.is_moving = False
            self.move_msg.move_status = self.is_moving
            self.move_status_pub.publish(self.move_msg)

            self.is_goal_status = False

            self.tracking_msg.tracking_msg = "start"
            self.tracking_msg_pub.publish(self.tracking_msg)
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        current_goal = self.goal_list[self.goal_index]
        # current_goal = (px, py, pz, ox, oy, oz, ow)
        goal_msg.pose.pose.position.x = current_goal[0]
        goal_msg.pose.pose.position.y = current_goal[1]
        goal_msg.pose.pose.position.z = current_goal[2]

        goal_msg.pose.pose.orientation.x = current_goal[3]
        goal_msg.pose.pose.orientation.y = current_goal[4]
        goal_msg.pose.pose.orientation.z = current_goal[5]
        goal_msg.pose.pose.orientation.w = current_goal[6]

        self.action_client.wait_for_server()
        self.get_logger().info(f'{self.goal_index + 1}번째 목표를 전송 중: ({current_goal[0]}, {current_goal[1]})')

        self._send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

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

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'현재 위치: {feedback_msg.feedback.current_pose.pose}')

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

    def start_with_delay(self):
        self.publish_initial_pose()
        self.get_logger().info('5초 대기 후 목표를 전송합니다...')
        self.create_timer(5.0, self.send_goal)

    def cancel_goal(self):
        if self._goal_handle is not None:
            self.get_logger().info('목표 취소를 시도하는 중...')
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        else:
            self.get_logger().info('취소할 활성 목표가 없습니다.')

    def cancel_done_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('목표가 취소되었습니다. 프로그램을 종료합니다...')
            self.destroy_node()
            rclpy.shutdown()
            sys.exit(0)
        else:
            self.get_logger().info('목표 취소가 실패했거나 활성 목표가 없습니다.')

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2)*math.sin(pitch / 2)*math.sin(yaw / 2)
        qy = math.cos(roll / 2)*math.sin(pitch / 2)*math.cos(yaw / 2) + math.sin(roll / 2)*math.cos(pitch / 2)*math.sin(yaw / 2)
        qz = math.cos(roll / 2)*math.cos(pitch / 2)*math.sin(yaw / 2) - math.sin(roll / 2)*math.sin(pitch / 2)*math.cos(yaw / 2)
        qw = math.cos(roll / 2)*math.cos(pitch / 2)*math.cos(yaw / 2) + math.sin(roll / 2)*math.sin(pitch / 2)*math.sin(yaw / 2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def send_find_target_car_goal(self):
        if self.find_goal_index >= len(self.find_goal_list):
            self.get_logger().info('기지에 복귀 완료')
            self.destroy_node()
            rclpy.shutdown()
            sys.exit(0)
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        current_goal = self.find_goal_list[self.find_goal_index]
        goal_msg.pose.pose.position.x = current_goal[0]
        goal_msg.pose.pose.position.y = current_goal[1]
        goal_msg.pose.pose.position.z = current_goal[2]

        goal_msg.pose.pose.orientation.x = current_goal[3]
        goal_msg.pose.pose.orientation.y = current_goal[4]
        goal_msg.pose.pose.orientation.z = current_goal[5]
        goal_msg.pose.pose.orientation.w = current_goal[6]

        self.action_client.wait_for_server()
        self.get_logger().info(f'{self.find_goal_index + 1}번째 목표를 전송 중: ({current_goal[0]}, {current_goal[1]})')

        self._send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.home_goal_response_callback)

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

    def target_listener_callback(self, msg):
        if msg.target_status:
            if not self.is_target_active:
                self.get_logger().info('타겟 차량 발견! 출동 시작.')
                self.is_target_active = True
                self.is_moving = True
                self.move_msg.move_status = True
                self.move_status_pub.publish(self.move_msg)
                self.is_goal_status = True
                self.start_with_delay()
            else:
                self.get_logger().info('타겟 차량을 계속 추적 중...')
        elif msg.target_status is None:
            self.get_logger().error('타겟 상태 확인 불가! 시스템 점검 필요.')
            self.cancel_goal()
            return
        else:
            if self.is_moving and self.is_target_active and self.is_goal_status:
                self.get_logger().info('이동 중 타겟 차량이 사라짐. 복귀 모드 전환.')
                self.is_target_active = False
                self.is_moving = False
                self.move_msg.move_status = False
                self.move_status_pub.publish(self.move_msg)
            elif not self.is_moving and self.is_target_active:
                self.get_logger().info('타겟 차량이 사라짐. 탐색 모드로 전환.')
                self.tracking_msg.tracking_msg = "stop"
                self.tracking_msg_pub.publish(self.tracking_msg)
                self.create_timer(5.0, self.send_find_target_car_goal)


def main(args=None):
    rclpy.init(args=args)
    node = AMRCtrl()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
