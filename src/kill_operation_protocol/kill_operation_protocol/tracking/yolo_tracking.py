import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np
import math

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, QoSDurabilityPolicy

from std_msgs.msg import String  # shutdown 용 메시지 타입 import

#메세지 임포트 추가
from swat_interface.msg import TrackingMsg  

class YOLOTeleopNode(Node):
    def __init__(self):
        super().__init__("yolo_teleop")


           # QoS 설정
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # RELIABLE로 설정하여 전송 보장
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # YOLO 모델 초기화
        try:
            self.model = YOLO(
                model='/home/rokey/kill-operation-protocol/src/yolo_model/yolov8RobotView_custom_model.pt',
                task='detect',
                verbose=False
            )
            self.get_logger().info("YOLO 모델 로드 성공")
        except Exception as e:
            self.get_logger().error(f"YOLO 모델 로드 실패: {e}")
            rclpy.shutdown()
            return

        # 변수 초기화
        self.target_pos = (160, 200)  # 목표 위치 설정 (이미지 좌표)
        self.offset = (160, 160)
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.image_subscriber = self.create_subscription(CompressedImage, "/camera/image/compressed", self.image_callback, qos_profile)
        self.shutdown_publisher = self.create_publisher(String, "/boom/AMR/shut_down_cam", 10)
        #시작여부 Subscribe
        self.is_start_subscriber = self.create_subscription(TrackingMsg, '/tracking_msg_pub',self.is_start_callback, 10)
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.get_logger().info("YOLO Teleop Node 초기화 완료")

        

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.last_image_time = 0
        self.processing_interval = 0.5   # 초당 2프레임 처리
        self.last_turn = None

        #실행여부 결정 변수
        self.is_start = ""
        self.is_shutdown = False


    #시작여부 받아오면 실행되는 함수
    def is_start_callback(self, msg):
        """
        시작 여부 결정
        """
        self.is_start = msg.tracking_msg

    # def image_callback(self, msg):
    #     """
    #     카메라 이미지 메시지를 수신하고 YOLO 모델을 통해 객체를 탐지.
    #     """
    #     #Image callback 수정 -> Image callback 가장위에 있음 : self.is_start가 False면 이미지 안받아옴
    #     current_time = self.get_clock().now().nanoseconds / 1e9
    #     if current_time - self.last_image_time < self.processing_interval and not self.is_start:
    #         return  # 프레임 처리 간격 유지

    #     self.get_logger().info("이미지 메시지 수신")
    #     self.last_image_time = current_time

    def image_callback(self, msg):
        """
        카메라 이미지 메시지를 수신하고 YOLO 모델을 통해 객체를 탐지.
        """

        if self.is_shutdown:
            self.get_logger().info("노드가 종료 상태입니다. 이미지 처리 중단.")
            return  # 추가 작업 중단

        # self.is_start가 True일 때만 실행
        if self.is_start != "start":
            self.get_logger().info("시작 신호 기다리는 중")
            return
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_image_time < self.processing_interval:
            return  # 프레임 처리 간격 유지

        self.last_image_time = current_time

        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                raise ValueError("디코딩된 이미지가 None입니다.")

        except Exception as e:
            self.get_logger().error(f"이미지 디코딩 실패: {e}")
            return

        # YOLO 객체 탐지
        result = self.detect_objects(frame)
        if result is None:
            return

        frame, detected_objects = self.post_process_img(frame, result)
        self.visualize(frame, detected_objects)

        self.next_action(detected_objects)

    def detect_objects(self, image):
        """
        YOLO 모델을 사용해 입력 이미지를 처리하고 객체를 탐지.
        """
        try:
            result = self.model(image, verbose=False)
            if not result or not hasattr(result[0], "boxes"):
                return None
            return result
        except Exception as e:
            self.get_logger().error(f"YOLO 탐지 오류: {e}")
            return None

    def post_process_img(self, frame, result):
        """
        YOLO 탐지 결과를 후처리하여 신뢰도가 80% 이상인 객체만 필터링.
        """
        detections = result[0].boxes
        if detections is None:
            return frame, []

        xyxy = detections.xyxy.cpu().numpy()
        confidences = detections.conf.cpu().numpy()
        class_ids = detections.cls.cpu().numpy()

        filtered_objects = [
                {"id": int(class_ids[i]), "confidence": confidences[i], "box": xyxy[i]}
                for i in range(len(confidences))
                if class_ids[i] == 1  # 필요 객체 필터링
            ]
        return frame, filtered_objects

    def visualize(self, frame, detected_objects):
        """
        탐지된 객체와 목표 위치를 시각적으로 표시합니다.
        """
        for obj in detected_objects:
            x1, y1, x2, y2 = map(int, obj["box"])
            confidence = obj["confidence"]
            label = f"ID: {obj['id']} ({confidence:.2f})"
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 125, 0), 2)  # 초록색 박스
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

        target_x, target_y = self.target_pos
        cv2.circle(frame, (target_x, target_y), 5, (0, 0, 255), -1)  # 빨간 점
        cv2.imshow("YOLO Detection", frame)
        cv2.waitKey(1)

    def next_action(self, results):
        """
        탐지된 객체의 위치를 기반으로 로봇의 이동 명령을 결정.
        """

        relative_x, relative_y = 0, 0
        target_x, target_y = self.target_pos
        offset_x, offset_y = self.offset
        twist = Twist()

        # 타겟이 탐지되지 않은 경우 제자리 회전
        if all(result["id"] != 1 for result in results): 
            if self.last_turn == "left":
                twist.angular.z = - 0.2  # 30도/초
                self.get_logger().info("목표 탐색중 - 왼쪽 회전")
                
            elif self.last_turn == "right":
                twist.angular.z = 0.2  # 30도/초
                self.get_logger().info("목표 탐색중 - 오른쪽 회전")
            
            else:
                twist.angular.z = 0.0
                self.get_logger().info("초기 탐색중")
                
            self.publisher.publish(twist)
            self.get_logger().info(f"publish : 회전 - {twist.angular.z}")           

        else:
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            for result in results:
                if result["id"] == 1:  # 타겟 ID 확인
                    x1, y1, x2, y2 = result["box"]
                    mid_y = (y1 + y2)/2
                    #mid_x = (x1 + x2)/2

                    # 목표 좌표가 박스 내부에 있는지 확인
                    if x1 <= target_x <= x2 and target_y <= mid_y:
                        shutdown_msg = String()
                        shutdown_msg.data = "Shutdown initiated"
                        self.shutdown_publisher.publish(shutdown_msg)
                        self.get_logger().info("목표 좌표가 박스 내부에 있음. 정지 및 폭파")
                        self.get_logger().info(" AMR 사망")
                        self.get_logger().info("폭파")
                        self.get_logger().info("폭파")
                        self.get_logger().info("폭파")
                        self.get_logger().info("폭파")
                        self.send_goal(0.0, 0.0)  # 정지 명령을 Nav2로 전송
                        self.create_timer(5.0, self.shutdown_node)
                        return

                    # 목표 좌표가 박스 내부에 없을 경우 이동 목표 계산
                    if mid_y < offset_y:
                        relative_x = 0.3  # 로봇 앞으로 0.5m
                        relative_y = 0.0  # 기본적으로 직진

                        # 목표 좌표가 박스의 왼쪽/오른쪽에 있는 경우
                        if target_x < x1:
                            relative_x = 0.3 * np.cos(np.radians(30))  # x축 거리 계산
                            relative_y = -0.3 * np.sin(np.radians(30))  # y축 거리 계산
                            self.get_logger().info("목표 좌표가 박스의 왼쪽에 있음. 좌측 30도 회전 이동.")
                            self.last_turn = "left"

                        elif target_x > x2:
                            relative_x = 0.3 * np.cos(np.radians(30))  # x축 거리 계산
                            relative_y = 0.3 * np.sin(np.radians(30))  # y축 거리 계산
                            self.get_logger().info("목표 좌표가 박스의 오른쪽에 있음. 우측 이동.")
                            self.last_turn = "right"
                        
                        else:
                            self.get_logger().info("직진")
                        self.send_goal(relative_x, relative_y)
                            
                    else:
                        self.send_goal(0.0, 0.0)
                        if mid_y > target_y:
                            twist.linear.x = 0
                        else:
                            twist.linear.x = abs(target_y - mid_y)
                            
                        if target_x > x2:
                            twist.angular.z = -0.17  # 좌회전 10도
                            self.get_logger().info("목표 좌표가 박스의 왼쪽에 있음. 좌측 이동.")
                            self.last_turn = "left"
                        else:
                            twist.angular.z = 0.17  # 우회전 10도
                            self.get_logger().info("목표 좌표가 박스의 오른쪽에 있음. 우측 이동")
                            self.last_turn = "right"
                        self.publisher.publish(twist)
                    return

    def shutdown_node(self):
        self.get_logger().info('Shutting down node...')
        rclpy.shutdown()

    def send_goal(self, relative_x, relative_y):
        try:
            # 수정: 타임아웃 설정 추가
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), timeout=rclpy.time.Duration(seconds=1.0))
            current_x = transform.transform.translation.x
            current_y = transform.transform.translation.y
            rotation = transform.transform.rotation

            if relative_x == 0 and relative_y == 0:
                goal_x = current_x
                goal_y = current_y
            
            else:
                _, _, yaw = self.quaternion_to_euler(rotation.x, rotation.y, rotation.z, rotation.w)
                self.get_logger().info(f"로봇 현재위치 : {(current_x, current_y)}, yaw : {(yaw)}")
                goal_x = current_x + relative_x * math.cos(yaw) - relative_y * math.sin(yaw)
                goal_y = current_y + relative_x * math.sin(yaw) + relative_y * math.cos(yaw)

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = goal_x
            goal_msg.pose.pose.position.y = goal_y

            # 수정: 액션 서버 대기 시 타임아웃 추가
            if not self._action_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("Nav2 액션 서버 연결 실패")
                return

            self._send_goal_future = self._action_client.send_goal_async(goal_msg)
            self._send_goal_future.add_done_callback(self.goal_response_callback)
            self.get_logger().info(f"Nav2로 목표 전송: x={goal_x}, y={goal_y}")
        except Exception as e:
            self.get_logger().warn(f"Nav2 목표 설정 실패: {e}")

    def goal_response_callback(self, future):
        if future.result().accepted:
            self.get_logger().info("Nav2 목표가 수락되었습니다.")
        else:
            self.get_logger().warn("Nav2 목표가 거부되었습니다.")
    
    def quaternion_to_euler(self, x, y, z, w):
        # Roll (X축 회전)
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        # Pitch (Y축 회전)
        t2 = 2.0 * (w * y - z * x)
        t2 = max(-1.0, min(1.0, t2))  # 값 제한
        pitch = math.asin(t2)

        # Yaw (Z축 회전)
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    node = YOLOTeleopNode()

    try:
        # ROS 2 노드 실행
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl + C로 종료 시 로깅 가능
        node.get_logger().info('Keyboard interrupt, shutting down...')
    except Exception as e:
        # 예외 발생 시 로그 출력 및 종료 처리
        node.get_logger().error(f"Unexpected error: {e}")
    finally:
        # 리소스 정리
        if node:
            node.destroy_node()
        rclpy.shutdown()

        # OpenCV 창 닫기 (창이 열려 있지 않은 경우에도 안전)
        try:
            cv2.destroyAllWindows()
        except Exception as e:
            print(f"Error while closing OpenCV windows: {e}")