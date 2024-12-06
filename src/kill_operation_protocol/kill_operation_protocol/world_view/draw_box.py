import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from swat_interface.msg import TargetStatus, TogetherStatus  # 새로 만든 메시지 타입
import cv2
import numpy as np
from ultralytics import YOLO

# YOLO 모델 로드
model = YOLO('src/yolo_model/yolov8TopView_custom_model.pt')

# 사각형 설정
x1, y1, x2, y2 = 200, 50, 800, 430
default_color = (255, 0, 0)  # 파란색
target_color = (0, 255, 0)   # 녹색
alert_color = (0, 0, 255)    # 빨간색
thickness = 2

# 타겟 객체의 클래스 KEY값
target_key_1 = 1
target_key_2 = 2

# 투시 변환 설정
width_default, height_default = 1200, 600
fixed_pts_src = [(38, 155), (566, 146), (636, 432), (1, 448)]  # 고정된 좌표
matrix = None


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            'worldview_camera/image/compressed',
            self.listener_callback,
            10
        )
        self.target_publisher = self.create_publisher(TargetStatus, 'world_view/target_status', 10)
        self.together_publisher = self.create_publisher(TogetherStatus, 'world_view/together_status', 10)
        self.setup_perspective_transform()

    def setup_perspective_transform(self):
        global matrix
        if matrix is None:
            pts_src_np = np.array(fixed_pts_src, dtype=np.float32)
            pts_dst = np.array([
                [0, 0],
                [width_default - 1, 0],
                [width_default - 1, height_default - 1],
                [0, height_default - 1]
            ], dtype=np.float32)
            matrix = cv2.getPerspectiveTransform(pts_src_np, pts_dst)
            print("투시 변환 행렬 생성 완료.")

    def listener_callback(self, msg):
        global matrix
        try:
            # CompressedImage 메시지 -> OpenCV 이미지로 변환
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # 투시 변환
            if matrix is not None:
                warped = cv2.warpPerspective(frame, matrix, (width_default, height_default))
                result, annotated_image = detect_objects(warped)

                # 상태 변수 초기화
                is_target_inside_1 = False
                is_target_inside_2 = False

                for detection in result[0].boxes:
                    x, y, w, h = detection.xywh[0].tolist()
                    x_min = x - w / 2
                    y_min = y - h / 2
                    x_max = x + w / 2
                    y_max = y + h / 2

                    class_id = int(detection.cls[0])

                    if class_id == target_key_1 and (x_min < x2 and x_max > x1 and y_min < y2 and y_max > y1):
                        is_target_inside_1 = True

                    if class_id == target_key_2 and (x_min < x2 and x_max > x1 and y_min < y2 and y_max > y1):
                        is_target_inside_2 = True

                # 색상 및 상태 확인
                if is_target_inside_1 and is_target_inside_2:
                    color = target_color
                    self.publish_together_status(True)
                elif is_target_inside_1:
                    color = alert_color
                    self.publish_target_status(True)
                else:
                    color = default_color
                    self.publish_together_status(False)
                    self.publish_target_status(False)

                # 사각형 그리기
                cv2.rectangle(annotated_image, (x1, y1), (x2, y2), color, thickness)

                # 이미지 출력
                cv2.imshow("Warped Feed", annotated_image)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f'Error: {e}')

    def publish_target_status(self, status):
        msg = TargetStatus()
        msg.target_status = status
        self.target_publisher.publish(msg)
        self.get_logger().info(f"TargetStatus Published: {status}")

    def publish_together_status(self, status):
        msg = TogetherStatus()
        msg.together_status = status
        self.together_publisher.publish(msg)
        self.get_logger().info(f"TogetherStatus Published: {status}")


def detect_objects(image):
    result = model(image, verbose=False)
    annotated_image = result[0].plot()
    return result, annotated_image


def main():
    rclpy.init()
    image_subscriber = ImageSubscriber()

    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
