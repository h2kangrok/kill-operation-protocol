import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from ultralytics import YOLO

# YOLO 모델 로드 (전역 변수로 선언하여 반복 로드를 방지)
model = YOLO('src/yolo_model/yolov8TopView_custom_model.pt')

# 사각형 설정
x1, y1, x2, y2 = 200, 50, 800, 430
default_color = (255, 0, 0)  # 기본 색 (파란색)
target_color = (0, 255, 0)   # 객체가 들어왔을 때 색 (녹색)
thickness = 2

# 타겟 객체의 클래스 KEY값 (target_key)
target_key_1 = 1  # 예시로 'Target_Car'의 KEY값을 1로 사용
target_key_2 = 2  # 예시로 'Turtlebot3'의 KEY값을 2로 사용

# 투시 변환 설정
width_default, height_default = 1200, 600
fixed_pts_src = [(38, 155), (566, 146), (636, 432), (1, 448)]  # 고정된 좌표
matrix = None


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            'camera/image/compressed',
            self.listener_callback,
            10
        )
        self.setup_perspective_transform()  # 초기 투시 변환 설정

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

            # Apply perspective transformation if matrix is available
            if matrix is not None:
                warped = cv2.warpPerspective(frame, matrix, (width_default, height_default))
                result, annotated_image = detect_objects(warped)

                # 두 객체가 사각형 안에 있는지 확인
                is_target_inside_1 = False
                is_target_inside_2 = False
                for detection in result[0].boxes:
                    x, y, w, h = detection.xywh[0].tolist()  # Bounding box center and size
                    x_min = x - w / 2  # 좌측 경계
                    y_min = y - h / 2  # 상단 경계
                    x_max = x + w / 2  # 우측 경계
                    y_max = y + h / 2  # 하단 경계

                    # 클래스 ID 얻기
                    class_id = int(detection.cls[0])  # 클래스 ID (tensor 또는 list로 가정)

                    # 탐지된 객체가 타겟이고 사각형과 겹치는지 확인
                    if class_id == target_key_1 and (
                        x_min < x2 and x_max > x1 and  # X축 겹침
                        y_min < y2 and y_max > y1     # Y축 겹침
                    ):
                        is_target_inside_1 = True
                    
                    if class_id == target_key_2 and (
                        x_min < x2 and x_max > x1 and  # X축 겹침
                        y_min < y2 and y_max > y1     # Y축 겹침
                    ):
                        is_target_inside_2 = True

                # 두 객체가 모두 사각형 안에 있으면 색상을 녹색으로 변경
                color = target_color if is_target_inside_1 and is_target_inside_2 else default_color

                # 변환된 이미지에 사각형 그리기
                cv2.rectangle(annotated_image, (x1, y1), (x2, y2), color, thickness)

                # 변환된 이미지와 함께 결과를 출력
                cv2.imshow("Warped Feed", annotated_image)

            # 'q'가 눌리면 프로그램 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f'Error: {e}')


def detect_objects(image):
    """
    YOLO 모델로 객체를 탐지하고 결과를 반환.
    :param image: 입력 이미지
    :return: 탐지 결과와 시각화된 이미지
    """
    result = model(image, verbose=False)
    print(result)
    annotated_image = result[0].plot()  # YOLO의 탐지 결과가 반영된 이미지
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
