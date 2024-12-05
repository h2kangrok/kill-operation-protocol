# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# import cv2
# from cv_bridge import CvBridge
# from ultralytics import YOLO
# import numpy as np

# # YOLO 모델 로드 (전역 변수로 선언하여 반복 로드를 방지)
# model = YOLO('src/yolo_model/yolov8TopView_custom_model.pt')

# # 사각형 설정
# x1, y1, x2, y2 = 500, 500, 800, 200
# default_color = (255, 0, 0)  # 기본 색 (파란색)
# target_color = (0, 255, 0)   # 객체가 들어왔을 때 색 (녹색)
# thickness = 2
# # 타겟 객체의 클래스 ID (target_key)
# target_key = 1  # 예시로 클래스 ID 44를 사용

# class ImageSubscriber(Node):
#     def __init__(self):
#         super().__init__('image_subscriber')
#         self.subscription = self.create_subscription(
#             Image,
#             '/image_raw',
#             self.listener_callback,
#             10
#         )
#         self.bridge = CvBridge()

#     def listener_callback(self, msg):
#         try:
#             # Convert ROS image message to OpenCV image
#             frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

#             # 카메라 매트릭스와 왜곡 계수 (제공된 값)
#             self.camera_matrix = np.array([[445.860105, 0.000000, 304.520700],
#                                         [0.000000, 443.733547, 236.349816],
#                                         [0.000000, 0.000000, 1.000000]])

#             self.distortion_coefficients = np.array([0.010617, -0.000807, -0.000064, -0.001558, 0.000000])

#             # 이미지 왜곡 처리 (보정)
#             frame_undistorted = cv2.undistort(frame, self.camera_matrix, self.distortion_coefficients)

#             # Perform YOLO object detection
#             result, annotated_image = detect_objects(frame_undistorted)

#             # Check if any target object is inside the rectangle
#             is_target_inside = False
#             for detection in result[0].boxes:
#                 x, y, w, h = detection.xywh[0].tolist()  # Bounding box center and size
#                 x_min = x - w / 2
#                 y_min = y - h / 2
#                 x_max = x + w / 2
#                 y_max = y + h / 2

#                 # Get the class ID of the detected object
#                 class_id = int(detection.cls[0])  # Assuming class is a tensor or list

#                 # Check if the detected object is the target and overlaps with the rectangle
#                 if class_id == target_key and (
#                     x_min < x2 and x_max > x1 and  # Overlap in X
#                     y_min < y1 and y_max > y2     # Overlap in Y
#                 ):
#                     is_target_inside = True
#                     break

#             # Change rectangle color based on detection
#             color = target_color if is_target_inside else default_color

#             # Draw the user-defined rectangle on the image
#             cv2.rectangle(annotated_image, (x1, y1), (x2, y2), color, thickness)

#             # Display the annotated frame (with both YOLO detection boxes and the rectangle)
#             cv2.imshow("Webcam", annotated_image)

#             # Wait for key press to exit
#             if cv2.waitKey(1) == ord('q'):
#                 rclpy.shutdown()

#         except Exception as e:
#             self.get_logger().error(f'Error: {e}')


# def detect_objects(image):
#     """
#     YOLO 모델로 객체를 탐지하고 결과를 반환.
#     :param image: 입력 이미지
#     :return: 탐지 결과와 시각화된 이미지
#     """
#     result = model(image, verbose=True)
#     annotated_image = result[0].plot()  # YOLO의 탐지 결과가 반영된 이미지
#     return result, annotated_image

# def main():
#     rclpy.init()
#     image_subscriber = ImageSubscriber()

#     try:
#         rclpy.spin(image_subscriber)
#     except KeyboardInterrupt:
#         pass

#     cv2.destroyAllWindows()
#     image_subscriber.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from ultralytics import YOLO

# YOLO 모델 로드 (전역 변수로 선언하여 반복 로드를 방지)
model = YOLO('src/yolo_model/yolov8TopView_custom_model.pt')

# 사각형 설정
x1, y1, x2, y2 = 500, 500, 800, 200
default_color = (255, 0, 0)  # 기본 색 (파란색)
target_color = (0, 255, 0)   # 객체가 들어왔을 때 색 (녹색)
thickness = 2

# 타겟 객체의 클래스 ID (target_key)
target_key = 1  # 예시로 클래스 ID 44를 사용


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.listener_callback,
            10
        )

        # 카메라 매트릭스와 왜곡 계수 (제공된 값)
        self.camera_matrix = np.array([[445.860105, 0.000000, 304.520700],
                                       [0.000000, 443.733547, 236.349816],
                                       [0.000000, 0.000000, 1.000000]])

        self.distortion_coefficients = np.array([0.010617, -0.000807, -0.000064, -0.001558, 0.000000])

    def listener_callback(self, msg):
        try:
            # CompressedImage 메시지 -> OpenCV 이미지로 변환
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # 이미지 왜곡 처리 (보정)
            frame_undistorted = cv2.undistort(frame, self.camera_matrix, self.distortion_coefficients)

            # Perform YOLO object detection
            result, annotated_image = detect_objects(frame_undistorted)

            # Check if any target object is inside the rectangle
            is_target_inside = False
            for detection in result[0].boxes:
                x, y, w, h = detection.xywh[0].tolist()  # Bounding box center and size
                x_min = x - w / 2
                y_min = y - h / 2
                x_max = x + w / 2
                y_max = y + h / 2

                # Get the class ID of the detected object
                class_id = int(detection.cls[0])  # Assuming class is a tensor or list

                # Check if the detected object is the target and overlaps with the rectangle
                if class_id == target_key and (
                    x_min < x2 and x_max > x1 and  # Overlap in X
                    y_min < y1 and y_max > y2     # Overlap in Y
                ):
                    is_target_inside = True
                    break

            # Change rectangle color based on detection
            color = target_color if is_target_inside else default_color

            # Draw the user-defined rectangle on the image
            cv2.rectangle(annotated_image, (x1, y1), (x2, y2), color, thickness)

            # Display the annotated frame (with both YOLO detection boxes and the rectangle)
            cv2.imshow("Webcam", annotated_image)

            # Wait for key press to exit
            if cv2.waitKey(1) == ord('q'):
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
