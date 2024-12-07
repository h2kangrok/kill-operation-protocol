import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, '/camera/image/compressed', 10)
        self.timer = self.create_timer(0.033, self.timer_callback)  # 30Hz, 0.033초마다 콜백
        self.cap = cv2.VideoCapture(7)  # 웹캠 연결
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)  # 해상도 너비 (320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)  # 해상도 높이 (240)
        self.cap.set(cv2.CAP_PROP_FPS, 30)  # FPS를 30으로 설정하여 속도 최적화

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture frame from camera.")
            return

        # JPEG 품질 설정
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30]  # 품질을 10으로 설정 (속도 우선)
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', frame, encode_param)[1]).tobytes()

        self.publisher_.publish(msg)
        self.get_logger().info("Published image.")



def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('프로그램 종료...')
    finally:
        node.cap.release()
        rclpy.shutdown()

if __name__ == '__main__':
    main()