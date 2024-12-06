import time
from datetime import datetime
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
import random

class LogPublisher(Node):
    def __init__(self):
        super().__init__('log_publisher')
        self.log_publisher = self.create_publisher(String, 'LOG', 10)
        self.timer = self.create_timer(2.0, self.publish_log)  # 2초마다 로그를 발행
        self.get_logger().info('Log Publisher Node has started')

    def publish_log(self):
        # 현재 시간과 간단한 로그 메시지를 만들어 발행
        current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        # '트래킹 시작!' 로그를 랜덤하게 발행
        if random.random() < 0.25:  # 30% 확률로 '트래킹 시작!' 로그 발행
            log_message = f"트래킹 시작! at {current_time}"
        else:
            log_message = f"Log entry at {current_time}: System running smoothly."
        
        msg = String()
        msg.data = log_message
        self.log_publisher.publish(msg)
        self.get_logger().info(f'Published log: {log_message}')

def main(args=None):
    rclpy.init(args=args)
    log_publisher = LogPublisher()
    try:
        rclpy.spin(log_publisher)
    except KeyboardInterrupt:
        log_publisher.get_logger().info('Shutting down Log Publisher Node...')
    finally:
        log_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
