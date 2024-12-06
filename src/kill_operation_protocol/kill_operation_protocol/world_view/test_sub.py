import rclpy
from rclpy.node import Node
from swat_interface.msg import TargetStatus, TogetherStatus  # Import custom message types

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        
        # Subscription to TargetStatus
        self.target_subscription = self.create_subscription(
            TargetStatus,
            '/world_view/target_status',  # Topic to subscribe to
            self.target_listener_callback,
            10  # Queue size
        )
        
        # Subscription to TogetherStatus
        self.together_subscription = self.create_subscription(
            TogetherStatus,
            '/world_view/together_status',  # Topic to subscribe to
            self.together_listener_callback,
            10  # Queue size
        )

    def target_listener_callback(self, msg):
        self.get_logger().info(f'Received TargetStatus: {msg.target_status}')

    def together_listener_callback(self, msg):
        self.get_logger().info(f'Received TogetherStatus: {msg.together_status}')


def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)

    simple_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
