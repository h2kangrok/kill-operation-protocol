import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DispatchReceiver(Node):
    def __init__(self):
        super().__init__('dispatch_receiver')
        self.get_logger().info('Dispatch Receiver Node is running')

        # QoS 설정
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE
        )
        
        # Subscriber 설정
        self.subscription_dispatch = self.create_subscription(
            String,
            'dispatch',
            self.dispatch_callback,
            qos_profile
        )

    def dispatch_callback(self, msg):
        self.get_logger().info(f'Received dispatch message: {msg.data}')
        if msg.data == '1':
            self.handle_dispatch_action()

    def handle_dispatch_action(self):
        self.get_logger().info('Handling dispatch action for message 1')


def main(args=None):
    rclpy.init(args=args)
    
    node = DispatchReceiver()
 
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Dispatch Receiver Node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
