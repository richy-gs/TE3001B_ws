import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubcriber(Node):
    def __init__(self):
        super().__init__("listener_node")
        self.subscription = self.create_subscription(
            String, "chatter", self.listener_cb, 10
        )
        self.subscription  # No unusedvariable warning

    def listener_cb(self, msg):
        self.get_logger().info('I heard "%s"' % msg.data)


# Main
def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubcriber()

    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        minimal_subscriber.destroy_node()


# Execute Node
if __name__ == "__main__":
    main()
