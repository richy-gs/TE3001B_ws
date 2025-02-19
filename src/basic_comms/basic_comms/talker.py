# Import your libraries here
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    # Write your code here
    def __init__(self):
        super().__init__("talker_node")
        self.publisher = self.create_publisher(String, "chatter", 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_cb)
        self.i = 0

    # Timer callback
    def timer_cb(self):
        msg = String()
        msg.data = "Hello world: %d" % self.i
        self.publisher.publish(msg)
        self.get_logger().info('Publising: "%s"' % msg.data)
        self.i += 1


# main
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        minimal_publisher.destroy_node()


# Execute Node
if __name__ == "__main__":
    main()
