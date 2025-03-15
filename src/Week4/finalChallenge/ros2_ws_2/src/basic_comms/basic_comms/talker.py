import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init_(self): 
        super().__init__('listener_node')
        self.publisher = self.create_publisher(String, 'chatter', 10)

        self.timer = self.create_timer(0.5, self.timer_cb)
    def timer_cb(self):
        msg = String()
        msg.data = 'Hello World'
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = MinimalSubscriber()

    rclpy.spin(node)


if __name__ == '__main__':
    main()
