import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class SignalGenerator(Node):
    def __init__(self):
        super().__init__("signal_generator")

        self.signal_publisher = self.create_publisher(Float32, "/signal", 10)
        self.time_publisher = self.create_publisher(Float32, "/time", 10)

        self.timer = self.create_timer(0.1, self.timer_callback)  # 0.1s = 10Hz
        self.get_logger().info("Signal generator node has started.")
        self.time = 0.0

    def timer_callback(self):
        signal_value = math.sin(self.time)

        signal_msg = Float32()
        time_msg = Float32()

        signal_msg.data = signal_value
        time_msg.data = self.time

        # Publish the messages
        self.signal_publisher.publish(signal_msg)
        self.time_publisher.publish(time_msg)

        self.time += 0.1

        # Log the values to the terminal
        self.get_logger().info(f"Time: {self.time}, Signal: {signal_value}")


def main(args=None):
    rclpy.init(args=args)

    signal_generator = SignalGenerator()

    try:
        rclpy.spin(signal_generator)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        signal_generator.destroy_node()


if __name__ == "__main__":
    main()
