import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class ProcessNode(Node):
    def __init__(self):
        super().__init__("process")

        self.signal_subscription = self.create_subscription(
            Float32, "/signal", self.signal_callback, 10
        )
        self.time_subscription = self.create_subscription(
            Float32, "/time", self.time_callback, 10
        )
        self.proc_signal_publisher = self.create_publisher(Float32, "/proc_signal", 10)

        self.received_signal = None
        self.received_time = None

        self.timer = self.create_timer(0.1, self.timer_callback)  # 0.1s = 10Hz
        self.get_logger().info("Process node has started.")

    def signal_callback(self, msg):
        self.received_signal = msg.data

    def time_callback(self, msg):
        self.received_time = msg.data

    def timer_callback(self):
        if self.received_signal is not None and self.received_time is not None:
            proc_signal_msg = Float32()
            proc_signal_msg.data = (
                math.sin(math.asin(self.received_signal) + math.pi) / 2.0 + 0.5
            )

            # Publicar la señal procesada
            self.proc_signal_publisher.publish(proc_signal_msg)

            # Imprimir la señal procesada en la terminal
            self.get_logger().info(
                f"Processed Signal: {proc_signal_msg.data}, Time: {self.received_time}"
            )


def main(args=None):
    rclpy.init(args=args)

    process_node = ProcessNode()

    try:
        rclpy.spin(process_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        process_node.destroy_node()


if __name__ == "__main__":
    main()
