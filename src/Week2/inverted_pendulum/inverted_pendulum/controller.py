import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Float32


class ControllerNode(Node):
    def __init__(self):
        super().__init__("ctrl_node")

        # Declare parameters with default values
        self.declare_parameter("Kp", 7.5)
        self.declare_parameter("Ki", 7.0)
        self.declare_parameter("Kd", 0.01)
        self.declare_parameter("sampling_time", 0.01)
        self.declare_parameter("rate", 160)  # Rate in Hz

        self.Kp = self.get_parameter("Kp").get_parameter_value().double_value
        self.Ki = self.get_parameter("Ki").get_parameter_value().double_value
        self.Kd = self.get_parameter("Kd").get_parameter_value().double_value
        self.sampling_time = (
            self.get_parameter("sampling_time").get_parameter_value().double_value
        )
        self.rate = self.get_parameter("rate").get_parameter_value().integer_value

        # Variables for the PID controller
        self.prev_error = 0
        self.integral = 0

        # Publisher for motor input control signal
        self.inv_pendulum_input_pub = self.create_publisher(Float32, "input_u", 10)

        # Subscribers for motor output and set point
        self.inv_pendulum_output_sub = self.create_subscription(
            Float32, "output_y", self.inv_pendulum_output_callback, 10
        )

        self.set_point_sub = self.create_subscription(
            Float32, "set_point", self.set_point_callback, 10
        )

        # Variable to store the set point and system output
        self.set_point = 0.0
        self.inv_pendulum_output = 0.0

        # Timer to control the sampling rate
        self.create_timer(1.0 / self.rate, self.timer_callback)

        # Parameter Callback
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Node Started
        self.get_logger().info("Controller Node Started \U0001f680")

    def inv_pendulum_output_callback(self, msg):
        """Callback for motor system output."""
        self.inv_pendulum_output = msg.data

    def set_point_callback(self, msg):
        """Callback for set point."""
        self.set_point = msg.data

    def timer_callback(self):
        """Called periodically to compute and publish the control input."""
        error = self.set_point - self.inv_pendulum_output
        self.integral += error * self.sampling_time
        derivative = (error - self.prev_error) / self.sampling_time

        # Compute PID control signal
        control_signal = (
            self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        )

        # Publish the control input
        control_msg = Float32()
        control_msg.data = control_signal
        self.inv_pendulum_input_pub.publish(control_msg)

        # Update the previous error
        self.prev_error = error

    def parameters_callback(self, params):
        for param in params:
            # system gain parameter check
            if param.name == "Kp":
                # check if it is negative
                if param.value < 0.0:
                    self.get_logger().warn("Invalid Kp value! It cannot be negative.")
                    return SetParametersResult(
                        successful=False, reason="Kp value cannot be negative"
                    )
                else:
                    self.Kp = param.value  # Update internal variable
                    self.get_logger().info(f"Kp value updated to {self.Kp}")

            # system gain parameter check
            if param.name == "Ki":
                # check if it is negative
                if param.value < 0.0:
                    self.get_logger().warn("Invalid Ki value! It cannot be negative.")
                    return SetParametersResult(
                        successful=False, reason="Ki value cannot be negative"
                    )
                else:
                    self.Ki = param.value  # Update internal variable
                    self.get_logger().info(f"Ki value updated to {self.Ki}")

                    # system gain parameter check
            if param.name == "Kd":
                # check if it is negative
                if param.value < 0.0:
                    self.get_logger().warn("Invalid Kd value! It cannot be negative.")
                    return SetParametersResult(
                        successful=False, reason="Kd value cannot be negative"
                    )
                else:
                    self.Kd = param.value  # Update internal variable
                    self.get_logger().info(f"Kd value updated to {self.Kd}")

        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    controller_node = ControllerNode()

    try:
        # Spin the node to handle callbacks
        rclpy.spin(controller_node)
    except KeyboardInterrupt:
        pass

    finally:
        controller_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
