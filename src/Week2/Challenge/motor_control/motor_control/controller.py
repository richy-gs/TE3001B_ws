import rclpy
from custom_interfaces.srv import SetProcessBool
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Float32


class ControllerNode(Node):
    def __init__(self):
        super().__init__("ctrl_node")

        # Variable of control for Server
        self.simulation_running = False

        # Declare parameters with default values
        self.declare_parameter("Kp", 7.5)
        self.declare_parameter("Ki", 7.0)
        self.declare_parameter("Kd", 0.01)
        self.declare_parameter("sampling_time", 0.05)
        self.declare_parameter("rate", 150)  # Rate in Hz

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
        self.motor_input_pub = self.create_publisher(Float32, "motor_input_u", 10)

        # Subscribers for motor output and set point
        self.motor_output_sub = self.create_subscription(
            Float32, "motor_speed_y", self.motor_output_callback, 10
        )

        self.set_point_sub = self.create_subscription(
            Float32, "set_point", self.set_point_callback, 10
        )

        # Set Server callback
        self.srv = self.create_service(
            SetProcessBool, "EnableProcess_ctrl", self.simulation_service_callback
        )

        # Variable to store the set point and system output
        self.set_point = 0.0
        self.motor_output = 0.0

        # Timer to control the sampling rate
        self.create_timer(1.0 / self.rate, self.timer_callback)

        # Parameter Callback
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Node Started
        self.get_logger().info("Controller Node Started \U0001f680")

    def motor_output_callback(self, msg):
        """Callback for motor system output."""
        self.motor_output = msg.data

    def set_point_callback(self, msg):
        """Callback for set point."""
        self.set_point = msg.data

    def timer_callback(self):
        # Stop processing if simulation is not running
        if not self.simulation_running:
            return

        """Called periodically to compute and publish the control input."""
        error = self.set_point - self.motor_output
        self.integral += error * self.sampling_time
        derivative = (error - self.prev_error) / self.sampling_time

        # Compute PID control signal
        control_signal = (
            self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        )

        # Publish the control input
        control_msg = Float32()
        control_msg.data = control_signal
        self.motor_input_pub.publish(control_msg)

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

    def simulation_service_callback(self, request, response):
        if request.enable:
            self.simulation_running = True
            self.get_logger().info("\U0001f680 Simulation Started")
            response.success = True
            response.message = "Simulation Started Successfully"
        else:
            self.simulation_running = False
            self.get_logger().info("\U0001f534 Simulation Stopped")
            response.success = True
            response.message = "Simulation Stopped Successfully"

        return response


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
