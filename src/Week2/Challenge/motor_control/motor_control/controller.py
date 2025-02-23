import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64


class ControllerNode(Node):
    def __init__(self):
        super().__init__("ctrl_node")

        # Declare parameters with default values
        self.declare_parameter("Kp", 1.0)
        self.declare_parameter("Ki", 0.1)
        self.declare_parameter("Kd", 0.01)
        self.declare_parameter("sampling_time", 0.1)
        self.declare_parameter("rate", 10)  # Rate in Hz

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
        self.motor_input_pub = self.create_publisher(
            Float64, "/motor_input_u", QoSProfile(depth=10)
        )

        # Subscribers for motor output and set point
        self.motor_output_sub = self.create_subscription(
            Float64, "/motor_speed_y", self.motor_output_callback, QoSProfile(depth=10)
        )

        self.set_point_sub = self.create_subscription(
            Float64, "/set_point", self.set_point_callback, QoSProfile(depth=10)
        )

        # Variable to store the set point and system output
        self.set_point = 0.0
        self.motor_output = 0.0

        # Timer to control the sampling rate
        self.create_timer(1.0 / self.rate, self.timer_callback)

    def motor_output_callback(self, msg):
        """Callback for motor system output."""
        self.motor_output = msg.data

    def set_point_callback(self, msg):
        """Callback for set point."""
        self.set_point = msg.data

    def timer_callback(self):
        """Called periodically to compute and publish the control input."""
        error = self.set_point - self.motor_output
        self.integral += error * self.sampling_time
        derivative = (error - self.prev_error) / self.sampling_time

        # Compute PID control signal
        control_signal = (
            self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        )

        # Publish the control input
        control_msg = Float64()
        control_msg.data = control_signal
        self.motor_input_pub.publish(control_msg)

        # Update the previous error
        self.prev_error = error

    # Cambiar en base a lo que se vio en la sesion
    def update_parameters(self):
        """Update the parameters dynamically."""
        try:
            # Read new parameters at runtime
            new_Kp = self.get_parameter("Kp").get_parameter_value().double_value
            new_Ki = self.get_parameter("Ki").get_parameter_value().double_value
            new_Kd = self.get_parameter("Kd").get_parameter_value().double_value

            # Validate the new parameters
            if new_Kp < 0 or new_Ki < 0 or new_Kd < 0:
                self.get_logger().error("Invalid parameters: Gains cannot be negative!")
                return False

            # Update parameters
            self.Kp = new_Kp
            self.Ki = new_Ki
            self.Kd = new_Kd

            self.get_logger().info(
                f"Parameters updated: Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd}"
            )
            return True
        except Exception as e:
            self.get_logger().error(f"Error updating parameters: {e}")
            return False


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
