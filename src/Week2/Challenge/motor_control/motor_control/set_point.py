import numpy as np
import rclpy
from custom_interfaces.srv import SetProcessBool
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Float32


class SetPointPublisher(Node):
    def __init__(self):
        super().__init__("set_point_node")

        # Variable of control for Client Service
        self.system_running_cli1 = False
        self.system_running_cli2 = False

        # Declare parameters with default values
        self.declare_parameter("signal_type", "sinusoidal")
        self.signal_type = (
            self.get_parameter("signal_type").get_parameter_value().string_value
        )

        self.declare_parameter("amplitude", 2.0)
        self.amplitude = (
            self.get_parameter("amplitude").get_parameter_value().double_value
        )

        self.declare_parameter("omega", 1.0)
        self.omega = self.get_parameter("omega").get_parameter_value().double_value

        self.timer_period = 0.1  # seconds

        # Create a publisher and timer for the signal
        self.signal_publisher = self.create_publisher(Float32, "set_point", 10)
        self.timer = self.create_timer(self.timer_period, self.timer_cb)

        # Create a service client  for /EnableProcess
        self.cli1 = self.create_client(SetProcessBool, "EnableProcess_ctrl")
        while not self.cli1.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service Controller not available, waiting again...")

        self.cli2 = self.create_client(SetProcessBool, "EnableProcess_dc_motor")
        while not self.cli2.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service DC Motor not available, waiting again...")

        # Create a message and variables to be used
        self.signal_msg = Float32()
        self.start_time = self.get_clock().now()

        # Call for service
        self.send_request_cli1(True)
        self.send_request_cli2(True)

        # Parameter callback
        self.add_on_set_parameters_callback(self.parameters_callback)
        self.get_logger().info("SetPoint Node Started 🚀")

    def timer_cb(self):
        # Stop processing if simulation is not running
        if not self.system_running_cli1 or not self.system_running_cli2:
            return

        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        if self.signal_type == "sinusoidal":
            self.signal_msg.data = self.amplitude * np.sin(self.omega * elapsed_time)
        elif self.signal_type == "square":
            self.signal_msg.data = (
                self.amplitude
                if np.sin(self.omega * elapsed_time) >= 0
                else -self.amplitude
            )
        elif self.signal_type == "step":
            self.signal_msg.data = self.amplitude if elapsed_time >= 1.0 else 0.0
        else:
            self.get_logger().warn(f"Unknown signal type: {self.signal_type}")
            return

        self.signal_publisher.publish(self.signal_msg)

    def parameters_callback(self, params):
        for param in params:
            if param.name == "signal_type" and param.type_ == param.Type.STRING:
                self.signal_type = param.value
                self.get_logger().info(f"Signal type updated to {self.signal_type}")
            elif param.name == "amplitude" and param.type_ == param.Type.DOUBLE:
                self.amplitude = param.value
                self.get_logger().info(f"Amplitude updated to {self.amplitude}")
            elif param.name == "omega" and param.type_ == param.Type.DOUBLE:
                self.omega = param.value
                self.get_logger().info(f"Omega updated to {self.omega}")

        return SetParametersResult(successful=True)

    def send_request_cli1(self, enable: bool):
        request = SetProcessBool.Request()
        request.enable = enable

        # Send a request to start or stop the simulation
        future = self.cli1.call_async(request)
        future.add_done_callback(self.response_callback_cli1)

    def response_callback_cli1(self, future):
        try:
            response = future.result()
            if response.success:
                self.system_running_cli1 = True
                self.get_logger().info(f"Success: {response.message}")
            else:
                self.system_running_cli1 = False
                self.get_logger().warn(f"Failure: {response.message}")
        except Exception as e:
            self.system_running_cli1 = False
            self.get_logger().error(f"Service call failed: {e}")

    def send_request_cli2(self, enable: bool):
        request = SetProcessBool.Request()
        request.enable = enable

        # Send a request to start or stop the simulation
        future = self.cli2.call_async(request)
        future.add_done_callback(self.response_callback_cli2)

    def response_callback_cli2(self, future):
        try:
            response = future.result()
            if response.success:
                self.system_running_cli2 = True
                self.get_logger().info(f"Success: {response.message}")
            else:
                self.system_running_cli2 = False
                self.get_logger().warn(f"Failure: {response.message}")
        except Exception as e:
            self.system_running_cli2 = False
            self.get_logger().error(f"Service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    set_point = SetPointPublisher()
    try:
        rclpy.spin(set_point)
    except KeyboardInterrupt:
        pass
    finally:
        set_point.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
