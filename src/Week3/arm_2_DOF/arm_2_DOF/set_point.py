import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class SetPointPublisher(Node):
    def __init__(self):
        super().__init__("set_point_node")

        # Parámetros para el primer setpoint (q1d)
        self.declare_parameter("signal_type_q1", "step")
        self.signal_type_q1 = (
            self.get_parameter("signal_type_q1").get_parameter_value().string_value
        )

        self.declare_parameter("amplitude_q1", 0.8)
        self.amplitude_q1 = (
            self.get_parameter("amplitude_q1").get_parameter_value().double_value
        )

        self.declare_parameter("omega_q1", 0.5)
        self.omega_q1 = (
            self.get_parameter("omega_q1").get_parameter_value().double_value
        )

        # Parámetros para el segundo setpoint (q2d)
        self.declare_parameter("signal_type_q2", "step")
        self.signal_type_q2 = (
            self.get_parameter("signal_type_q2").get_parameter_value().string_value
        )

        self.declare_parameter("amplitude_q2", 0.5)
        self.amplitude_q2 = (
            self.get_parameter("amplitude_q2").get_parameter_value().double_value
        )

        self.declare_parameter("omega_q2", 0.5)
        self.omega_q2 = (
            self.get_parameter("omega_q2").get_parameter_value().double_value
        )

        self.timer_period = 0.2  # Intervalo de tiempo en segundos

        # Publicador del setpoint
        self.signal_publisher = self.create_publisher(
            Float32MultiArray, "/set_point", 10
        )

        # Timer para actualizar el setpoint
        self.timer = self.create_timer(self.timer_period, self.timer_cb)

        # Variables iniciales
        self.start_time = self.get_clock().now()

        # Callback de parámetros
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.get_logger().info("SetPoint Node Started 🚀")

    def generate_signal(self, signal_type, amplitude, omega, elapsed_time):
        """Genera la señal de acuerdo con el tipo especificado."""
        if signal_type == "sinusoidal":
            return amplitude * np.sin(omega * elapsed_time)
        elif signal_type == "square":
            return amplitude if np.sin(omega * elapsed_time) >= 0 else -amplitude
        elif signal_type == "step":
            return amplitude if elapsed_time >= 1.0 else 0.0
        else:
            self.get_logger().warn(f"Unknown signal type: {signal_type}")
            return 0.0

    def timer_cb(self):
        """Callback periódico para calcular y publicar los setpoints."""
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        q1d = self.generate_signal(
            self.signal_type_q1, self.amplitude_q1, self.omega_q1, elapsed_time
        )
        q2d = self.generate_signal(
            self.signal_type_q2, self.amplitude_q2, self.omega_q2, elapsed_time
        )

        # Publicar el setpoint como un array con [q1d, q2d]
        msg = Float32MultiArray()
        msg.data = [q1d, q2d]
        self.signal_publisher.publish(msg)

    def parameters_callback(self, params):
        """Callback para actualizar los parámetros de forma dinámica."""
        for param in params:
            if param.name == "signal_type_q1" and param.type_ == param.Type.STRING:
                self.signal_type_q1 = param.value
                self.get_logger().info(
                    f"Signal type q1 updated to {self.signal_type_q1}"
                )
            elif param.name == "amplitude_q1" and param.type_ == param.Type.DOUBLE:
                self.amplitude_q1 = param.value
                self.get_logger().info(f"Amplitude q1 updated to {self.amplitude_q1}")
            elif param.name == "omega_q1" and param.type_ == param.Type.DOUBLE:
                self.omega_q1 = param.value
                self.get_logger().info(f"Omega q1 updated to {self.omega_q1}")

            elif param.name == "signal_type_q2" and param.type_ == param.Type.STRING:
                self.signal_type_q2 = param.value
                self.get_logger().info(
                    f"Signal type q2 updated to {self.signal_type_q2}"
                )
            elif param.name == "amplitude_q2" and param.type_ == param.Type.DOUBLE:
                self.amplitude_q2 = param.value
                self.get_logger().info(f"Amplitude q2 updated to {self.amplitude_q2}")
            elif param.name == "omega_q2" and param.type_ == param.Type.DOUBLE:
                self.omega_q2 = param.value
                self.get_logger().info(f"Omega q2 updated to {self.omega_q2}")

        return SetParametersResult(successful=True)


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
