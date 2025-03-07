import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class ControllerNode(Node):
    def __init__(self):
        super().__init__("ctrl_node")

        # Declarar parámetros para cada eslabón
        self.declare_parameter("Kp_q1", 160.0)
        self.declare_parameter("Ki_q1", 0.3)
        self.declare_parameter("Kd_q1", 45.0)

        self.declare_parameter("Kp_q2", 10.55)
        self.declare_parameter("Ki_q2", 0.002)
        self.declare_parameter("Kd_q2", 1.0)

        self.declare_parameter("sampling_time", 0.05)
        self.declare_parameter("rate", 50)  # Frecuencia en Hz

        # Obtener valores de los parámetros
        self.Kp = [
            self.get_parameter("Kp_q1").get_parameter_value().double_value,
            self.get_parameter("Kp_q2").get_parameter_value().double_value,
        ]
        self.Ki = [
            self.get_parameter("Ki_q1").get_parameter_value().double_value,
            self.get_parameter("Ki_q2").get_parameter_value().double_value,
        ]
        self.Kd = [
            self.get_parameter("Kd_q1").get_parameter_value().double_value,
            self.get_parameter("Kd_q2").get_parameter_value().double_value,
        ]
        self.sampling_time = (
            self.get_parameter("sampling_time").get_parameter_value().double_value
        )
        self.rate = self.get_parameter("rate").get_parameter_value().integer_value

        # Variables del PID para cada eslabón
        self.prev_error = [0.0, 0.0]
        self.integral = [0.0, 0.0]

        # Publicador para el torque de control
        self.torque_pub = self.create_publisher(Float32MultiArray, "/tau", 10)

        # Suscriptores para el estado articular y referencia
        self.joint_state_sub = self.create_subscription(
            Float32MultiArray, "/joint_states", self.joint_state_callback, 10
        )
        self.set_point_sub = self.create_subscription(
            Float32MultiArray, "/set_point", self.set_point_callback, 10
        )

        # Variables para almacenar referencia y estados actuales
        self.q = [0.0, 0.0]  # Ángulos actuales
        self.qd = [0.8, 0.5]  # Referencia

        # Timer para la actualización del control
        self.create_timer(1.0 / self.rate, self.timer_callback)

        # Callback de parámetros dinámicos
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.get_logger().info("Controller Node Started 🚀")

    def joint_state_callback(self, msg):
        """Callback para recibir los ángulos actuales del brazo."""
        if len(msg.data) == 2:
            self.q = list(msg.data)

    def set_point_callback(self, msg):
        """Callback para recibir la referencia deseada."""
        if len(msg.data) == 2:
            self.qd = list(msg.data)

    def timer_callback(self):
        """Calcula la señal de control y publica el torque."""
        error = [self.qd[i] - self.q[i] for i in range(2)]
        self.integral = [
            self.integral[i] + error[i] * self.sampling_time for i in range(2)
        ]
        derivative = [
            (error[i] - self.prev_error[i]) / self.sampling_time for i in range(2)
        ]

        # Calcular la señal de control PID con los parámetros de cada eslabón
        tau = [
            self.Kp[i] * error[i]
            + self.Ki[i] * self.integral[i]
            + self.Kd[i] * derivative[i]
            for i in range(2)
        ]

        # Publicar el torque
        msg = Float32MultiArray()
        msg.data = tau
        self.torque_pub.publish(msg)

        # Actualizar error anterior
        self.prev_error = error

    def parameters_callback(self, params):
        """Callback para actualizar los parámetros PID en tiempo real."""
        for param in params:
            if param.name in ["Kp_q1", "Ki_q1", "Kd_q1", "Kp_q2", "Ki_q2", "Kd_q2"]:
                idx = 0 if "q1" in param.name else 1
                if param.value < 0.0:
                    self.get_logger().warn(
                        f"Invalid {param.name} value! It cannot be negative."
                    )
                    return SetParametersResult(
                        successful=False, reason=f"{param.name} cannot be negative"
                    )
                else:
                    if "Kp" in param.name:
                        self.Kp[idx] = param.value
                    elif "Ki" in param.name:
                        self.Ki[idx] = param.value
                    elif "Kd" in param.name:
                        self.Kd[idx] = param.value
                    self.get_logger().info(f"{param.name} updated to {param.value}")

        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    controller_node = ControllerNode()

    try:
        rclpy.spin(controller_node)
    except KeyboardInterrupt:
        pass
    finally:
        controller_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
