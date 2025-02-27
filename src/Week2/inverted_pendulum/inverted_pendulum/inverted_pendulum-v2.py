import numpy as np
import rclpy
from rclpy.node import Node
from scipy.integrate import solve_ivp
from std_msgs.msg import Float32


class InvertedPendulumNode(Node):
    def __init__(self):
        super().__init__("inverted_pendulum")

        # Parámetros del sistema
        self.m = 0.1  # Masa del péndulo (kg)
        self.M = 1.5  # Masa del carrito (kg)
        self.l = 0.4  # Longitud del péndulo (m)
        self.g = 9.81  # Gravedad (m/s^2)

        # Estado inicial [x, x_dot, theta, theta_dot]
        self.state = np.array([0.0, 0.0, 0.1, 0.0])

        # Suscribirse al tópico de entrada de control
        self.subscription = self.create_subscription(
            Float32, "input_u", self.control_callback, 10
        )

        # Publicadores de la posición del carrito y el ángulo del péndulo
        self.car_pos_pub = self.create_publisher(Float32, "carro/output_y", 10)
        self.angle_pos_pub = self.create_publisher(Float32, "angle/output_y", 10)

        # Crear un timer para la integración del sistema dinámico
        self.dt = 0.01  # Paso de tiempo de integración (10ms)
        self.timer = self.create_timer(self.dt, self.update_system)

        # Variable de control inicial
        self.u = 0.0

    def control_callback(self, msg):
        """Callback para recibir el control del motor."""
        self.u = msg.data

    def dynamics(self, t, state):
        """Ecuaciones de movimiento del péndulo invertido."""
        x, x_dot, theta, theta_dot = state
        u = self.u  # Entrada de control

        # Ecuaciones de movimiento
        x_ddot = (1 / self.M) * (u - self.m * self.g * theta)
        theta_ddot = (u - (self.m + self.M) * self.g * theta) / (self.M * self.l)

        return [x_dot, x_ddot, theta_dot, theta_ddot]

    def update_system(self):
        """Integrar las ecuaciones del sistema y actualizar el estado."""
        t_span = (0, self.dt)
        sol = solve_ivp(self.dynamics, t_span, self.state, method="RK45")
        self.state = sol.y[:, -1]  # Tomar el último valor de la solución

        # Publicar las posiciones
        car_pos_msg = Float32()
        car_pos_msg.data = self.state[0]  # Posición del carrito (x)
        self.car_pos_pub.publish(car_pos_msg)

        angle_pos_msg = Float32()
        angle_pos_msg.data = self.state[2]  # Ángulo del péndulo (theta)
        self.angle_pos_pub.publish(angle_pos_msg)

        self.get_logger().info(
            f"Car position: {self.state[0]:.3f}, Angle: {self.state[2]:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = InvertedPendulumNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
