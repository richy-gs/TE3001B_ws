import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class ArmSimulator(Node):
    def __init__(self):
        super().__init__("arm_simulator")

        # Suscribirse al tópico de torque
        self.subscription = self.create_subscription(
            Float32MultiArray, "/tau", self.torque_callback, 10
        )
        self.subscription  # evitar warning de variable sin usar

        # Publicador del estado articular
        self.publisher = self.create_publisher(Float32MultiArray, "/joint_states", 10)

        # Definir parámetros del sistema
        self.l1 = 0.450  # Longitud del eslabón 1 (m)
        self.l2 = 0.450  # Longitud del eslabón 2 (m)
        self.lc1 = 0.091  # Distancia al centro de masa del eslabón 1 (m)
        self.lc2 = 0.048  # Distancia al centro de masa del eslabón 2 (m)
        self.m1 = 23.902  # Masa del eslabón 1 (kg)
        self.m2 = 3.880  # Masa del eslabón 2 (kg)
        self.I1 = 1.266  # Inercia del eslabón 1 (kg*m^2)
        self.I2 = 0.093  # Inercia del eslabón 2 (kg*m^2)
        self.g = 9.81  # Gravedad (m/s^2)

        # Condiciones iniciales
        self.q = np.array([0.0, 0.0])  # Ángulos iniciales
        self.q_dot = np.array([0.0, 0.0])  # Velocidades iniciales

        # Tiempo de muestreo
        self.dt = 0.01  # 10ms
        self.timer = self.create_timer(self.dt, self.update_system)

        # Torque actual
        self.tau = np.array([0.0, 0.0])

    def torque_callback(self, msg):
        """Recibe el torque de control"""
        if len(msg.data) == 2:
            self.tau = np.array(msg.data)

    def compute_dynamics(self, q, q_dot, tau):
        """Calcula la aceleración de los ángulos articulares usando la ecuación dinámica"""
        M = np.array(
            [
                [
                    self.I1
                    + self.I2
                    + self.m1 * self.lc1**2
                    + self.m2 * (self.l1**2 + self.lc2**2),
                    self.I2 + self.m2 * self.lc2**2,
                ],
                [self.I2 + self.m2 * self.lc2**2, self.I2 + self.m2 * self.lc2**2],
            ]
        )

        C = np.array(
            [
                [
                    -self.m2 * self.l1 * self.lc2 * np.sin(q[1]) * q_dot[1],
                    -self.m2
                    * self.l1
                    * self.lc2
                    * np.sin(q[1])
                    * (q_dot[0] + q_dot[1]),
                ],
                [self.m2 * self.l1 * self.lc2 * np.sin(q[1]) * q_dot[0], 0],
            ]
        )

        G = np.array(
            [
                (
                    self.m1 * self.g * self.lc1 * np.cos(q[0])
                    + self.m2
                    * self.g
                    * (self.l1 * np.cos(q[0]) + self.lc2 * np.cos(q[0] + q[1]))
                ),
                (self.m2 * self.g * self.lc2 * np.cos(q[0] + q[1])),
            ]
        )

        q_ddot = np.linalg.inv(M) @ (tau - C @ q_dot - G)
        return q_ddot

    def update_system(self):
        """Actualiza la dinámica del sistema en cada iteración"""
        q_ddot = self.compute_dynamics(self.q, self.q_dot, self.tau)

        # Integración numérica simple (Euler)
        self.q_dot += q_ddot * self.dt
        self.q += self.q_dot * self.dt

        # Publicar el estado actual del manipulador
        msg = Float32MultiArray()
        msg.data = [self.q[0], self.q[1]]
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    arm_simulator = ArmSimulator()
    rclpy.spin(arm_simulator)
    arm_simulator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
