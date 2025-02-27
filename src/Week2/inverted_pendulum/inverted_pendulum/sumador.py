import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class SumadorNodo(Node):
    def __init__(self):
        super().__init__("sumador_nodo")
        self.suscripcion_carro = self.create_subscription(
            Float32, "carro/input_u", self.callback_carro, 10
        )
        self.suscripcion_angulo = self.create_subscription(
            Float32, "angle/input_u", self.callback_angulo, 10
        )
        self.publicador = self.create_publisher(Float32, "input_u", 10)
        self.valor_carro = 0.0
        self.valor_angulo = 0.0

    def callback_carro(self, msg):
        self.valor_carro = msg.data
        self.publicar_suma()

    def callback_angulo(self, msg):
        self.valor_angulo = msg.data
        self.publicar_suma()

    def publicar_suma(self):
        suma = self.valor_carro + self.valor_angulo
        msg = Float32()
        msg.data = suma
        self.publicador.publish(msg)
        self.get_logger().info(f"Publicando suma: {suma}")


def main(args=None):
    rclpy.init(args=args)
    node = SumadorNodo()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
