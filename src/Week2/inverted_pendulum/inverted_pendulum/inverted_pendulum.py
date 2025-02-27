# python imports

# ros imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class Dynamics(Node):
    def __init__(self):
        super().__init__("inv_pendulum_dynamics")

        # constants
        self.g = 9.8

        # params
        self.declare_parameter("pendulum_mass", 0.1)
        self.pendulum_mass = self.get_parameter("pendulum_mass").value

        self.declare_parameter("car_mass", 1.5)
        self.car_mass = self.get_parameter("car_mass").value

        self.declare_parameter("theta", 0.1)
        self.theta = self.get_parameter("theta").value

        self.declare_parameter("axis_length", 0.4)
        self.axis_length = self.get_parameter("axis_length").value

        self.declare_parameter("x_c", 0)
        self.desired_x_c = self.get_parameter("x_c").value

        # publishers
        self.xc_pub = self.create_publisher(Float32, "carro/output_y", 10)
        self.theta_pub = self.create_publisher(Float32, "angle/output_y", 10)

        # subscribers
        self.subscription = self.create_subscription(
            Float32, "input_u", self.simulate, 10
        )

        # timers
        self.dt = 1 / 100
        self.create_timer(self.dt, self.euler_step)

        # state
        self.theta_dot_dot = 0.0
        self.theta_dot = 0.0
        # self.theta = 0.0

        self.x_c_dot_dot = 0.0
        self.x_c_dot = 0.0
        self.x_c = 0.0

    def simulate(self, u):
        l = self.axis_length
        m = self.pendulum_mass
        M = self.car_mass
        g = self.g
        theta = self.theta
        u = u.data

        self.x_c_dot_dot = 1 / M * (u - m * g * theta)
        self.theta_dot_dot = (u - (m + M) * g * theta) / (M * l)

        # self.get_logger().info(f"x_c_dot_dot [{self.x_c_dot_dot}]")

    def euler_step(self):
        # euler integration for position
        self.x_c_dot += self.x_c_dot_dot * self.dt
        self.x_c += self.x_c_dot * self.dt

        # euler integration for angle
        self.theta_dot += self.theta_dot_dot * self.dt
        self.theta += self.theta_dot * self.dt

        # publish position
        xc_msg = Float32()
        xc_msg.data = self.x_c
        self.xc_pub.publish(xc_msg)

        # publish angle
        theta_msg = Float32()
        theta_msg.data = self.theta
        self.theta_pub.publish(theta_msg)

        # log to terminal
        # self.get_logger().info(f"x_c [{xc_msg.data}]")
        # self.get_logger().info(f"theta [{theta_msg.data}]")


def main(args=None):
    rclpy.init(args=args)
    dn = Dynamics()

    try:
        rclpy.spin(dn)
    except KeyboardInterrupt:
        pass
    finally:
        dn.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
