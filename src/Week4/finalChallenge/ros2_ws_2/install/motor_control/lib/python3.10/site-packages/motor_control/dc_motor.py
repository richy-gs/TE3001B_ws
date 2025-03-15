# Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult


#Class Definition
class DCMotor(Node):
    def __init__(self):
        super().__init__('dc_motor')

        # Declare parameters
        # System sample time in seconds
        self.declare_parameter('sample_time', 0.018)
        self.declare_parameter('armature_inductance_La', 28.44e-3)
        self.declare_parameter('armature_resistance_Ra', 1.27)
        self.declare_parameter('motor_const_Ka', 0.35)
        self.declare_parameter('back_emf_const_Kb', 0.35)
        self.declare_parameter('motor_inertia_Jm', 0.007)
        self.declare_parameter('motor_friction_b', 0.00173)
        self.declare_parameter('load_torque_tau_c', 0.0)
        

        # DC Motor Parameters
        self.sample_time = self.get_parameter('sample_time').value
        self.La = self.get_parameter('armature_inductance_La').value
        self.Ra = self.get_parameter('armature_resistance_Ra').value
        self.Ka = self.get_parameter('motor_const_Ka').value
        self.Kb = self.get_parameter('back_emf_const_Kb').value
        self.Jm = self.get_parameter('motor_inertia_Jm').value
        self.b  = self.get_parameter('motor_friction_b').value
        self.tau_c = self.get_parameter('load_torque_tau_c').value

        
        
        #Set the messages
        self.motor_output_msg = Float32()
        self.motor_acc_msg = Float32()
        self.i_arm_msg = Float32()

        #Set variables to be used
        self.w = 0.0          # Velocidad angular [rad/s]
        self.i_arm = 0.0    # Corriente de armadura [A]
        self.input_u = 0.0  # [V]
    
        #Declare publishers, subscribers and timers
        self.motor_input_sub = self.create_subscription(Float32, 'motor_input_u', self.input_callback, 10) 
        # Publicador de la velocidad angular
        self.motor_speed_pub = self.create_publisher(Float32, 'motor_speed_y', 10)
        self.motor_acc_pub = self.create_publisher(Float32, 'motor_acc_y', 10)
        self.i_arm_pub = self.create_publisher(Float32, 'motor_i_arm', 10)
        # Timer para integración periódica del modelo
        self.timer = self.create_timer(self.sample_time, self.timer_cb)

        #Parameter Callback
        self.add_on_set_parameters_callback(self.parameters_callback)


        #Node Started
        self.get_logger().info('Dynamical System Node Started \U0001F680')   
        
    #Timer Callback
    def timer_cb(self):         
        #DC Motor Simulation
        #DC Motor Equation 𝑦[𝑘+1] = 𝑦[𝑘] + ((−1/𝜏) 𝑦[𝑘] + (𝐾/𝜏) 𝑢[𝑘]) 𝑇_𝑠
        #self.output_y += (-1.0/self.param_T * self.output_y + self.param_K/self.param_T * self.input_u) * self.sample_time 

        # dot_w    = (Ka * i_arm - b*w - tau_c) / Jm
        # dot_iArm = (1/La) * [u - Ra*i_arm - Kb*w]

        dot_w = (self.Ka * self.i_arm - self.b * self.w - self.tau_c) / self.Jm
        dot_i_arm = (self.input_u - self.Ra * self.i_arm - self.Kb * self.w) / self.La

        self.w     = self.w     + dot_w     * self.sample_time
        self.i_arm = self.i_arm + dot_i_arm * self.sample_time

        #Publish the result
        self.motor_output_msg.data = float(self.w)
        self.motor_speed_pub.publish(self.motor_output_msg)

        self.motor_acc_msg.data = float(dot_w)
        self.motor_acc_pub.publish(self.motor_acc_msg)

        self.i_arm_msg.data = float(self.i_arm)
        self.i_arm_pub.publish(self.i_arm_msg)

    #Subscriber Callback
    def input_callback(self, input_sgn):
        self.input_u = input_sgn.data

    def parameters_callback(self, params):
        for param in params:
            if param.name == "armature_inductance_La":
                if param.value > 0:
                    self.La = param.value
                    self.get_logger().info(f"Updated La: {self.La}")
                else:
                    return SetParametersResult(successful=False, reason="La must be > 0")

            elif param.name == "armature_resistance_Ra":
                if param.value > 0:
                    self.Ra = param.value
                    self.get_logger().info(f"Updated Ra: {self.Ra}")
                else:
                    return SetParametersResult(successful=False, reason="Ra must be > 0")

            elif param.name == "motor_const_Ka":
                if param.value >= 0:
                    self.Ka = param.value
                    self.get_logger().info(f"Updated Ka: {self.Ka}")
                else:
                    return SetParametersResult(successful=False, reason="Ka must be >= 0")

            elif param.name == "back_emf_const_Kb":
                if param.value >= 0:
                    self.Kb = param.value
                    self.get_logger().info(f"Updated Kb: {self.Kb}")
                else:
                    return SetParametersResult(successful=False, reason="Kb must be >= 0")

            elif param.name == "motor_inertia_Jm":
                if param.value > 0:
                    self.Jm = param.value
                    self.get_logger().info(f"Updated Jm: {self.Jm}")
                else:
                    return SetParametersResult(successful=False, reason="Jm must be > 0")

            elif param.name == "motor_friction_b":
                if param.value >= 0:
                    self.b = param.value
                    self.get_logger().info(f"Updated b: {self.b}")
                else:
                    return SetParametersResult(successful=False, reason="b must be >= 0")

            elif param.name == "load_torque_tau_c":
                self.tau_c = param.value
                self.get_logger().info(f"Updated tau_c: {self.tau_c}")

            elif param.name == "sample_time":
                if param.value > 0:
                    self.sample_time = param.value
                    self.get_logger().info(f"Updated sample_time: {self.sample_time}")
                else:
                    return SetParametersResult(successful=False, reason="sample_time must be > 0")

        return SetParametersResult(successful=True)


#Main
def main(args=None):
    rclpy.init(args=args)

    node = DCMotor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

#Execute Node
if __name__ == '__main__':
    main()
