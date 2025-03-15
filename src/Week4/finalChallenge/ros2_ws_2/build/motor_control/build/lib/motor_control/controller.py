import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult

class Controller(Node): 
    def __init__(self):
        super().__init__('ctrl')

        # Declare parameters for the PID 
        # System sample time in seconds 
        self.declare_parameter('sample_time', 0.02)
        # PID gains 
        self.declare_parameter('kP', 1.0)
        self.declare_parameter('kI', 0.5)
        self.declare_parameter('kD', 0.01)
        # Declare Motor Parameters 
        self.declare_parameter('La', 0.02844)      # Inductancia
        self.declare_parameter('Ra', 1.27)         # Resistencia
        self.declare_parameter('Ka', 0.35)         # Constante de torque
        self.declare_parameter('Kb', 0.35)         # Constante de FEM
        self.declare_parameter('Jm', 0.007)        # Inercia
        self.declare_parameter('b',  0.00173)      # Fricción viscosa
        self.declare_parameter('tau_c', 0.0)       # Par de carga
        self.declare_parameter('vmin', -9.0)       # Saturación inferior
        self.declare_parameter('vmax',  9.0)       # Saturación superior

        # Controller parameters 
        self.sample_time = self.get_parameter('sample_time').value
        self.ctrl_P = self.get_parameter('kP').value
        self.ctrl_I = self.get_parameter('kI').value
        self.ctrl_D = self.get_parameter('kD').value

        # Motor Parameters 
        self.La = self.get_parameter('La').value
        self.Ra = self.get_parameter('Ra').value
        self.Ka = self.get_parameter('Ka').value
        self.Kb = self.get_parameter('Kb').value
        self.Jm = self.get_parameter('Jm').value
        self.b  = self.get_parameter('b').value
        self.tau_c = self.get_parameter('tau_c').value
        self.vmin = self.get_parameter('vmin').value
        self.vmax = self.get_parameter('vmax').value

        # Mensaje de salida
        self.ctrl_output_msg = Float32()

        # Controller variables  
        self.set_point = 0.0          # Referencia (w_ref)
        self.motor_input = 0.0 
        self.motor_output_y = 0.0     # Medida de la velocidad del motor
        self.motor_acc = 0.0          # Aceleración angular (dot(w))
        self.i_arm = 0.0              # Corriente de armadura

        # Variables para aproximar w_dot_ref y w_ddot_ref
        self.prev_setpoint = None
        self.prev_w_dot_ref = 0.0
        self.prev_time = None

        # Declare subscribers and publishers 
        self.ctrl_input_sub = self.create_subscription(Float32, 'set_point', self.setpoint_callback, 10)
        self.ctrl_feedback_sub = self.create_subscription(Float32, 'motor_speed_y', self.motor_output_callback, 10)
        self.create_subscription(Float32, 'motor_i_arm', self.i_arm_callback, 10)
        self.acc_sub = self.create_subscription(Float32, 'motor_acc_y', self.acc_callback, 10)

        self.ctrl_output_pub = self.create_publisher(Float32, 'motor_input_u', 10)
        self.timer = self.create_timer(self.sample_time, self.timer_cb)

        # Parameter Callback 
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.get_logger().info('Controller Node Started 🚀')

    def timer_cb(self):
        """
        Se calcula la ley de control mediante linearización por retroalimentación.
        Además, se aproxima numéricamente la primera y segunda derivada de la referencia.
        
        La ley de control es:
        
            e = w_ref - w
            e_dot = ẇ_ref - ẇ   (ẇ_ref se aproxima numéricamente)
            
            u_aux = ẅ_ref + kP * e + kD * e_dot
            
            u = (Jm*La/Ka) * [
                   u_aux - (Ka/(Jm*La)*(-Ra*i_arm - Kb*w)
                   - (b/(Jm^2))*(Ka*i_arm - b*w - tau_c)
                ]
                
            u se satura entre vmin y vmax.
        """
        # Obtener tiempo actual (en segundos)
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Aproximar ẇ_ref y ẅ_ref a partir de la referencia (set_point)
        if self.prev_time is None:
            # Primera iteración: inicializamos las variables
            self.prev_time = current_time
            self.prev_setpoint = self.set_point
            w_dot_ref = 0.0
            w_ddot_ref = 0.0
        else:
            dt = current_time - self.prev_time
            # Aproximamos la primera derivada (ẇ_ref) como diferencia entre el set_point actual y el previo
            w_dot_ref = (self.set_point - self.prev_setpoint) / dt
            # Aproximamos la segunda derivada (ẅ_ref) a partir del cambio en ẇ_ref
            w_ddot_ref = (w_dot_ref - self.prev_w_dot_ref) / dt

            self.prev_setpoint = self.set_point
            self.prev_w_dot_ref = w_dot_ref
            self.prev_time = current_time

        # Calcular error y su derivada
        e = self.set_point - self.motor_output_y
        # Se asume que self.motor_acc es la aceleración medida (ẇ)
        e_dot = w_dot_ref - self.motor_acc

        # Término de control auxiliar (feedforward + retroalimentación)
        u_aux = w_ddot_ref + self.ctrl_P * e + self.ctrl_D * e_dot

        # Cálculo de u mediante linearización por retroalimentación:
        u = ((self.Jm * self.La) / self.Ka) * (u_aux - (
            (self.Ka / (self.Jm * self.La)) * (-self.Ra * self.i_arm - self.Kb * self.motor_output_y)
            - (self.b / (self.Jm * self.Jm)) * (self.Ka * self.i_arm - self.b * self.motor_output_y - self.tau_c)
        ))
                
        # Saturar u
        u_saturated = max(self.vmin, min(self.vmax, u))

        # Publicar señal de control
        self.ctrl_output_msg.data = u_saturated
        self.ctrl_output_pub.publish(self.ctrl_output_msg)

    # Subscriber Callbacks 
    def setpoint_callback(self, setpoint_sgn):
        self.set_point = setpoint_sgn.data 
    
    def motor_output_callback(self, motor_output_sgn):
        self.motor_output_y = motor_output_sgn.data

    def acc_callback(self, acc_sgn):
        self.motor_acc = acc_sgn.data
    
    def i_arm_callback(self, i_arm_msg):
        self.i_arm = i_arm_msg.data

    def parameters_callback(self, params): 
        for param in params:
            # Actualiza gains PD
            if param.name == "kP": 
                if param.value < 0.0:
                    return SetParametersResult(successful=False, reason="kP < 0")
                self.ctrl_P = param.value
            elif param.name == "kD": 
                if param.value < 0.0:
                    return SetParametersResult(successful=False, reason="kD < 0")
                self.ctrl_D = param.value
            
            # Actualiza modelo
            elif param.name == "La":
                if param.value <= 0.0:
                    return SetParametersResult(successful=False, reason="La <= 0")
                self.La = param.value
            elif param.name == "Ra":
                if param.value <= 0.0:
                    return SetParametersResult(successful=False, reason="Ra <= 0")
                self.Ra = param.value
            elif param.name == "Ka":
                if param.value < 0.0:
                    return SetParametersResult(successful=False, reason="Ka < 0")
                self.Ka = param.value
            elif param.name == "Kb":
                if param.value < 0.0:
                    return SetParametersResult(successful=False, reason="Kb < 0")
                self.Kb = param.value
            elif param.name == "Jm":
                if param.value <= 0.0:
                    return SetParametersResult(successful=False, reason="Jm <= 0")
                self.Jm = param.value
            elif param.name == "b":
                if param.value < 0.0:
                    return SetParametersResult(successful=False, reason="b < 0")
                self.b = param.value
            elif param.name == "tau_c":
                self.tau_c = param.value
            elif param.name == "vmin":
                self.vmin = param.value
            elif param.name == "vmax":
                self.vmax = param.value
            # sample_time
            elif param.name == "sample_time":
                if param.value <= 0.0:
                    return SetParametersResult(successful=False, reason="sample_time <= 0")
                self.sample_time = param.value

        return SetParametersResult(successful=True)

# Main 
def main(args=None): 
    rclpy.init(args=args)
    node = Controller() 
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass
    finally: 
        node.destroy_node() 
        rclpy.try_shutdown()

# Execute Node 
if __name__ == '__main__':
    main()
