import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult
import mediapipe as mp

mp_hands = mp.solutions.hands

class SetPointPublisher(Node):
    def __init__(self):
        super().__init__('set_point_node')

        self.declare_parameter('signal_type', 'sine')  # 'sine', 'square' o 'vision'
        self.declare_parameter('amplitude', 2.0)
        self.declare_parameter('omega', 1.0)

        # Parámetros
        self.signal_type = self.get_parameter('signal_type').value
        self.amplitude = self.get_parameter('amplitude').value
        self.omega  = self.get_parameter('omega').value 

        # Publisher y timer
        self.signal_publisher = self.create_publisher(Float32, 'set_point', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_cb)
        
        # Mensaje y tiempo
        self.signal_msg = Float32()
        self.start_time = self.get_clock().now()

        # Manejo de parámetros dinámicos
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Si es modo visión, inicializamos
        self.cap = None
        self.hands_detector = None
        if self.signal_type == 'vision':
            self._init_vision()

        self.get_logger().info("SetPoint Node Started \U0001F680")

    def _init_vision(self):
        """Inicializa cámara y mediapipe."""
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("No se pudo abrir la cámara!")

        self.hands_detector = mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

    def timer_cb(self):
        # Tiempo transcurrido
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        if self.signal_type == 'sine':
            self.signal_msg.data = self.amplitude * np.sin(self.omega * elapsed_time)
        elif self.signal_type == 'square':
            self.signal_msg.data = self.amplitude * np.sign(np.sin(self.omega * elapsed_time))
        elif self.signal_type == 'vision':
            self._vision_logic()

        # Publica
        self.signal_publisher.publish(self.signal_msg)

    def _vision_logic(self):
        """
        Detecta mano, mide cuán 'abierta' está y asigna setpoint:
         - Puño cerrado => 0
         - Mano máxima abierta => ±20
         - Interpolación en el rango intermedio
        """
        if self.cap is None or self.hands_detector is None:
            self.signal_msg.data = 0.0
            return

        ret, frame = self.cap.read()
        if not ret:
            self.signal_msg.data = 0.0
            return
        
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self.hands_detector.process(frame_rgb)

        if not result.multi_hand_landmarks:
            # No detecta mano
            self.signal_msg.data = 0.0
            return

        # Tomamos la primera mano detectada
        hand_landmarks = result.multi_hand_landmarks[0]
        handedness_info = result.multi_handedness[0].classification[0].label  # 'Left' o 'Right'

        # Calculamos distancia media de yemas a la muñeca
        open_level = self._calc_open_level(hand_landmarks, frame.shape)

        # Escalado: 
        #   - Por debajo de un umbral => 0
        #   - Por encima => va hasta 20 (o -20)
        # Ajusta estos valores a tu conveniencia
        CLOSED_FIST_THRESHOLD = 30.0  # debajo de esto se considera puño cerrado
        MAX_OPEN_DIST = 150.0         # distancia a partir de la cual consideramos "completamente abierta"

        if open_level < CLOSED_FIST_THRESHOLD:
            # puño cerrado => 0
            base_speed = 0.0
        else:
            # lineal entre CLOSED_FIST_THRESHOLD y MAX_OPEN_DIST => 0..20
            rel = (open_level - CLOSED_FIST_THRESHOLD) / (MAX_OPEN_DIST - CLOSED_FIST_THRESHOLD)
            rel = max(0.0, min(rel, 1.0))  # clamp 0..1
            base_speed = rel * 20.0

        # Signo según la mano
        if handedness_info.lower() == 'right':
            self.signal_msg.data = base_speed
        else:
            self.signal_msg.data = -base_speed

    def _calc_open_level(self, hand_landmarks, frame_shape):
        """
        Mide cuán abierta está la mano, calculando la distancia promedio 
        de las yemas al landmark 0 (muñeca). 
        Retorna un valor en px (0 => muy cerrada).
        """
        image_h, image_w, _ = frame_shape
        # Indices de yemas
        fingertip_ids = [4, 8, 12, 16, 20]  # pulgar, índice, medio, anular, meñique
        # Convertimos a px
        px_coords = []
        for lm in hand_landmarks.landmark:
            px_coords.append((int(lm.x * image_w), int(lm.y * image_h)))

        wrist = px_coords[0]  # (x, y) de la muñeca
        dists = []
        for tip_id in fingertip_ids:
            tip = px_coords[tip_id]
            dist = np.hypot(tip[0] - wrist[0], tip[1] - wrist[1])
            dists.append(dist)

        # distancia promedio
        return float(np.mean(dists))

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'signal_type' and param.type_ == param.Type.STRING:
                old_type = self.signal_type
                self.signal_type = param.value
                self.get_logger().info(f'Signal type updated to: {self.signal_type}')
                if self.signal_type == 'vision' and (self.cap is None or self.hands_detector is None):
                    self._init_vision()
                elif old_type == 'vision' and self.signal_type != 'vision':
                    if self.cap:
                        self.cap.release()
                        self.cap = None
                    self.hands_detector = None
            elif param.name == 'amplitude' and param.type_ == param.Type.DOUBLE:
                self.amplitude = param.value
                self.get_logger().info(f'Amplitude updated to: {self.amplitude}')
            elif param.name == 'omega' and param.type_ == param.Type.DOUBLE:
                self.omega = param.value
                self.get_logger().info(f'Omega updated to: {self.omega}')
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    set_point = SetPointPublisher()
    try:
        rclpy.spin(set_point)
    except KeyboardInterrupt:
        pass
    finally:
        if set_point.cap:
            set_point.cap.release()
        set_point.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
