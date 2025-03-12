#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>

#define ENCODER_A 19
#define ENCODER_B 21
#define PULSOS_POR_REV 1000  // Ajusta según el encoder

// Variables del encoder
volatile long motorPosition = 0;
unsigned long lastTime = 0;
long lastPosition = 0;

// micro-ROS
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t publisher;
rclc_executor_t executor;
std_msgs__msg__Float32MultiArray msg;

void updateMotorPosition() {
  if (digitalRead(ENCODER_B) != digitalRead(ENCODER_A)) {
    motorPosition++;
  } else {
    motorPosition--;
  }
}

void setup() {
  // Configurar encoder
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateMotorPosition, CHANGE);
  
  // Inicializar Serial y micro-ROS
  Serial.begin(115200);
  set_microros_transports();
  
  // Inicializar soporte de micro-ROS
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "encoder_node", "", &support);
  rclc_publisher_init_default(
      &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "encoder_data");

  msg.data.capacity = 3;
  msg.data.size = 3;
  msg.data.data = (float*)malloc(msg.data.capacity * sizeof(float));
}

void loop() {
  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - lastTime;

  if (deltaTime >= 100) {  // Medir cada 100 ms
    long deltaPosition = motorPosition - lastPosition;
    float pulsePerSec = (deltaPosition * 1000.0) / deltaTime;
    float rpm = (pulsePerSec / PULSOS_POR_REV) * 60.0;
    float radPerSec = rpm * (2.0 * PI) / 60.0;

    // Asignar valores al mensaje ROS2
    msg.data.data[0] = (float)motorPosition;  // Total de pulsos
    msg.data.data[1] = rpm;                   // Velocidad en RPM
    msg.data.data[2] = radPerSec;              // Velocidad en rad/s

    // Publicar en ROS2
    rcl_publish(&publisher, &msg, NULL);

    // Mostrar en Serial
    Serial.print("Pulsos: ");
    Serial.print(motorPosition);
    Serial.print(" | RPM: ");
    Serial.print(rpm);
    Serial.print(" | rad/s: ");
    Serial.println(radPerSec);

    // Actualizar valores previos
    lastPosition = motorPosition;
    lastTime = currentTime;
  }

  delay(10);  // Pequeña pausa para evitar sobrecarga
}
