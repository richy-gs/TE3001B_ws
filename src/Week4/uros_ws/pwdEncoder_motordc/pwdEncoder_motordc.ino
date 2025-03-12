#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include "driver/mcpwm.h"

#define ENCODER_A 19
#define ENCODER_B 21
#define PULSOS_POR_REV 895

volatile long motorPosition = 0;
unsigned long lastTime = 0;
long lastPosition = 0;

#define MOTOR_IN1 5
#define MOTOR_IN2 18
#define PWM_PIN 2
#define PWM_FREQ 5000
#define PWM_CHNL MCPWM_OPR_A
float pwm_set_point = 0.0;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t encoder_publisher;
rcl_subscription_t pwm_subscriber;
rclc_executor_t executor;
std_msgs__msg__Float32MultiArray encoder_msg;
std_msgs__msg__Float32 pwm_msg;

void updateMotorPosition() {
  if (digitalRead(ENCODER_B) != digitalRead(ENCODER_A)) {
    motorPosition++;
  } else {
    motorPosition--;
  }
}

void pwm_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  pwm_set_point = constrain(msg->data, -1.0, 1.0);

  if (pwm_set_point == 0) {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, PWM_CHNL, 0);
  } else {
    if (pwm_set_point < 0) {
      digitalWrite(MOTOR_IN1, HIGH);
      digitalWrite(MOTOR_IN2, LOW);
    } else {
      digitalWrite(MOTOR_IN1, LOW);
      digitalWrite(MOTOR_IN2, HIGH);
    }
    float duty_cycle = abs(pwm_set_point) * 100.0;
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, PWM_CHNL, duty_cycle);
  }
}

void setup() {
  Serial.begin(115200);
  set_microros_transports();
  delay(2000);

  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateMotorPosition, CHANGE);

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);

  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM_PIN);
  mcpwm_config_t pwm_config = {
    .frequency = PWM_FREQ,
    .cmpr_a = 0,
    .cmpr_b = 0,
    .duty_mode = MCPWM_DUTY_MODE_0,
    .counter_mode = MCPWM_UP_COUNTER
  };
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "motor_control_node", "", &support);

  rclc_publisher_init_default(
      &encoder_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "encoder_data");

  rclc_subscription_init_default(
      &pwm_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "cmd_pwm");

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &pwm_subscriber, &pwm_msg, &pwm_callback, ON_NEW_DATA);

  encoder_msg.data.capacity = 3;
  encoder_msg.data.size = 3;
  encoder_msg.data.data = (float*)malloc(encoder_msg.data.capacity * sizeof(float));
}

void loop() {
  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - lastTime;

  if (deltaTime >= 100) {
    long deltaPosition = motorPosition - lastPosition;
    float pulsePerSec = (deltaPosition * 1000.0) / deltaTime;
    float rpm = (pulsePerSec / PULSOS_POR_REV) * 60.0;
    float radPerSec = rpm * (2.0 * PI) / 60.0;

    encoder_msg.data.data[0] = (float)motorPosition;
    encoder_msg.data.data[1] = rpm;
    encoder_msg.data.data[2] = radPerSec;
    rcl_publish(&encoder_publisher, &encoder_msg, NULL);

    Serial.print("Pulsos: ");
    Serial.print(motorPosition);
    Serial.print(" | RPM: ");
    Serial.print(rpm);
    Serial.print(" | rad/s: ");
    Serial.println(radPerSec);

    lastPosition = motorPosition;
    lastTime = currentTime;
  }

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  delay(10);
}
