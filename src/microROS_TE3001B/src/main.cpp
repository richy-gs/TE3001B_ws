// Include Libraries to be used
#include <micro_ros_platformio.h>    // Micro-ROS Arduino library
#include <rcl/rcl.h>              // Core ROS 2 Client Library (RCL) for node management.
#include <rcl/error_handling.h>   // Error handling utilities for Micro-ROS.
#include <rclc/rclc.h>            // Micro-ROS Client library for embedded devices.
#include <rclc/executor.h>        // Micro-ROS Executor to manage callbacks
#include <std_msgs/msg/float32.h> // Predefined ROS 2 message type
#include <stdio.h>                // Standard I/O library for debugging.
#include <Arduino.h>
#include "driver/mcpwm.h"         // MCPWM driver for ESP32
#include <cmath>


// Declare nodes to be used
rcl_node_t node;  // Represents a ROS 2 Node running on the microcontroller.


// Instantiate executor and its support classes
rclc_executor_t executor;   // Manages task execution (timers, callbacks, etc.).
rclc_support_t support;     // Data structure that holds the execution context of Micro-ROS.
rcl_allocator_t allocator;  // Manages memory allocation.


// Declare Subscribers
rcl_subscription_t subscriber;


// Declare timers
rcl_timer_t timer;


// Declare Messages
std_msgs__msg__Float32 msg;


// Define Macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


// Pin Definitions
#define LED_PIN 23      // LED Indicator
#define PWM_FREQ 5000   // PWM Frequency (Hz)
#define PWM_RES 8       // PWM Resolution (Bits)
#define PWM_CHNL MCPWM_OPR_A // MCPWM Operator


// Motor pins
#define MOTOR_IN1 5        // Pin IN1 del L298N (debe estar en HIGH o LOW)
#define MOTOR_IN2 18       // Pin IN2 del L298N (debe estar en HIGH o LOW)
#define PWM_PIN 2      // PWM Output Pin for LED/ENA


// Variables
float pwm_set_point = 0.0;  // PWM Duty Cycle


// Error Handling
void error_loop() {
   while(1) {
       digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Toggle LED state
       delay(100); // Blink every 100 ms
   }
}


// Callback for ROS2 Subscription
void subscription_callback(const void *msgin) { 
   // Get message data
   const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
   pwm_set_point = constrain(abs(msg->data), 0.0, 1.0); // Clamp input between 0 and 1


   // Convert to percentage (0 - 100%)


   if (msg->data < 0) {
     digitalWrite(MOTOR_IN1, HIGH);
     digitalWrite(MOTOR_IN2, LOW);
   }
   else {
     digitalWrite(MOTOR_IN1, LOW);
     digitalWrite(MOTOR_IN2, HIGH);
   }
  
   float duty_cycle = pwm_set_point * 100.0;
  
   // Apply PWM using MCPWM
   mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, PWM_CHNL, duty_cycle);
}


// Setup Function
void setup() {
    // Configure serial transport
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    delay(2000);

   // Motor pins init
   pinMode(MOTOR_IN1, OUTPUT);
   pinMode(MOTOR_IN2, OUTPUT);
   digitalWrite(MOTOR_IN1, HIGH);
   digitalWrite(MOTOR_IN2, LOW);


   // Initialize LED Pin
   pinMode(LED_PIN, OUTPUT);
   digitalWrite(LED_PIN, HIGH);


   // Initialize MCPWM for PWM output
   mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM_PIN);


   mcpwm_config_t pwm_config;
   pwm_config.frequency = PWM_FREQ;
   pwm_config.cmpr_a = 0;  // Start with 0% duty cycle
   pwm_config.cmpr_b = 0;  // Unused channel
   pwm_config.counter_mode = MCPWM_UP_COUNTER;
   pwm_config.duty_mode = MCPWM_DUTY_MODE_0;


   // Apply PWM Configuration
   mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);


   delay(2000); // Connection delay


   // Initialize Micro-ROS
   allocator = rcl_get_default_allocator();
   RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));


   // Create Node
   RCCHECK(rclc_node_init_default(&node, "micro_ros_sub_node", "", &support));


   // Create Subscriber
   RCCHECK(rclc_subscription_init_default(
       &subscriber,
       &node,
       ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
       "micro_ros_sub"));


   // Create Executor
   RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
   RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}


// Main Loop
void loop() {
   delay(100);
   RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100))); 
}
