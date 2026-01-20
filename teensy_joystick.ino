#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int16_multi_array.h>

#define LED_PIN 13

const int MOTOR_PINS[6][2] = {
  {6, 0}, {7, 1}, {8, 2}, {9, 3}, {10, 4}, {11, 5}
};

rcl_subscription_t subscriber;
std_msgs__msg__Int16MultiArray motor_cmd_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

int motor_pwm_values[6] = {0, 0, 0, 0, 0, 0};

void motor_callback(const void * msgin) {
  const std_msgs__msg__Int16MultiArray * msg = 
      (const std_msgs__msg__Int16MultiArray *)msgin;
  
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  
  for (int i = 0; i < 6 && i < msg->data.size; i++) {
    motor_pwm_values[i] = msg->data.data[i];
  }
}

void apply_motors() {
  for (int i = 0; i < 6; i++) {
    int pwm = motor_pwm_values[i];
    if (pwm > 0) {
      digitalWrite(MOTOR_PINS[i][0], HIGH);
      analogWrite(MOTOR_PINS[i][1], abs(pwm));
    } else if (pwm < 0) {
      digitalWrite(MOTOR_PINS[i][0], LOW);
      analogWrite(MOTOR_PINS[i][1], abs(pwm));
    } else {
      digitalWrite(MOTOR_PINS[i][0], LOW);
      analogWrite(MOTOR_PINS[i][1], 0);
    }
  }
}

void setup() {
  // Motors
  for (int i = 0; i < 6; i++) {
    pinMode(MOTOR_PINS[i][0], OUTPUT);
    pinMode(MOTOR_PINS[i][1], OUTPUT);
    digitalWrite(MOTOR_PINS[i][0], LOW);
    analogWrite(MOTOR_PINS[i][1], 0);
  }
  
  pinMode(LED_PIN, OUTPUT);
  
  // Motor test
  for (int i = 0; i < 6; i++) {
    digitalWrite(MOTOR_PINS[i][0], HIGH);
    analogWrite(MOTOR_PINS[i][1], 150);
    delay(200);
    analogWrite(MOTOR_PINS[i][1], 0);
    delay(50);
  }
  
  set_microros_transports();
  delay(2000);
  
  allocator = rcl_get_default_allocator();
  
  // Wait for agent
  while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(200);
  }
  
  digitalWrite(LED_PIN, HIGH);
  
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "teensy_motor_node", "", &support);
  
  motor_cmd_msg.data.capacity = 6;
  motor_cmd_msg.data.size = 0;
  motor_cmd_msg.data.data = (int16_t*) malloc(6 * sizeof(int16_t));
  
  rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
      "motor_commands");
  
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  
  rclc_executor_add_subscription(
      &executor, 
      &subscriber, 
      &motor_cmd_msg, 
      &motor_callback, 
      ON_NEW_DATA);
  
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  apply_motors();
  yield();
}
