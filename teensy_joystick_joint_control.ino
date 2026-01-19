#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int16_multi_array.h>

// Motor pin definitions - YOUR ACTUAL HARDWARE
const int MOTOR_PINS[6][2] = {
  {6, 0},   // Motor 1: DIR pin 6, PWM pin 0
  {7, 1},   // Motor 2: DIR pin 7, PWM pin 1
  {8, 2},   // Motor 3: DIR pin 8, PWM pin 2
  {9, 3},   // Motor 4: DIR pin 9, PWM pin 3
  {10, 4},  // Motor 5: DIR pin 10, PWM pin 4
  {11, 5}   // Motor 6: DIR pin 12, PWM pin 5
};

// micro-ROS entities
rcl_subscription_t subscriber;
std_msgs__msg__Int16MultiArray motor_cmd_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Motor PWM values
int motor_pwm_values[6] = {0, 0, 0, 0, 0, 0};

#define LED_PIN 13

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("====================================");
  Serial.println("Teensy Motor Control Node");
  Serial.println("====================================");
  
  // Initialize motor pins
  for (int i = 0; i < 6; i++) {
    pinMode(MOTOR_PINS[i][0], OUTPUT); // DIR
    pinMode(MOTOR_PINS[i][1], OUTPUT); // PWM
    digitalWrite(MOTOR_PINS[i][0], LOW);
    analogWrite(MOTOR_PINS[i][1], 0);
  }
  Serial.println("✓ Motor pins initialized");
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  // Initialize micro-ROS
  Serial.println("Initializing micro-ROS...");
  set_microros_transports();
  delay(2000);
  
  allocator = rcl_get_default_allocator();
  
  // Create support
  Serial.println("Creating support...");
  rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
  if (ret != RCL_RET_OK) {
    Serial.print("✗ Failed to init support, error: ");
    Serial.println(ret);
    error_loop();
  }
  Serial.println("✓ Support initialized");
  
  // Create node
  Serial.println("Creating node...");
  ret = rclc_node_init_default(&node, "teensy_motor_node", "", &support);
  if (ret != RCL_RET_OK) {
    Serial.print("✗ Failed to create node, error: ");
    Serial.println(ret);
    error_loop();
  }
  Serial.println("✓ Node created");
  
  // Allocate memory for motor command array
  motor_cmd_msg.data.capacity = 6;
  motor_cmd_msg.data.size = 0;
  motor_cmd_msg.data.data = (int16_t*) malloc(motor_cmd_msg.data.capacity * sizeof(int16_t));
  
  // Create subscriber
  Serial.println("Creating subscriber to /motor_commands...");
  ret = rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
      "motor_commands");
  if (ret != RCL_RET_OK) {
    Serial.print("✗ Failed to create subscriber, error: ");
    Serial.println(ret);
    error_loop();
  }
  Serial.println("✓ Subscriber created");
  
  // Create executor
  Serial.println("Creating executor...");
  ret = rclc_executor_init(&executor, &support.context, 1, &allocator);
  if (ret != RCL_RET_OK) {
    Serial.print("✗ Failed to init executor, error: ");
    Serial.println(ret);
    error_loop();
  }
  Serial.println("✓ Executor created");
  
  // Add subscription
  Serial.println("Adding subscription...");
  ret = rclc_executor_add_subscription(
      &executor, 
      &subscriber, 
      &motor_cmd_msg, 
      &motor_callback, 
      ON_NEW_DATA);
  if (ret != RCL_RET_OK) {
    Serial.print("✗ Failed to add subscription, error: ");
    Serial.println(ret);
    error_loop();
  }
  Serial.println("✓ Subscription added");
  
  Serial.println("====================================");
  Serial.println("✓✓✓ Teensy Motor Node Ready!");
  Serial.println("====================================");
  Serial.println("Listening to: /motor_commands");
  Serial.println("Expecting: [m1, m2, m3, m4, m5, m6]");
  Serial.println("PWM range: -255 to +255");
  Serial.println("====================================");
  
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  // Spin executor
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  
  // Apply motor commands
  apply_motor_commands();
  
  delay(10);
}

// Motor command callback
void motor_callback(const void * msgin) {
  const std_msgs__msg__Int16MultiArray * msg = 
      (const std_msgs__msg__Int16MultiArray *)msgin;
  
  Serial.print("📥 Received: [");
  
  // Update motor values (up to 6 motors)
  for (int i = 0; i < 6 && i < msg->data.size; i++) {
    motor_pwm_values[i] = msg->data.data[i];
    Serial.print(motor_pwm_values[i]);
    if (i < 5) Serial.print(", ");
  }
  
  Serial.println("]");
  
  // Show active motors
  for (int i = 0; i < 6; i++) {
    if (motor_pwm_values[i] != 0) {
      Serial.print("  Motor ");
      Serial.print(i + 1);
      Serial.print(": ");
      if (motor_pwm_values[i] > 0) {
        Serial.print("FWD ");
      } else {
        Serial.print("REV ");
      }
      Serial.println(abs(motor_pwm_values[i]));
    }
  }
}

// Apply motor commands to hardware
void apply_motor_commands() {
  for (int i = 0; i < 6; i++) {
    int pwm = motor_pwm_values[i];
    
    if (pwm > 0) {
      // Forward
      digitalWrite(MOTOR_PINS[i][0], HIGH);
      analogWrite(MOTOR_PINS[i][1], abs(pwm));
    } else if (pwm < 0) {
      // Reverse
      digitalWrite(MOTOR_PINS[i][0], LOW);
      analogWrite(MOTOR_PINS[i][1], abs(pwm));
    } else {
      // Stop
      digitalWrite(MOTOR_PINS[i][0], LOW);
      analogWrite(MOTOR_PINS[i][1], 0);
    }
  }
}

// Error loop - blink LED rapidly
void error_loop() {
  Serial.println("✗✗✗ ERROR - Entering error loop");
  Serial.println("LED will blink rapidly");
  while(1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}
