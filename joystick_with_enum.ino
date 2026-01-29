#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int16_multi_array.h>

#define LED_PIN 13

const int MOTOR_PINS[6][2] = {
  {6, 0}, {7, 1}, {8, 2}, {9, 3}, {10, 4}, {11, 5}
};

// State enumeration
enum SystemState {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
};

SystemState current_state = WAITING_AGENT;

rcl_subscription_t subscriber;
std_msgs__msg__Int16MultiArray motor_cmd_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

int motor_pwm_values[6] = {0, 0, 0, 0, 0, 0};
unsigned long last_connection_check = 0;
const unsigned long CONNECTION_CHECK_INTERVAL = 500; // Check every 500ms

void motor_callback(const void * msgin) {
  const std_msgs__msg__Int16MultiArray * msg = (const std_msgs__msg__Int16MultiArray *)msgin;
  
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

void stop_all_motors() {
  for (int i = 0; i < 6; i++) {
    motor_pwm_values[i] = 0;
    digitalWrite(MOTOR_PINS[i][0], LOW);
    analogWrite(MOTOR_PINS[i][1], 0);
  }
}

void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  
  rcl_subscription_fini(&subscriber, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

bool create_entities() {
  allocator = rcl_get_default_allocator();
  
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    return false;
  }
  
  if (rclc_node_init_default(&node, "teensy_motor_node", "", &support) != RCL_RET_OK) {
    return false;
  }
  
  motor_cmd_msg.data.capacity = 6;
  motor_cmd_msg.data.size = 0;
  motor_cmd_msg.data.data = (int16_t*) malloc(6 * sizeof(int16_t));
  
  if (rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
      "motor_commands") != RCL_RET_OK) {
    return false;
  }
  
  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) {
    return false;
  }
  
  if (rclc_executor_add_subscription(
      &executor, &subscriber, &motor_cmd_msg, &motor_callback, ON_NEW_DATA) != RCL_RET_OK) {
    return false;
  }
  
  return true;
}

void setup() {
  // Initialize motors
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
  
  current_state = WAITING_AGENT;
}

void loop() {
  unsigned long current_time = millis();
  
  switch (current_state) {
    case WAITING_AGENT:
      // Blink LED while waiting
      if (current_time - last_connection_check >= CONNECTION_CHECK_INTERVAL) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        last_connection_check = current_time;
        
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
          current_state = AGENT_AVAILABLE;
          digitalWrite(LED_PIN, HIGH);
        }
      }
      stop_all_motors();
      break;
      
    case AGENT_AVAILABLE:
      if (create_entities()) {
        current_state = AGENT_CONNECTED;
        digitalWrite(LED_PIN, LOW);
      } else {
        current_state = WAITING_AGENT;
      }
      break;
      
    case AGENT_CONNECTED:
      // Check connection periodically
      if (current_time - last_connection_check >= CONNECTION_CHECK_INTERVAL) {
        last_connection_check = current_time;
        
        if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
          current_state = AGENT_DISCONNECTED;
        }
      }
      
      // Normal operation
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
      apply_motors();
      break;
      
    case AGENT_DISCONNECTED:
      stop_all_motors();
      destroy_entities();
      digitalWrite(LED_PIN, LOW);
      current_state = WAITING_AGENT;
      break;
  }
  
  yield();
}
