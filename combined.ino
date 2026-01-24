#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <std_msgs/msg/string.h>
#include <Encoder.h>
#include <CytronMotorDriver.h>

// ================== CONTROL MODE ==================
enum ControlMode {
  MODE_PID,
  MODE_PWM
};
ControlMode current_mode = MODE_PWM;

// ================== MOTOR CONFIGURATION ==================
#define NUM_MOTORS 6
#define LED_PIN 13

#define ENC_A_1 23
#define ENC_B_1 19
#define MOTOR_PWM_1 0
#define MOTOR_DIR_1 6

#define ENC_A_2 20
#define ENC_B_2 21
#define MOTOR_PWM_2 1
#define MOTOR_DIR_2 7

#define ENC_A_3 40
#define ENC_B_3 37
#define MOTOR_PWM_3 2
#define MOTOR_DIR_3 8

#define ENC_A_4 18
#define ENC_B_4 17
#define MOTOR_PWM_4 3
#define MOTOR_DIR_4 9

#define ENC_A_5 22
#define ENC_B_5 15
#define MOTOR_PWM_5 4
#define MOTOR_DIR_5 10

#define ENC_A_6 16
#define ENC_B_6 14
#define MOTOR_PWM_6 5
#define MOTOR_DIR_6 11

const long COUNTS_PER_REV[NUM_MOTORS] = {
  480000, 400000, 320000, 36000, 240000, 240000
};

const int ENCODER_DIRECTION[NUM_MOTORS] = {
  1, 1, -1, 1, 1, 1
};

#define MAX_PWM 200
#define MIN_PWM 40
#define CONTROL_FREQUENCY 1000  // Reduced from 5000 to 1000Hz
#define TIMER_PERIOD_US 1000    // 1ms = 1000us
#define TIMER_DT 0.001          // 1ms in seconds
#define PUBLISH_FREQUENCY 50    // Publish at 50Hz
#define PUBLISH_DECIMATION (CONTROL_FREQUENCY / PUBLISH_FREQUENCY)

// ================== PID PARAMETERS ==================
struct MotorPID {
  float kp, ki, kd;
  float target_angle, current_angle;
  float error, last_error, integral;
  int pwm;
  int settled_count;
};

MotorPID motors[NUM_MOTORS];
const float TOLERANCE_DEG = 0.5;
const int SETTLED_THRESHOLD = CONTROL_FREQUENCY;

int direct_pwm[NUM_MOTORS] = {0, 0, 0, 0, 0, 0};

Encoder* encoders[NUM_MOTORS];
CytronMD* motor_drivers[NUM_MOTORS];

// ================== MICRO-ROS ==================
// Only 1 publisher now - just joint states
rcl_publisher_t pub_joint_states;

// 3 subscribers: joint commands, pwm commands, control mode
rcl_subscription_t sub_joint_commands;
rcl_subscription_t sub_pwm_commands;
rcl_subscription_t sub_control_mode;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

sensor_msgs__msg__JointState joint_state_msg;
std_msgs__msg__Float64MultiArray joint_command_msg;
std_msgs__msg__Int16MultiArray pwm_command_msg;
std_msgs__msg__String mode_msg;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

float countsToAngle(long counts, int motor_idx) {
  float angle = (counts * 360.0) / COUNTS_PER_REV[motor_idx];
  return angle * ENCODER_DIRECTION[motor_idx];
}

// ================== CALLBACKS ==================
void mode_callback(const void * msgin) {
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  if (strcmp(msg->data.data, "PID_CONTROL") == 0) {
    current_mode = MODE_PID;
    for (int i = 0; i < NUM_MOTORS; i++) {
      motors[i].integral = 0;
      motors[i].settled_count = 0;
    }
  } else if (strcmp(msg->data.data, "PWM_CONTROL") == 0) {
    current_mode = MODE_PWM;
    for (int i = 0; i < NUM_MOTORS; i++) {
      direct_pwm[i] = 0;
    }
  }
}

void pwm_command_callback(const void * msgin) {
  const std_msgs__msg__Int16MultiArray * msg = (const std_msgs__msg__Int16MultiArray *)msgin;
  for (size_t i = 0; i < NUM_MOTORS && i < msg->data.size; i++) {
    direct_pwm[i] = msg->data.data[i];
  }
}

void joint_command_callback(const void * msgin) {
  const std_msgs__msg__Float64MultiArray * msg = (const std_msgs__msg__Float64MultiArray *)msgin;
  for (size_t i = 0; i < NUM_MOTORS && i < msg->data.size; i++) {
    float new_target = msg->data.data[i] * (180.0 / M_PI);
    if (i == 2) { new_target = new_target * -1; }
    if (abs(new_target - motors[i].target_angle) > 0.1) {
      motors[i].target_angle = new_target;
      motors[i].integral = 0;
      motors[i].settled_count = 0;
    }
  }
}

void update_motor_pid(int motor_idx, float dt) {
  MotorPID &m = motors[motor_idx];
  m.error = m.target_angle - m.current_angle;
  m.integral += m.error * dt;
  m.integral = constrain(m.integral, -50, 50);
  float derivative = (m.error - m.last_error) / dt;
  float p_term = m.kp * m.error;
  float i_term = m.ki * m.integral;
  float d_term = m.kd * derivative;
  float pid_output = p_term + i_term + d_term;
  m.pwm = constrain(pid_output, -MAX_PWM, MAX_PWM);
  if (abs(m.pwm) > 0 && abs(m.pwm) < MIN_PWM) {
    m.pwm = (m.pwm > 0) ? MIN_PWM : -MIN_PWM;
  }
  if (abs(m.error) < TOLERANCE_DEG) {
    m.settled_count++;
    if (m.settled_count >= SETTLED_THRESHOLD) {
      m.pwm = 0;
    }
  } else {
    m.settled_count = 0;
  }
  m.last_error = m.error;
}

// ================== TIMER CALLBACK ==================
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    const float dt = TIMER_DT;
    static int publish_counter = 0;
    publish_counter++;
    
    // Read encoders
    for (int i = 0; i < NUM_MOTORS; i++) {
      motors[i].current_angle = countsToAngle(encoders[i]->read(), i);
    }
    
    // Apply control based on mode
    if (current_mode == MODE_PID) {
      for (int i = 0; i < NUM_MOTORS; i++) {
        update_motor_pid(i, dt);
        motor_drivers[i]->setSpeed(motors[i].pwm);
      }
    } else {
      for (int i = 0; i < NUM_MOTORS; i++) {
        motor_drivers[i]->setSpeed(direct_pwm[i]);
      }
    }
    
    // Publish at reduced rate (50Hz)
    if (publish_counter >= PUBLISH_DECIMATION) {
      publish_counter = 0;
      
      unsigned long current_time = millis();
      
      // Only update joint state message
      for (int i = 0; i < NUM_MOTORS; i++) {
        float angle_rad = motors[i].current_angle * (M_PI / 180.0);
        if (i == 2) { angle_rad = -angle_rad; }
        joint_state_msg.position.data[i] = angle_rad;
        joint_state_msg.velocity.data[i] = 0.0;
      }
      
      joint_state_msg.header.stamp.sec = current_time / 1000;
      joint_state_msg.header.stamp.nanosec = (current_time % 1000) * 1000000;
      
      // Only publish joint states
      RCSOFTCHECK(rcl_publish(&pub_joint_states, &joint_state_msg, NULL));
    }
  }
}

// ================== SETUP ==================
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // Initialize motor PID parameters
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].kp = 10.5;
    motors[i].ki = 0.0;
    motors[i].kd = 0.0;
    motors[i].target_angle = 0.0;
    motors[i].current_angle = 0.0;
    motors[i].error = 0.0;
    motors[i].last_error = 0.0;
    motors[i].integral = 0.0;
    motors[i].pwm = 0;
    motors[i].settled_count = 0;
  }

  // Initialize encoders and motor drivers
  encoders[0] = new Encoder(ENC_A_1, ENC_B_1);
  motor_drivers[0] = new CytronMD(PWM_DIR, MOTOR_PWM_1, MOTOR_DIR_1);
  encoders[1] = new Encoder(ENC_A_2, ENC_B_2);
  motor_drivers[1] = new CytronMD(PWM_DIR, MOTOR_PWM_2, MOTOR_DIR_2);
  encoders[2] = new Encoder(ENC_A_3, ENC_B_3);
  motor_drivers[2] = new CytronMD(PWM_DIR, MOTOR_PWM_3, MOTOR_DIR_3);
  encoders[3] = new Encoder(ENC_A_4, ENC_B_4);
  motor_drivers[3] = new CytronMD(PWM_DIR, MOTOR_PWM_4, MOTOR_DIR_4);
  encoders[4] = new Encoder(ENC_A_5, ENC_B_5);
  motor_drivers[4] = new CytronMD(PWM_DIR, MOTOR_PWM_5, MOTOR_DIR_5);
  encoders[5] = new Encoder(ENC_A_6, ENC_B_6);
  motor_drivers[5] = new CytronMD(PWM_DIR, MOTOR_PWM_6, MOTOR_DIR_6);

  // Initialize micro-ROS
  set_microros_transports();
  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "teensy_motor_control", "", &support));

  // ================== QoS Configuration ==================
  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  qos_profile.depth = 1;

  // ================== Allocate Joint State Message ==================
  joint_state_msg.name.capacity = NUM_MOTORS;
  joint_state_msg.name.size = NUM_MOTORS;
  joint_state_msg.name.data = (rosidl_runtime_c__String*) malloc(NUM_MOTORS * sizeof(rosidl_runtime_c__String));
  
  joint_state_msg.position.capacity = NUM_MOTORS;
  joint_state_msg.position.size = NUM_MOTORS;
  joint_state_msg.position.data = (double*) malloc(NUM_MOTORS * sizeof(double));
  
  joint_state_msg.velocity.capacity = NUM_MOTORS;
  joint_state_msg.velocity.size = NUM_MOTORS;
  joint_state_msg.velocity.data = (double*) malloc(NUM_MOTORS * sizeof(double));
  
  joint_state_msg.effort.capacity = 0;
  joint_state_msg.effort.size = 0;

  // Frame ID
  static char frame_id[] = "base_link";
  joint_state_msg.header.frame_id.data = frame_id;
  joint_state_msg.header.frame_id.size = strlen(frame_id);
  joint_state_msg.header.frame_id.capacity = sizeof(frame_id);

  // Joint names
  const char* joint_names[NUM_MOTORS] = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
  for (int i = 0; i < NUM_MOTORS; i++) {
    joint_state_msg.name.data[i].data = (char*) joint_names[i];
    joint_state_msg.name.data[i].size = strlen(joint_names[i]);
    joint_state_msg.name.data[i].capacity = strlen(joint_names[i]) + 1;
  }

  // ================== Allocate Subscriber Messages ==================
  joint_command_msg.data.capacity = NUM_MOTORS;
  joint_command_msg.data.size = 0;
  joint_command_msg.data.data = (double*) malloc(NUM_MOTORS * sizeof(double));
  
  pwm_command_msg.data.capacity = NUM_MOTORS;
  pwm_command_msg.data.size = 0;
  pwm_command_msg.data.data = (int16_t*) malloc(NUM_MOTORS * sizeof(int16_t));
  
  mode_msg.data.data = (char*) malloc(20);
  mode_msg.data.capacity = 20;
  mode_msg.data.size = 0;

  // ================== Create Publisher (only joint states) ==================
  RCCHECK(rclc_publisher_init(&pub_joint_states, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), 
    "/joint_states_teensy", &qos_profile));

  // ================== Create Subscribers ==================
  RCCHECK(rclc_subscription_init(&sub_joint_commands, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray), 
    "/joint_commands_to_teensy", &qos_profile));
    
  RCCHECK(rclc_subscription_init(&sub_pwm_commands, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray), 
    "/motor_commands", &qos_profile));
    
  RCCHECK(rclc_subscription_init(&sub_control_mode, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), 
    "/control_mode", &qos_profile));

  // ================== Executor: 1 timer + 3 subscriptions = 4 ==================
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_US_TO_NS(TIMER_PERIOD_US), timer_callback));
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_joint_commands, &joint_command_msg, &joint_command_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_pwm_commands, &pwm_command_msg, &pwm_command_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_control_mode, &mode_msg, &mode_callback, ON_NEW_DATA));

  digitalWrite(LED_PIN, LOW);
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
