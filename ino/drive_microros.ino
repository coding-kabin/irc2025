#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/float32.h>
#include <Cytron_SmartDriveDuo.h>
#include <rmw_microros/rmw_microros.h>

#define IN1 27
#define IN2 14
#define IN3 12
#define BAUDRATE 115200
#define LED_PIN 13
#define ARRAY_LEN 6
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Float32MultiArray msg;
std_msgs__msg__Float32 msg_fb; // Feedback message
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;

#define fan1 18
#define fan2 23
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

Cytron_SmartDriveDuo motor_back(SERIAL_SIMPLIFIED, IN1, BAUDRATE);
Cytron_SmartDriveDuo motor_mid(SERIAL_SIMPLIFIED, IN2, BAUDRATE);
Cytron_SmartDriveDuo motor_front(SERIAL_SIMPLIFIED, IN3, BAUDRATE);

float right_wheel = 0.0;
float left_wheel = 0.0;
float right_wheel_mid = 0.0;
float left_wheel_mid = 0.0;
float right_wheel_front = 0.0;
float left_wheel_front = 0.0;
float right_wheel_back = 0.0;
float left_wheel_back = 0.0;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

bool publish_feedback = false; // Flag to publish feedback

void error_loop(){
  while(1){
    digitalWrite(fan1, LOW);
    digitalWrite(fan2, LOW);
    delay(100);
  }
}

void subscription_callback(const void *msgin)
{
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  right_wheel_front = msg->data.data[0];
  left_wheel_front = msg->data.data[1];
  right_wheel_mid = msg->data.data[2];
  left_wheel_mid = msg->data.data[3];
  right_wheel_back = msg->data.data[4];
  left_wheel_back = msg->data.data[5];

  msg_fb.data = (right_wheel_front+left_wheel_front+right_wheel_mid+left_wheel_mid+right_wheel_back+left_wheel_back)/6.0;
  // Set flag to publish feedback
  publish_feedback = true;
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL && publish_feedback) {
    RCCHECK(rcl_publish(&publisher, &msg_fb, NULL));
    publish_feedback = false;
  }
}


bool create_entities()
{
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  msg.data.capacity = ARRAY_LEN;
  msg.data.size = 0;
  msg.data.data = (float*) malloc(msg.data.capacity * sizeof(float));
  
  msg_fb.data = 0.0;

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "rover"));

  // Initialize feedback publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "feedback"));

  RCCHECK(rclc_timer_init_default(
  &timer,
  &support,
  RCL_MS_TO_NS(100),  // Publish at 10Hz
  timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  return true;
}


void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  free(msg.data.data);

  rcl_subscription_fini(&subscriber, &node);
  rcl_publisher_fini(&publisher, &node); // Clean up publisher
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  pinMode(fan1, OUTPUT);
  pinMode(fan2, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(fan1, HIGH);
  digitalWrite(fan2, HIGH);
  
  set_microros_transports();
  delay(1000);

  state = WAITING_AGENT;
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        motor_back.control(left_wheel_back, right_wheel_back);
        motor_mid.control(left_wheel_mid, right_wheel_mid);
        motor_front.control(left_wheel_front, right_wheel_front);
        RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  if (state == AGENT_CONNECTED) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
}