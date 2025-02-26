#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int16.h>
#include <ESP32Servo.h>
#include <rmw_microros/rmw_microros.h>

#define LED_PIN 2

const int dir1 = 4;
const int pwm1 = 0;

const int dir2 = 33;
const int pwm2 = 32;

const int dir3 = 26;
const int pwm3 = 25;
const int dir4 = 12;  //scissor lift
const int pwm4 = 14;
const int dir5 = 27;  // augre
const int pwm5 = 13;
const int dir6 = 5;  //magnetic mixer
const int pwm6 = 15;
const int en = 18;
const int dirst = 19;
const int step = 23;
float b[13];

Servo servo1;
Servo servo2;
Servo servo3;

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
rcl_subscription_t subscriber_s1;
rcl_subscription_t subscriber_s2;
rcl_subscription_t subscriber_s3;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

std_msgs__msg__Float32MultiArray msg;
std_msgs__msg__Int16 msg_s1;
std_msgs__msg__Int16 msg_s2;
std_msgs__msg__Int16 msg_s3;

#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) {} \
  }
#define EXECUTE_EVERY_N_MS(MS, X) \
  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis(); } \
    if (uxr_millis() - init > MS) { \
      X; \
      init = uxr_millis(); \
    } \
  } while (0)

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

void error_loop(){
  while(1){
    digitalWrite(LED_PIN,HIGH);
    delay(100);
  }
}

void subscription_callback(const void *msgin)
{
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  const float *data = msg->data.data;
  memcpy(b, data, sizeof(b));
}
void subscription_callback_s1(const void *msgin)
{
  const std_msgs__msg__Int16 * msg = (const std_msgs__msg__Int16 *)msgin;
  servo1.write(msg->data);
}
void subscription_callback_s2(const void *msgin)
{
  const std_msgs__msg__Int16 * msg = (const std_msgs__msg__Int16 *)msgin;
  servo2.write(msg->data);

}
void subscription_callback_s3(const void *msgin)
{
  const std_msgs__msg__Int16 * msg = (const std_msgs__msg__Int16 *)msgin;
  servo3.write(msg->data);
}

bool create_entities()
{
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "ld_microros", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "publish"));

  RCCHECK(rclc_subscription_init_default(
    &subscriber_s1,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "servoc1"));

  RCCHECK(rclc_subscription_init_default(
    &subscriber_s2,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "servoc2"));

  RCCHECK(rclc_subscription_init_default(
    &subscriber_s3,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "servoc3"));      

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_s1, &msg_s1, &subscription_callback_s1, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_s2, &msg_s2, &subscription_callback_s2, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_s3, &msg_s3, &subscription_callback_s3, ON_NEW_DATA));
  return true;
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&publisher, &node);
  rcl_subscription_fini(&subscriber, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(dir3, OUTPUT);
  pinMode(dir4, OUTPUT);
  pinMode(dir5, OUTPUT);
  pinMode(dir6, OUTPUT);
  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(pwm3, OUTPUT);
  pinMode(pwm4, OUTPUT);
  pinMode(pwm5, OUTPUT);
  pinMode(pwm6, OUTPUT);
  pinMode(en, OUTPUT);
  pinMode(dirst, OUTPUT);
  pinMode(step, OUTPUT);
  digitalWrite(en, HIGH);
  servo1.attach(22);
  servo2.attach(21);
  servo3.attach(17);

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
        if (b[0] == 1) {  //pumps are unidirectional and on digital high
          digitalWrite(dir1, HIGH);

          digitalWrite(pwm1, HIGH);

        } else if (b[0] == 2) {
          digitalWrite(dir2, HIGH);

          digitalWrite(pwm2, HIGH);

        } else if (b[0] == 3) {
          digitalWrite(dir3, HIGH);

          digitalWrite(pwm3, HIGH);

        }

        // the 4 buttons on the right. If rightmost is pressed then b[2]=1 (scissor lift) if uppermost is pressed then b[2] is 2 (augre) and if leftmost is pressed then b[2] is 3 (magnet)
        // have to hold down the mentioned button and then move the left joystick up or down to change direction (button must be pressed while doing this)
        else if (b[2] == 1 && b[3] > 0.1)  // scissor lift, magnetic spinner and augre are bi directional and analog
        {
          digitalWrite(dir4, HIGH);

          analogWrite(pwm4, b[3] * 255);


        } else if (b[2] == 1 && b[3] < -0.1) {
          digitalWrite(dir4, LOW);

          analogWrite(pwm4, (-1 * b[3]) * 255);


        } else if (b[2] == 2 && b[1] > 0.1) {
          digitalWrite(dir5, HIGH);

          analogWrite(pwm5, b[3] * 255);

        } else if (b[2] == 2 && b[1] < -0.1) {
          digitalWrite(dir5, LOW);

          analogWrite(pwm5, (-1 * b[3]) * 255);

        } else if (b[2] == 3 && b[3] > 0.1) {
          digitalWrite(dir6, HIGH);

          analogWrite(pwm6, b[3] * 255);


        } else if (b[2] == 3 && b[3] < -0.1) {
          digitalWrite(dir6, LOW);

          analogWrite(pwm6, (-1 * b[3]) * 255);
        }
        //b[4] is 1 when left trigger is pressed: enabled. b[4]=0 when right trigger is pressed. b[5] is 1 when the lowermost of the left 4 buttons is pressed. is -1 when lowermost of right 4 buttons is pressed.
        if (b[4] == 1) {
          digitalWrite(en, LOW);  // stepper is enabled
          if (b[5] == 1) {
            digitalWrite(dirst, LOW);  //Changes the direction of rotation

            digitalWrite(step, HIGH);
            delayMicroseconds(500);
            digitalWrite(step, LOW);
            delayMicroseconds(500);


          } else if (b[5] == -1) {
            digitalWrite(dirst, HIGH);  // Enables the motor to move in a particular direction
            digitalWrite(step, HIGH);
            delayMicroseconds(500);
            digitalWrite(step, LOW);
            delayMicroseconds(500);
          }
        }
        if (b[4] == 0) {
          digitalWrite(en, HIGH);
        }
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
