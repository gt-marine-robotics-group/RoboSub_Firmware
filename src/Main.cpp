#include <Arduino.h>
#include "micro_ros_platformio.h"

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <Servo.h>

rcl_subscription_t subscriber0;
rcl_subscription_t subscriber1;
rcl_subscription_t subscriber2;
rcl_subscription_t subscriber3;
rcl_subscription_t subscriber4;

std_msgs__msg__Int32 msg0;
std_msgs__msg__Int32 msg1;
std_msgs__msg__Int32 msg2;
std_msgs__msg__Int32 msg3;
std_msgs__msg__Int32 msg4;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

Servo servo0;
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

#define LED_PIN 13
#define ESC_0 2
#define ESC_1 6
#define ESC_2 9
#define ESC_3 7
#define ESC_4 0


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, HIGH);
    delay(10);
  }
}

void subscription_callback0(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH); 
  int pwm = msg->data; 
  pwm = 1500 + (4*constrain(pwm, -100, 100));
  servo0.writeMicroseconds(pwm);
}

void subscription_callback1(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH); 

  int pwm = msg->data; 
  pwm = 1500 + (4*constrain(pwm, -100, 100));
  servo1.writeMicroseconds(pwm);
}

void subscription_callback2(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH);

  int pwm = msg->data; 
  pwm = 1500 + (4*constrain(pwm, -100, 100));
  servo2.writeMicroseconds(pwm);
}

void subscription_callback3(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH);  
  int pwm = msg->data; 
  pwm = 1500 + (4*constrain(pwm, -100, 100));
  servo3.writeMicroseconds(pwm);
}

void subscription_callback4(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH); 

  int pwm = msg->data; 
  pwm = 1500 + (4*constrain(pwm, -100, 100));
  servo4.writeMicroseconds(pwm);
}

void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  

  //pinMode(ESC_0, OUTPUT);
  servo0.attach(ESC_0);
  servo1.attach(ESC_1);
  servo2.attach(ESC_2);
  servo3.attach(ESC_3);
  servo4.attach(ESC_4);
 
  servo0.writeMicroseconds(1500);
  servo1.writeMicroseconds(1500);
  servo2.writeMicroseconds(1500);
  servo3.writeMicroseconds(1500);
  servo4.writeMicroseconds(1500);

  delay(1000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "mr", "", &support));

  // create subscriber0
  RCCHECK(rclc_subscription_init_best_effort(
    &subscriber0,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "nekton/thrusters/id_0/input"));

  // create subscriber1
  RCCHECK(rclc_subscription_init_best_effort(
    &subscriber1,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "nekton/thrusters/id_1/input"));

  // create subscriber2
  RCCHECK(rclc_subscription_init_best_effort(
    &subscriber2,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "nekton/thrusters/id_2/input"));

  // create subscriber3
  RCCHECK(rclc_subscription_init_best_effort(
    &subscriber3,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "nekton/thrusters/id_3/input"));

  // create subscriber4
  RCCHECK(rclc_subscription_init_best_effort(
    &subscriber4,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "nekton/thrusters/id_4/input"));
  

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber0, &msg0, &subscription_callback0, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber1, &msg1, &subscription_callback1, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber2, &msg2, &subscription_callback2, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber3, &msg3, &subscription_callback3, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber4, &msg4, &subscription_callback4, ON_NEW_DATA));

  /*rcl_subscription_fini(&subscriber0, &node);
  rcl_subscription_fini(&subscriber1, &node);
  rcl_subscription_fini(&subscriber2, &node);
  rcl_subscription_fini(&subscriber3, &node);
  rcl_subscription_fini(&subscriber4, &node); */
}

void loop() {
  //delay(100);

  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
