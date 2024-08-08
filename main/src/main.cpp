#include <Arduino.h>
#include "ChildMotor.h"
#include "ChildRobot.h"
#include "pinassign.h"

#include "micro_ros_platformio.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/point32.h>

#include "pinassign.h"

#include "dist_control.h"

#define INTERVAL 100

#define P_COEF_ROT 0.001
#define I_COEF_ROT 0.00003
#define P_COEF_DIST 3
#define D_COEF_DIST 0
#define P_COEF_THETA 0.025
#define D_COEF_THETA 0

ChildRobot R;

ChildMotor m1;
ChildMotor m2;
ChildMotor m3;
ChildMotor m4;

std::array<int, 4> rotary_counts = {0};
void callback_ro1(){rotary_counts[0]++;}
void callback_ro2(){rotary_counts[1]++;}
void callback_ro3(){rotary_counts[2]++;}
void callback_ro4(){rotary_counts[3]++;}

rcl_subscription_t sub_target;
rcl_subscription_t sub_reset;
rcl_publisher_t publisher;
geometry_msgs__msg__Point32 msg;

void sub_target_callback(const void *msgin);
void sub_reset_callback(const void *msgin);
void timer_callback(rclc_timer_t * timer, int64_t last_call_time);

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

void error_loop();

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void setup_motor(){
  m1.attach_motor(M1IN1, 0, M1IN2, 1);
  m1.attach_encoder(callback_ro1, &rotary_counts[0], RO1A, RO1B, INTERVAL);
  m1.attach_control(P_COEF_ROT, I_COEF_ROT);

  m2.attach_motor(M2IN1, 2, M2IN2, 3);
  m2.attach_encoder(callback_ro2, &rotary_counts[1], RO2A, RO2B, INTERVAL);
  m2.attach_control(P_COEF_ROT, I_COEF_ROT);

  m3.attach_motor(M3IN1, 4, M3IN2, 5);
  m3.attach_encoder(callback_ro3, &rotary_counts[2], RO3A, RO3B, INTERVAL);
  m3.attach_control(P_COEF_ROT, I_COEF_ROT);

  m4.attach_motor(M4IN1, 6, M4IN2, 7);
  m4.attach_encoder(callback_ro4, &rotary_counts[3], RO4A, RO4B, INTERVAL);
  m4.attach_control(P_COEF_ROT, I_COEF_ROT);
}

void setup() {
  Serial.begin(115200);

  R.set_control(P_COEF_DIST, D_COEF_DIST, P_COEF_THETA, D_COEF_THETA);
  if(!R.bno_begin(RX, TX, INTERVAL)){
      Serial.println("cannot connect to bno");
  }else{
      Serial.println("starting bno");
      delay(1000);
      R.update_bno();
      R.reset_bno();
  }
  R.set_motor(m1, m2, m3, m4);
  R.set_targ(0,0,0);

  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &sub_target,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32),
    "/ro4_target"));

  RCCHECK(rclc_subscription_init_default(
    &sub_reset,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32),
    "/ro4_reset"));

  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32),
    "/ro4_current"
  ));

  RCCHECK(rclc_executor_add_subscription(&executor, &sub_target, &msg, &sub_target_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_reset, &msg, &sub_reset_callback, ON_NEW_DATA));

  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RFL_MS_TO_NS(timer_timeout),
    timer_callback
  ));

  digitalWrite(LED_G, LOW);
  digitalWrite(LED_R, LOW);
}

void loop() {  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

void error_loop(){
  while(1){
    digitalWrite(LED_R, HIGH);
    delay(100);
  }
}

void sub_target_callback(const void *msgin){
  const geometry_msgs__msg__Point32 * msg = (const geometry_msgs__msg__Point32 *)msgin;
  R.set_target(msg.x, msg.y, msg.z);
}

void sub_reset_callback(const void *msgin){
  const geometry_msgs__msg__Point32 * msg = (const geometry_msgs__msg__Point32 *)msgin;
  R.reset(msg.x, msg.y, msg.z);
}

void timer_callback(rclc_timer_t * timer, int64_t last_call_time){
  RCLC_UNUSED(last_call_time);
  if (timer != NULL){
    geometry_msgs__msg__Point32 msg;
    std::array<float, 3> current = R.getCurrent();
    msg.x = current[0];
    msg.y = current[1];
    msg.z = current[2];
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  }
}
