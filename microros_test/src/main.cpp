#include <Arduino.h>
#include "micro_ros_platformio.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/point32.h>

#include "pinassign.h"

rcl_subscription_t subscriber;
rcl_publisher_t publisher;
geometry_msgs__msg__Point32 msg;
void subscription_callback(const void *msgin);

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

void error_loop();

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32),
    "micro_ros_platformio_node_subscriber"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA))

  pinMode(LED_G, OUTPUT);
  pinMode(LED_R, OUTPUT);
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_R, LOW);
}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

void error_loop(){
  while(1){
    digitalWrite(LED_R, HIGH);
    delay(100);
  }
}

void subscription_callback(const void *msgin){
  const geometry_msgs__msg__Point32 * msg = (const geometry_msgs__msg__Point32 *)msgin;
  digitalWrite(LED_G, (msg->x > 0) ? HIGH : LOW);
  digitalWrite(LED_R, (msg->y > 0) ? HIGH : LOW);
}
