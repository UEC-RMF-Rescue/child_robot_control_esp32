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

#define INTERVAL 100

#define P_COEF_ROT 0.001
#define I_COEF_ROT 0.00003
#define P_COEF_DIST 3
#define D_COEF_DIST 0
#define P_COEF_THETA 0.025
#define D_COEF_THETA 0

#define MOVE 1
#define STOP 0
int state = STOP;

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
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);

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
  pinMode(LED_G, OUTPUT);
  pinMode(LED_R, OUTPUT);

  digitalWrite(LED_G, LOW);
  digitalWrite(LED_R, HIGH);

  Serial.begin(115200);
  setup_motor();
  Serial.println("motor_setup done");

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

  digitalWrite(LED_G, HIGH);
  digitalWrite(LED_R, LOW);
}

void monitor_order(){
  if(Serial.available() > 0){
    String data = Serial.readStringUntil('\n');

    std::array<int, 3> idx;
    idx[0] = data.indexOf(',');
    idx[1] = data.indexOf(',', idx[0] + 1);
    idx[2] = data.indexOf(',', idx[1] + 1);

    int order = data.substring(0, idx[0]).toInt();
    float x = data.substring(idx[0]+1, idx[1]).toFloat();
    float y = data.substring(idx[1]+1, idx[2]).toFloat();
    float z = data.substring(idx[2]+1        ).toFloat();
    
    // if(order == 0){
    //   state = MOVE;
    //   R.set_targ(x,y,z);
    // }else if (order == 1){
    //   state = MOVE;
    //   R.reset_dist(x,y);
    //   R.reset_bno(z);
    // }else if (order == 2){
    //   state = STOP;
    // }
    if(order == 0){
      state = MOVE;
      R.set_targ(x,y,z);
    }else{
      state = STOP;
    }
  }
}

void loop() {
  if (state == MOVE){
    R.update();
    Serial.print("get order");
    Serial.println("I need it");
  }else{
    R.stop();
  }
  monitor_order();
}
