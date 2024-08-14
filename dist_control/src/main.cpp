#include <Arduino.h>
#include "ChildMotor.h"
#include "ChildRobot.h"
#include "pinassign.h"

#define INTERVAL 100

// #define P_COEF_ROT 0.0008
// #define I_COEF_ROT 0.00003
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
    setup_motor();

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

    R.set_targ(0 ,0 ,0);
    R.reset_bno(180);
}

int i;
void loop() {    
    R.update();
    
    Serial.print("dist: ");
    Serial.print(R.getCurrent()[0]);
    Serial.print(", ");
    Serial.print(R.getCurrent()[1]);
    Serial.print(", angle: ");
    Serial.print(R.getAngle());
    Serial.print(", error: ");
    Serial.print(R.getError()[0]);
    Serial.print(", ");
    Serial.print(R.getError()[1]);
    Serial.print(", ");
    Serial.println(R.getError()[2]);
}
