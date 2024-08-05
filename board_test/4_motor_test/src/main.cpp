#include <Arduino.h>
#include "ChildMotor_dev.h"
#include "pinassign.h"

#define INTERVAL 100

#define P_COEF 0.0008
#define I_COEF 0.00003

ChildMotor m1;
ChildMotor m2;
ChildMotor m3;
ChildMotor m4;

int counts[4] = {0};
void callback_ro1(){counts[0]++;}
void callback_ro2(){counts[1]++;}
void callback_ro3(){counts[2]++;}
void callback_ro4(){counts[3]++;}

void setup() {
  pinMode(LED_G, OUTPUT);
  digitalWrite(LED_G, LOW);
  pinMode(LED_R, OUTPUT);
  digitalWrite(LED_R, LOW);

  m1.attach_motor(M1IN1, 0, M1IN2, 1);
  m1.attach_encoder(callback_ro1, &counts[0], RO1A, RO1B, INTERVAL);
  m1.attach_control(P_COEF, I_COEF);

  m2.attach_motor(M2IN1, 0, M2IN2, 1);
  m2.attach_encoder(callback_ro2, &counts[1], RO2A, RO2B, INTERVAL);
  m2.attach_control(P_COEF, I_COEF);

  // m3.attach_motor(M3IN1, 0, M3IN2, 1);
  // m3.attach_encoder(RO3A, RO3B, INTERVAL);
  // m3.attach_control(P_COEF, I_COEF);

  // m4.attach_motor(M4IN1, 0, M4IN2, 1);
  // m4.attach_encoder(RO4A, RO4B, INTERVAL);
  // m4.attach_control(P_COEF, I_COEF);

  Serial.begin(115200);
}

long previousMillis = 0;
long currentMillis;
void loop() {
  // m1.update(100);
  // m2.update(0);
  // m3.update(0);
  // m4.update(0);

  m1.move_motor(0.2);
  m2.move_motor(0.2);

  m1.update_rotary();
  m2.update_rotary();
  
  // currentMillis = millis();
  // if (currentMillis - previousMillis > INTERVAL){
  //   previousMillis = currentMillis;
  //   m1.update_func();
  //   m2.update_func();
    
  //   m1.reset();
  // }
  
  Serial.print("m1: ");
  // Serial.print("error: ");
  // Serial.print(m1.getError());
  Serial.print(", vel: ");
  Serial.print(m1.getVel());

  Serial.print(", value: ");
  Serial.print(m1.getValue());
  
  // Serial.print(", dir: ");
  // Serial.print(m1.getDir());
  Serial.print(", duty: ");
  Serial.println(m1.getDuty());

  Serial.print("m2: ");
  // Serial.print("error: ");
  // Serial.print(m2.getError());
  Serial.print(", vel: ");
  Serial.print(m2.getVel());

  Serial.print(", value: ");
  Serial.print(m2.getValue());

  // Serial.print(", dir: ");
  // Serial.print(m2.getDir());
  Serial.print(", duty: ");
  Serial.println(m2.getDuty());
}
