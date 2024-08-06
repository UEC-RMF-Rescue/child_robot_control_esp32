#include <Arduino.h>
#include "ChildMotor.h"
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

  m2.attach_motor(M2IN1, 2, M2IN2, 3);
  m2.attach_encoder(callback_ro2, &counts[1], RO2A, RO2B, INTERVAL);
  m2.attach_control(P_COEF, I_COEF);

  m3.attach_motor(M3IN1, 4, M3IN2, 5);
  m3.attach_encoder(callback_ro3, &counts[2], RO3A, RO3B, INTERVAL);
  m3.attach_control(P_COEF, I_COEF);

  m4.attach_motor(M4IN1, 6, M4IN2, 7);
  m4.attach_encoder(callback_ro4, &counts[3], RO4A, RO4B, INTERVAL);
  m4.attach_control(P_COEF, I_COEF);

  Serial.begin(115200);
}

int count;
float targ = 100;
void loop() {
  if (count > 2400){
    targ = 100;
    count = 0;
  }else if (count > 1800){
    targ = -250;
  }else if (count > 1200){
    targ = 250;
  }else if (count > 600){
    targ = -100;
  }
  count ++;

  m1.update(targ);
  m2.update(targ);
  m3.update(targ);
  m4.update(targ);

  Serial.println(count);

  Serial.print("m1...");
  Serial.print("error: ");
  Serial.print(m1.getError());
  Serial.print(", vel: ");
  Serial.print(m1.getVel());
  Serial.print(", duty: ");
  Serial.println(m1.getDuty());

  Serial.print("m2...");
  Serial.print("error: ");
  Serial.print(m2.getError());
  Serial.print(", vel: ");
  Serial.print(m2.getVel());
  Serial.print(", duty: ");
  Serial.println(m2.getDuty());

  Serial.print("m3...");
  Serial.print("error: ");
  Serial.print(m3.getError());
  Serial.print(", vel: ");
  Serial.print(m3.getVel());
  Serial.print(", duty: ");
  Serial.println(m3.getDuty());

  Serial.print("m4...");
  Serial.print("error: ");
  Serial.print(m4.getError());
  Serial.print(", vel: ");
  Serial.print(m4.getVel());
  Serial.print(", duty: ");
  Serial.println(m4.getDuty());
}
