#include <Arduino.h>
// #include "QRotary.h"
#include "ChildMotor.h"
#include "pinassign.h"

#define INTERVAL 100

#define P_COEF 0.0008
#define I_COEF 0.00003

ChildMotor m;

void setup() {
  pinMode(LED_G, OUTPUT);
  digitalWrite(LED_G, LOW);
  pinMode(LED_R, OUTPUT);
  digitalWrite(LED_R, LOW);

  m.attach_motor(M4IN1, 0, M4IN2, 1);
  m.attach_encoder(RO4A, RO4B, INTERVAL);
  m.attach_control(P_COEF, I_COEF);

  Serial.begin(115200);
}

void loop() {
  // pid_test
  // m.update(100);
  // Serial.print("error: ");
  // Serial.print(m.getError());
  // Serial.print(", vel: ");
  // Serial.print(m.getVel());
  // Serial.print(", dir: ");
  // Serial.print(m.getDir());
  // Serial.print(", duty: ");
  // Serial.println(m.getDuty());

  // motor_test
  m.move_motor(0.2);
  delay(5000);

  m.move_motor(0);
  delay(1000);

  m.move_motor(-0.2);
  delay(5000);

  m.move_motor(0);
  delay(1000);
}
