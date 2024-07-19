#include <Arduino.h>
#include "QRotary.h"
#include "ChildMotor.h"

#define VCC 4
#define MIN1 12
#define MIN2 13

#define ROA 19
#define ROB 20

#define INTERVAL 100

#define STEP 12
#define WHEEL_RAD (29.0) // mm
#define GEAR_RATIO 315

// when targ = 300
#define P_COEF 0.0008
#define I_COEF 0.000003

// when targ = 100
// #define P_COEF 0.002
// #define I_COEF 0.000002

// #define P_COEF 0.0015
// #define I_COEF 0.000003

ChildMotor m;

void setup() {
  m.attach_motor(MIN1, 0, MIN2, 1, 5000, 8);
  m.attach_encoder(ROA, ROB, STEP, WHEEL_RAD, GEAR_RATIO, INTERVAL);
  m.attach_control(P_COEF, I_COEF);
  pinMode(VCC, OUTPUT);
  digitalWrite(VCC, HIGH);

  Serial.begin(115200);
}

void loop() {
  m.update(20);
  Serial.print("error: ");
  Serial.print(m.getError());
  Serial.print(", vel: ");
  Serial.print(m.getVel());
  Serial.print(", duty: ");
  Serial.println(m.getDuty());
}
