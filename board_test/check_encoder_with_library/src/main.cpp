#include <Arduino.h>
#include "QRotary.h"
#include "ChildMotor.h"

#define VCC 4

// M1
// #define MIN1 47
// #define MIN2 48
// #define ROA 21
// #define ROB 19

// M2
#define MIN1 40
#define MIN2 41
#define ROA 1
#define ROB 42


#define STEP 12
#define WHEEL_RAD 29
#define GEAR_RATIO 315
#define INTERVAL 100

QRotary r;
ChildMotor m;

void setup() {
  pinMode(VCC, OUTPUT);
  digitalWrite(VCC, HIGH);

  m.attach_motor(MIN1, 0, MIN2, 1);
  // choose one of them
  // r.begin(ROA, ROB, STEP, WHEEL_RAD, GEAR_RATIO, INTERVAL);
  m.attach_encoder(ROA, ROB, INTERVAL);
  Serial.begin(115200);
}

void loop() {
  m.move_motor(0.2);

  // r.update();
  // Serial.print("vel value from QRotary: ");
  // Serial.println(r.getVel_abs());
  
  m.update_rotary();
  Serial.print("vel value from ChildMotor: ");
  Serial.println(m.getVel());
  delay(10);
}
