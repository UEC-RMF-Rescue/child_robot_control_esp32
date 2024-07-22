#include <Arduino.h>
#include "QRotary.h"
#include "pinassign.h"

#define STEP 20
#define WHEEL_RAD 29
#define GEAR_RATIO 315
#define INTERVAL 100

QRotary r;

void updateEncoder();

void setup() {
  pinMode(VCC, OUTPUT);
  digitalWrite(VCC, HIGH);

  Serial.begin(115200);
  // r.begin(M1ROA, M1ROB, STEP, WHEEL_RAD, GEAR_RATIO, INTERVAL);

  attachInterrupt(digitalPinToInterrupt(ROA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROB), updateEncoder, CHANGE);

}

void loop() {
  // // r.update();
  // Serial.println(r.getCount());
  delay(10);
}

int i = 0;
void updateEncoder(){
  Serial.println(i);
  i++;
}
