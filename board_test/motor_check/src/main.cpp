#include <Arduino.h>
#include "pinassign_M1.h"

void setup(){
  ledcSetup(0, 5000, 8);
  ledcAttachPin(MIN1, 0);

  ledcSetup(1, 5000, 8);
  ledcAttachPin(MIN2, 1);
}

void stop(){
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  delay(500);
}

void loop(){
  ledcWrite(0, 100);
  ledcWrite(1, 0);

  delay(1000);

  stop();

  ledcWrite(1, 100);
  ledcWrite(0, 0);

  delay(1000);

  stop();
}
