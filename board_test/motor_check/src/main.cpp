#include <Arduino.h>
#include "pinassign.h"

void setup(){
  ledcSetup(0, 5000, 8);
  ledcAttachPin(M1IN1, 0);

  ledcSetup(1, 5000, 8);
  ledcAttachPin(M1IN2, 1);
}

void stop(){
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  delay(500);
}

void loop(){
  ledcWrite(0, 60);
  ledcWrite(1, 0);

  delay(5000);

  stop();

  ledcWrite(1, 60);
  ledcWrite(0, 0);

  delay(5000);

  stop();
}
