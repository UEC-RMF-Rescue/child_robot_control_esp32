#include <Arduino.h>
#include "ESPRotary.h"

#define VCC 4
#define MIN1 12
#define MIN2 13

#define ROA 19
#define ROB 20

#define INTERVAL 100

#define STEP 12
#define WHEEL_RAD (29.0) // mm
#define GEAR_RATIO 315

#define OFFSET 40

int previousMillis = 0;

int encode_v = 0;
int duty = 0;

// float p_coef = 0.65;
float p_coef = 0.004;
float i_coef = 0.00003;

ESPRotary r;

void rotary(ESPRotary& r);
void check(ESPRotary& r);

void setup() {
  pinMode(VCC, OUTPUT);
  
  ledcSetup(0, 5000, 8);
  ledcAttachPin(MIN1, 0);
  ledcSetup(1, 5000, 8);
  ledcAttachPin(MIN2, 1);

  digitalWrite(VCC, HIGH);
  
  Serial.begin(115200);

  r.begin(ROA, ROB, 1, -1000, 1000, 0, 1);
  r.setChangedHandler(rotary);
  // r.setLeftRotationHandler(check);
  // r.setRightRotationHandler(check);
  // r.setIncrement(1);

  Serial.println("Serail start");
}

static float vel = 0;
float encoder2vel(int pulse_count) {
  float vel_now = ( (float)pulse_count / (float)STEP ) * 2.0 * (float)WHEEL_RAD * 3.14 * (1000.0 / (float)INTERVAL) / (float)GEAR_RATIO ; // mm/sec
  vel = vel * 0 + vel_now * 1;
  return vel;
}

float targ_v = 15; // mm/sec
float error = 0;
float error_i = 0;

void loop() {
  r.loop();
  
  error = targ_v - encoder2vel(encode_v);
  error_i += error;
  duty = error*p_coef + error_i*i_coef;

  if (duty > 0){
    if(duty > 255 - OFFSET) duty = 255 - OFFSET;
    ledcWrite(0, 0);
    ledcWrite(1, duty + OFFSET);
  }else if(duty < 0){
    if (duty < -255 + OFFSET) duty = -255 + OFFSET;
    ledcWrite(0, -duty + OFFSET);
    ledcWrite(1, 0);
  }
  else{
    ledcWrite(0,0);
    ledcWrite(1,0);
  }
  
  // Serial.print("encode_v: ");
  // Serial.print(encode_v);
  // Serial.print(", v: ");
  // Serial.print(encoder2vel(encode_v));
  // Serial.print(", error: ");
  // Serial.print(error);
  // Serial.print(", duty: ");
  // Serial.println(duty);
}

int dir = 1;
int i = 0;
void rotary(ESPRotary& r){
  // Serial.print("real value: ");
  // Serial.print(i);
  i++;
  // Serial.print("encoder value: ");
  // Serial.println(r.getPosition());
  int currentMillis = millis();
  if(currentMillis - previousMillis > INTERVAL){
    previousMillis = currentMillis;
    // encode_v = r.getPosition();
    encode_v = i * dir;
    i = 0;
    // Serial.println(encode_v);

    // r.resetPosition();
    // Serial.println(r.getPosition());
  }
}

void check(ESPRotary& r){
  if (r.directionToString(r.getDirection()) == "right") {dir = 1;
  }else{dir = -1;}
}
