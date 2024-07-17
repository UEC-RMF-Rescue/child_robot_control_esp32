#include <Arduino.h>

#define VCC 4
#define MIN1 12
#define MIN2 13

#define ROA 19
#define ROB 20

#define INTERVAL 200

#define STEP 12
#define WHEEL_RAD (29.0) // mm
#define GEAR_RATIO 315

#define OFFSET 40

long previousMillis = 0;
long currentMillis = 0;

int encoder_read = 0;
int encoder_v = 0;

void updateEncoder();
void InitEncoder(){
  attachInterrupt(digitalPinToInterrupt(ROA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROB), updateEncoder, CHANGE);
}

void setup() {
  InitEncoder();

  pinMode(VCC, OUTPUT);
  
  ledcSetup(0, 5000, 8);
  ledcAttachPin(MIN1, 0);
  ledcSetup(1, 5000, 8);
  ledcAttachPin(MIN2, 1);

  digitalWrite(VCC, HIGH);
  
  Serial.begin(115200);
  Serial.println("Serail start");

  encoder_read = 0;
  encoder_v = 0;
  previousMillis = millis();
}

static float vel = 0;
float encoder2vel(int pulse_count) {
  float vel_now = ( (float)pulse_count / (float)STEP ) * 2.0 * (float)WHEEL_RAD * 3.14 * (1000.0 / (float)INTERVAL) / (float)GEAR_RATIO ; // mm/sec
  vel = vel * 0 + vel_now * 1;
  return vel;
}

void loop() {
  ledcWrite(0, 0);
  ledcWrite(1, 100);
  delay(10);
  Serial.println(encoder_v);
}

void updateEncoder(){
  encoder_read++;
  currentMillis = millis();
  if (currentMillis - previousMillis >= INTERVAL){
    previousMillis = currentMillis;
    encoder_v = encoder_read;
    encoder_read = 0;
  }
}
