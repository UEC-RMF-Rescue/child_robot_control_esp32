#include <Arduino.h>
#include <QRotary.h>

#define VCC 4
#define MIN1 12
#define MIN2 13

#define ROA 19
#define ROB 20

#define INTERVAL 300

#define STEP 12
#define WHEEL_RAD (29.0) // mm
#define GEAR_RATIO 315

#define OFFSET 40

QRotary m;

void setup() {
  m.begin(ROA, ROB, STEP, WHEEL_RAD, GEAR_RATIO, INTERVAL);

  pinMode(VCC, OUTPUT);
  
  ledcSetup(0, 5000, 8);
  ledcAttachPin(MIN1, 0);
  ledcSetup(1, 5000, 8);
  ledcAttachPin(MIN2, 1);

  digitalWrite(VCC, HIGH);
  
  Serial.begin(115200);
  Serial.println("Serail start");
}

void loop(){
  ledcWrite(0, 60);
  ledcWrite(1, 0);

  m.update();
  Serial.println(m.getValue());
  delay(10);
}
