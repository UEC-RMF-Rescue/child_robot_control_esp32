#include <Arduino.h>
#define LEDG 39
#define LEDR 40

void setup() {
  pinMode(LEDG, OUTPUT);
  pinMode(LEDR, OUTPUT);
}

void loop() {
  digitalWrite(LEDG, HIGH);
  delay(600);
  digitalWrite(LEDG, LOW);
  delay(600);

  digitalWrite(LEDR, HIGH);
  delay(300);
  digitalWrite(LEDR, LOW);
  delay(300);
}
