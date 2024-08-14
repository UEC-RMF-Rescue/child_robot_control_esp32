#include <Arduino.h>
#include "pinassign.h"

void setup(){
  pinMode(LED_G, OUTPUT);
  pinMode(LED_R, OUTPUT);
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_R, LOW);
  Serial.begin(115200);
}

void loop() {
  if(Serial.available() > 0){
    String data = Serial.readStringUntil('\n');

    std::array<int, 3> idx;
    idx[0] = data.indexOf(',');
    idx[1] = data.indexOf(',', idx[0] + 1);
    idx[2] = data.indexOf(',', idx[1] + 1);

    int order = data.substring(0, idx[0]).toInt();
    float x = data.substring(idx[0]+1, idx[1]).toFloat();
    float y = data.substring(idx[1]+1, idx[2]).toFloat();
    float z = data.substring(idx[2]+1        ).toFloat();
    
    if (order == 0){
      digitalWrite(LED_G, HIGH); digitalWrite(LED_R, HIGH);
    }else if (order == 1){
      digitalWrite(LED_G, HIGH); digitalWrite(LED_R, LOW); 
    }else if (order == 2){
      digitalWrite(LED_G, LOW); digitalWrite(LED_R, HIGH);
    }
    
    Serial.println("complete");

  }
}
