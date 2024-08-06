#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <pinassign.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

void setup() {
  Serial.begin(115200);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  Wire.begin(RX, TX);

  if(!bno.begin()){
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }
  delay(1000);

  bno.setExtCrystalUse(true);
  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
}

// void printEvent(sensors_event_t* event);

void loop() {
  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t orientationData;
  //  , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  // bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  // bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  // bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  // bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  // bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  Serial.print("yaw angle: ");
  Serial.println(orientationData.orientation.x);

  // printEvent(&orientationData);
  // printEvent(&angVelocityData);
  // printEvent(&linearAccelData);
  // printEvent(&magnetometerData);
  // printEvent(&accelerometerData);
  // printEvent(&gravityData);

  // int8_t boardTemp = bno.getTemp();
  // Serial.println();
  // Serial.print(F("temperature: "));
  // Serial.println(boardTemp);

  // uint8_t system, gyro, accel, mag = 0;
  // bno.getCalibration(&system, &gyro, &accel, &mag);
  // Serial.println();
  // Serial.print("Calibration: Sys=");
  // Serial.print(system);
  // Serial.print(" Gyro=");
  // Serial.print(gyro);
  // Serial.print(" Accel=");
  // Serial.print(accel);
  // Serial.print(" Mag=");
  // Serial.println(mag);

  // Serial.println("--");
  delay(BNO055_SAMPLERATE_DELAY_MS);
}

// void printEvent(sensors_event_t* event) {
//   double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
//   if (event->type == SENSOR_TYPE_ACCELEROMETER) {
//     Serial.print("Accl:");
//     x = event->acceleration.x;
//     y = event->acceleration.y;
//     z = event->acceleration.z;
//   }
//   else if (event->type == SENSOR_TYPE_ORIENTATION) {
//     Serial.print("Orient:");
//     x = event->orientation.x;
//     y = event->orientation.y;
//     z = event->orientation.z;
//   }
//   else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
//     Serial.print("Mag:");
//     x = event->magnetic.x;
//     y = event->magnetic.y;
//     z = event->magnetic.z;
//   }
//   else if (event->type == SENSOR_TYPE_GYROSCOPE) {
//     Serial.print("Gyro:");
//     x = event->gyro.x;
//     y = event->gyro.y;
//     z = event->gyro.z;
//   }
//   else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
//     Serial.print("Rot:");
//     x = event->gyro.x;
//     y = event->gyro.y;
//     z = event->gyro.z;
//   }
//   else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
//     Serial.print("Linear:");
//     x = event->acceleration.x;
//     y = event->acceleration.y;
//     z = event->acceleration.z;
//   }
//   else if (event->type == SENSOR_TYPE_GRAVITY) {
//     Serial.print("Gravity:");
//     x = event->acceleration.x;
//     y = event->acceleration.y;
//     z = event->acceleration.z;
//   }
//   else {
//     Serial.print("Unk:");
//   }

//   Serial.print("\tx= ");
//   Serial.print(x);
//   Serial.print(" |\ty= ");
//   Serial.print(y);
//   Serial.print(" |\tz= ");
//   Serial.println(z);
// }
