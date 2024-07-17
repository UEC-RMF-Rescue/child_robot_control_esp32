#ifndef childmotor_h
#define childmotor_h
#include <Arduino.h>
#include <iostream>

class ChildMotor
{
public:
    // setup
    void init(int rotary_pin_1, int rotary_pin_2, int steps_per_click);
    void setWheel(int wheel_rad_mm, int gear_ratio);
    void setPI(float p_coef, float i_coef, int offset, int interval);
    void setInterval(int interval);

    // functions for checking the data
    int getCount();
    float getVel(); // mm/sec
    float getRPM();

    // functions to use in main loop
    void updateCount();
    // void updateDuty(int* duty);

private:
    int encoder_value = 0;

    int rotary_pin_1;
    int rotary_pin_2;
    int offset;
    int steps_per_click;

    int wheel_rad_mm; // mm
    int gear_ratio;

    float p_coef;
    float i_coef;
    int interval;

    long previousMillis = 0;
    long currentMillis;
};

#endif
