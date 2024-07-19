#ifndef QRotary_h
#define QRotary_h
#include <Arduino.h>

class QRotary
{
public:
    // setup
    void begin(int rotary_pin_1, int rotary_pin_2, int steps_per_click, int wheel_rad_mm, int gear_ratio, int interval);
    void setRotaryPin(int rotary_pin_1, int rotary_pin_2);
    void setStepsPerClick(int steps_per_click);
    void setWheel(int wheel_rad_mm, int gear_ratio);
    void setInterval(int interval);

    // functions for checking the data
    int getCount();
    int getValue();
    float getVel_abs(); // mm/sec
    float getRPM();

    // functions to use in main loop
    void update();

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
