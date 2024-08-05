#include "QRotary_dev.h"

// initialize encoder
// for callback, give increment function
void QRotary::begin(void (*callback)(), int* rotary_count, int rotary_pin_1, int rotary_pin_2, int steps_per_click, int wheel_rad_mm, int gear_ratio, int interval){
    setRotaryPin(callback, rotary_count, rotary_pin_1, rotary_pin_2);
    setStepsPerClick(steps_per_click);
    setWheel(wheel_rad_mm, gear_ratio);
    setInterval(interval);
}

void QRotary::setRotaryPin(void (*callback)(), int* rotary_count, int rotary_pin_1, int rotary_pin_2){
    this->rotary_count = rotary_count;
    attachInterrupt(digitalPinToInterrupt(rotary_pin_1), callback, CHANGE);
    attachInterrupt(digitalPinToInterrupt(rotary_pin_2), callback, CHANGE);
}

void QRotary::setStepsPerClick(int steps_per_click){ this->steps_per_click = steps_per_click; }

void QRotary::setWheel(int wheel_rad_mm, int gear_ratio){
    this->wheel_rad_mm = wheel_rad_mm;
    this->gear_ratio = gear_ratio;
}

void QRotary::setInterval(int interval){ this->interval = interval; }

// get information
// int QRotary::getCount(){ return rotary_count; }

int QRotary::getValue(){ return encoder_value; }

float QRotary::getVel_abs(){
    return ( (float)encoder_value / steps_per_click / gear_ratio ) *  ( 2.0 * wheel_rad_mm * 3.14 ) * ( 1000.0 / interval );
}

float QRotary::getRPM(){
    return ( (float)encoder_value / steps_per_click / gear_ratio ) * ( 1000.0 / interval ) * 60;
}

// update information
// give pointor of rotary increment which you set in "begin"
void QRotary::update(){
    currentMillis = millis();
    if (currentMillis - previousMillis > interval){
        previousMillis = currentMillis;
        encoder_value = *rotary_count;
        *rotary_count = 0;
    }
}
