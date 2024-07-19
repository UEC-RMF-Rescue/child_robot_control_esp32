#include "QRotary.h"

int rotary_count = 0;
void updateEncoder(){ rotary_count++; }

// initialize encoder
void QRotary::begin(int rotary_pin_1, int rotary_pin_2, int steps_per_click, int wheel_rad_mm, int gear_ratio, int interval){
    setRotaryPin(rotary_pin_1, rotary_pin_2);
    setStepsPerClick(steps_per_click);
    setWheel(wheel_rad_mm, gear_ratio);
    setInterval(interval);
}

void QRotary::setRotaryPin(int rotary_pin_1, int rotary_pin_2){
    attachInterrupt(digitalPinToInterrupt(rotary_pin_1), updateEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(rotary_pin_2), updateEncoder, CHANGE);
}

void QRotary::setStepsPerClick(int steps_per_click){ this->steps_per_click = steps_per_click; }

void QRotary::setWheel(int wheel_rad_mm, int gear_ratio){
    this->wheel_rad_mm = wheel_rad_mm;
    this->gear_ratio = gear_ratio;
}

void QRotary::setInterval(int interval){ this->interval = interval; }

// get information
int QRotary::getCount(){ return rotary_count; }

int QRotary::getValue(){ return encoder_value; }

float QRotary::getVel_abs(){
    return ( (float)encoder_value / steps_per_click / gear_ratio ) *  ( 2.0 * wheel_rad_mm * 3.14 ) * ( 1000.0 / interval );
}

float QRotary::getRPM(){
    return ( (float)encoder_value / steps_per_click / gear_ratio ) * ( 1000.0 / interval ) * 60;
}

// update information
void QRotary::update(){
    currentMillis = millis();
    if (currentMillis - previousMillis > interval){
        previousMillis = currentMillis;
        encoder_value = getCount();
        rotary_count = 0;
    }
}
