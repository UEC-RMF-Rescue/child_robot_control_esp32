#include "childmotor.h"

int rotary_count = 0;
void updateEncoder(){rotary_count++;}

void ChildMotor::init(int rotary_pin_1, int rotary_pin_2, int steps_per_click){
    attachInterrupt(digitalPinToInterrupt(rotary_pin_1), updateEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(rotary_pin_2), updateEncoder, CHANGE);
    this->steps_per_click = steps_per_click;
}

void ChildMotor::setWheel(int wheel_rad_mm, int gear_ratio){
    this->wheel_rad_mm = wheel_rad_mm;
    this->gear_ratio = gear_ratio;
}

void ChildMotor::setPI(float p_coef, float i_coef, int offset, int interval){
    this->p_coef = p_coef;
    this->i_coef = i_coef;
    this->offset = offset;
    this->interval = interval;
}
void ChildMotor::setInterval(int interval){this->interval = interval;}

int ChildMotor::getCount(){
    return rotary_count;
}

float ChildMotor::getVel(){
    return ( (float)encoder_value / steps_per_click / gear_ratio ) *  ( 2.0 * wheel_rad_mm * 3.14 ) * ( 1000.0 / interval );
}
float ChildMotor::getRPM(){
    return ( (float)encoder_value / steps_per_click / gear_ratio ) * ( 1000.0 / interval ) * 60;
}

void ChildMotor::updateCount(){
    currentMillis = millis();
    if (currentMillis - previousMillis > interval){
        previousMillis = currentMillis;
        encoder_value = getCount();
        rotary_count = 0;
    }
}

// void ChildMotor::updateDuty(int* duty){}
