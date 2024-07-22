#include "ChildMotor.h"

void ChildMotor::attach_motor(int motor_pin_1, uint8_t motor_channel_1, int motor_pin_2, uint8_t motor_channel_2, uint32_t freq, uint8_t resolution_bites, float max_v_mm_sec, float min_v_mm_sec){
    ledcSetup(motor_channel_1, freq, resolution_bites);
    ledcAttachPin(motor_pin_1, motor_channel_1);
    ledcSetup(motor_channel_2, freq, resolution_bites);
    ledcAttachPin(motor_pin_2, motor_channel_2);
    this->motor_channel_1 = motor_channel_1;
    this->motor_channel_2 = motor_channel_2;
    this->max_v_mm_sec = max_v_mm_sec;
    this->min_v_mm_sec = min_v_mm_sec;
    this->ave_v_mm_sec = (max_v_mm_sec + max_v_mm_sec) / 2;
    this->max_duty = pow(2.0, (float)resolution_bites);
}

void ChildMotor::attach_encoder(int rotary_pin_1, int rotary_pin_2, int interval, int steps_per_click, int wheel_rad_mm, int gear_ratio){
    r.begin(rotary_pin_1, rotary_pin_2, steps_per_click, wheel_rad_mm, gear_ratio, interval);
}

void ChildMotor::attach_control(float p_coef, float i_coef, float offset_ratio){
    this->p_coef = p_coef;
    this->i_coef = i_coef;
    this->offset_duty = max_duty * offset_ratio;
}

float error;
float error_i;
int ChildMotor::getDuty(){ return duty_now; }
int ChildMotor::getDir(){ return dir; }
float ChildMotor::getError(){ return error; }
float ChildMotor::getMaxV(){ return max_v_mm_sec; }
float ChildMotor::getMinV(){ return min_v_mm_sec; }
float ChildMotor::getVel(){  return r.getVel_abs() * dir; }

void ChildMotor::move_motor(float input){
    int duty = offset_duty + (max_duty - offset_duty) * abs(input);
    this->duty_now = duty;
    if (input > 0){
        ledcWrite(motor_channel_1, duty);
        ledcWrite(motor_channel_2, 0);
        dir = 1;
    }else if (input < 0){
        ledcWrite(motor_channel_1, 0);
        ledcWrite(motor_channel_2, duty);
        dir = -1;
    }else{
        ledcWrite(motor_channel_1, 0);
        ledcWrite(motor_channel_2, 0);
    }
}

void ChildMotor::velWrite(float targ_vel_mm_sec){
    error = targ_vel_mm_sec - getVel();
    error_i += error;

    float input = error*p_coef + error_i*i_coef;
    if (input > 1){ input = 1; 
    }else if (input < -1 ){ input = -1; }
    
    move_motor(input);
}

void ChildMotor::velWrite_ratio(float input){}

void ChildMotor::update_rotary(){
    r.update();
}

void ChildMotor::update(float targ_vel_mm_sec){
    r.update();
    velWrite(targ_vel_mm_sec);
}
