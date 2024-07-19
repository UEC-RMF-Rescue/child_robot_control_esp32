#ifndef ChildMotor_h
#define ChildMotor_h

#include <Arduino.h>
#include "QRotary.h"
#include <math.h>

class ChildMotor
{
public:
    // setup
    void attach_motor(int motor_pin_1, uint8_t motor_channel_1, int motor_pin_2, uint8_t motor_channel_2, uint32_t fleq, uint8_t resolution_bites, float max_v_mm_sec = 550, float min_v_mm_sec = 15);
    void attach_encoder(int rotary_pin_1, int rotary_pin_2, int steps_per_click, int wheel_rad_mm, int gear_ratio, int interval);
    void attach_control(float p_coef, float i_coef, float p_inc = -0.0000006, float i_inc = 0.0000000006, float offset_ratio = 0.1);

    // checking private variables
    int getDuty();
    float getError();
    float getMaxV();
    float getMinV();
    float getVel();

    // move motor
    void move_motor(float input); // ratio: -1 - 1
    void velWrite(float targ_vel_mm_sec);
    void velWrite_ratio(float input); // ratio: -1 - 1

    void update(float targ_vel_mm_sec);

private:
    QRotary r;

    uint8_t motor_channel_1;
    uint8_t motor_channel_2;

    float max_v_mm_sec;
    float min_v_mm_sec;
    float ave_v_mm_sec;

    int max_duty;

    float p_coef;
    float i_coef;
    float p_inc;
    float i_inc;
    int offset_duty;

    int dir = 0;
    int duty_now;
};

#endif
