#ifndef ChildMotor_h
#define ChildMotor_h

#include <Arduino.h>
#include "../QRotary/srcQRotary.h"
#include <math.h>

class ChildMotor
{
public:
    // setup
    void attach_motor(int motor_pin_1, uint8_t motor_channel_1, int motor_pin_2, uint8_t motor_channel_2, uint32_t fleq = 5000, uint8_t resolution_bites = 8, float max_v_mm_sec = 550, float min_v_mm_sec = 15);
    void attach_encoder(void (*callback)(), int* rotary_count, int rotary_pin_1, int rotary_pin_2, int interval, int steps_per_click = 12, int wheel_rad_mm = 29, int gear_ratio = 315);
    void attach_control(float p_coef, float i_coef, float offset_ratio = 0.1);

    // checking private variables
    int getDuty();
    int getDir();
    float getError();
    float getMaxV();
    float getMinV();
    float getVel();

    int getCount(){return r.getCount();};
    int getValue(){return r.getValue();};

    // move motor
    void move_motor(float input); // ratio: -1 - 1
    void velWrite(float targ_vel_mm_sec);
    void velWrite_ratio(float input); // ratio: -1 - 1

    void update_rotary();
    void update(float targ_vel_mm_sec);
    
private:
    QRotary r;
    int rotary_count;
    void rotary_callback(){ rotary_count++; };

    uint8_t motor_channel_1;
    uint8_t motor_channel_2;

    float max_v_mm_sec;
    float min_v_mm_sec;
    float ave_v_mm_sec;

    float error;
    float error_i;

    int max_duty;

    float p_coef;
    float i_coef;
    int offset_duty;

    int dir = 0;
    int duty_now;
};

#endif
