#ifndef chidrobot_h
#define chidrobot_h

#include <Arduino.h>
#include <array>
#include <cmath>
#include "ChildMotor.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
// #include <pinassign.h>

class ChildRobot
{
public:
    void set_targ(float targ_rx, float targ_ry, float targ_theta);
    void set_control(float p_coef_dist, float d_coef_dist, float p_coef_theta, float d_coef_theta);
    int bno_begin(int rx, int tx, int interval);
    void set_motor(ChildMotor m1, ChildMotor m2, ChildMotor m3, ChildMotor m4);
    void set_body(int rad = 63);

    void reset_bno(float yaw = 0);
    void reset_dist(float x, float y);

    std::array<float, 2> getCurrent();
    float getAngle();
    std::array<float, 3> getError();
    std::array<float, 4> getVelOrder();
    std::array<float, 2> getVelPrev();

    void update_vel();
    void update_dist();
    void update_bno();
    void update();
    void update_state();

private:
    // rx, ry, theta
    std::array<float, 3> targ = {0};
    std::array<float, 3> current;
    std::array<float, 2> prev = {0};

    std::array<float, 3> error;
    std::array<float, 3> error_prev = {0};
    std::array<float, 3> error_d;

    // vx, vy
    std::array<float, 2> vel_prev = {0};
    // v1,v2,v3,v4
    std::array<float, 4> vel_order;

    // body_info
    std::array<ChildMotor, 4> motors;
    int rad = 63;

    float p_coef_dist;
    float d_coef_dist;
    float p_coef_theta;
    float d_coef_theta;

    long previousMillis = 0;
    long currentMillis;
    int interval;

    Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);
    float offset = 0;
    // sensors_event_t yaw;

    double cos_calc();
    double sin_calc();

    std::array<float, 4> vel_divide(float vx, float vy, float vtheta);
    std::array<float, 2> vel_merge(float v1, float v2, float v3);

};

#endif
