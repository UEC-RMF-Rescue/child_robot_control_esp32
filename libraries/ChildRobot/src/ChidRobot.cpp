#include "ChildRobot.h"

double ChildRobot::cos_calc(){ return std::cos( (current[2] + 45.0) * M_PI / 180.0 ); }
double ChildRobot::sin_calc(){ return std::sin( (current[2] + 45.0) * M_PI / 180.0 ); }

std::array<float, 4> ChildRobot::vel_divide(float vx, float vy, float vtheta){
    double c = cos_calc();
    double s = sin_calc();

    float v1 =  c*vx - s*vy + this->rad*vtheta;
    float v2 =  s*vx + c*vy + this->rad*vtheta;
    float v3 = -c*vx + s*vy + this->rad*vtheta;
    float v4 = -s*vx - c*vy + this->rad*vtheta;
    
    return {v1, v2, v3, v4};
}

std::array<float, 2> ChildRobot::vel_merge(float v1, float v2, float v3){
    double c = cos_calc();
    double s = sin_calc();

    float vx =  (c-s)*0.5*v1 + s*v2 - (c+s)*0.5*v3;
    float vy = -(c+s)*0.5*v1 + c*v2 - (c-s)*0.5*v3;

    return {vx, vy};
}

void ChildRobot::set_targ(float targ_rx, float targ_ry, float targ_theta){
    this->targ[0] = targ_rx;
    this->targ[1] = targ_ry;
    this->targ[2] = targ_theta;
}

void ChildRobot::set_control(float p_coef_dist, float d_coef_dist, float p_coef_theta, float d_coef_theta){
    this-> p_coef_dist = p_coef_dist;
    this-> d_coef_dist = d_coef_dist;
    this-> p_coef_theta = p_coef_theta;
    this-> d_coef_theta = d_coef_theta;
}

int ChildRobot::bno_begin(int rx, int tx, int interval){
    this->interval = interval;
    Wire.begin(rx, tx);
    if(!bno.begin()){return 0;}
    bno.setExtCrystalUse(true);
    update_bno();
    return 1;
}

void ChildRobot::set_motor(ChildMotor m1, ChildMotor m2, ChildMotor m3, ChildMotor m4){
    motors[0] = m1; motors[1] = m2; motors[2] = m3; motors[3] = m4;
}

void ChildRobot::set_body(int rad){ this->rad = rad; }

void ChildRobot::reset_bno(float yaw){
    offset = -yaw;
    // offset = yaw - current[2];
    // current[2] = yaw; prev[2] = yaw;
}

void ChildRobot::reset_dist(float x, float y){
    current[0] = x; current[1] = y;
    prev[0] = x; prev[1] = y;
    targ[0] = current[0]; targ[1] = current[1];
}

std::array<float, 2> ChildRobot::getCurrent(){
    return { current[0], current[1] };
}

float ChildRobot::getAngle(){
    return current[2];
}

std::array<float, 3> ChildRobot::getError(){
    return { error[0], error[1], error[2] };
}

std::array<float, 4> ChildRobot::getVelOrder(){
    return vel_order;
}

std::array<float, 2> ChildRobot::getVelPrev(){
    return vel_prev;
}

void ChildRobot::update_vel(){
    error[0] = targ[0] - current[0];
    error[1] = targ[1] - current[1];
    error[2] = targ[2] - current[2];
    // normalization
    if (error[2] < -180.0 ){ error[2] = 360 + error[2]; }
    // limit
    if (error[0] > 900){ error[0] = 900;
    }else if (error[0] < -900){error[0] = -900;}
    if (error[1] > 900){ error[1] = 900;
    }else if (error[1] < -900){error[1] = -900;}

    if (abs(error[0]) < 30) error[0] = 0;
    if (abs(error[1]) < 30) error[1] = 0;
    
    if (abs(error[2]) < 10) { error[2] = 0; 
    }else if (abs(error[2] > 25)){
        error[0] = 0; error[1] = 0; 
    }
    
    error_d[0] = error[0] - error_prev[0];
    error_d[1] = error[1] - error_prev[1];
    error_d[2] = error[2] - error_prev[2];

    float vx = p_coef_dist*error[0] + d_coef_dist*error_d[0];
    float vy = p_coef_dist*error[1] + d_coef_dist*error_d[1];
    float vtheta = p_coef_theta*error[2] + d_coef_theta*error_d[2];

    if (vx*vx + vy*vy > 62500){
        vx *= 250/std::sqrt(vx*vx + vy*vy);
        vy *= 250/std::sqrt(vx*vx + vy*vy);
    }

    std::array<float, 4> vel = vel_divide(vx, vy, vtheta);
    motors[0].update(vel[0]);
    motors[1].update(vel[1]);
    motors[2].update(vel[2]);
    motors[3].update(vel[3]);
    
    for (int i=0; i<3; i++){ error_prev[i] = error[i] ;}
    for (int i=0; i<4; i++){ vel_order[i] = vel[i]; }
}

void ChildRobot::update_dist(){
    float v1 = motors[0].getVel();
    float v2 = motors[1].getVel();
    float v3 = motors[2].getVel();
    std::array<float, 2> vel = vel_merge(v1, v2, v3);

    current[0] += (vel_prev[0] + vel[0]) * interval / 1000.0 *0.5;
    current[1] += (vel_prev[1] + vel[1]) * interval / 1000.0 *0.5;
    vel_prev[0] = vel[0];
    vel_prev[1] = vel[1];
}

void ChildRobot::update_bno(){
    sensors_event_t yaw;
    bno.getEvent(&yaw, Adafruit_BNO055::VECTOR_EULER);
    current[2] = (float)yaw.orientation.x + offset;
    if ( current[2] < 0.0 ){ current[2] = 360.0 + current[2]; 
    }else if ( current[2] > 360.0 ){ current[2] = current[2] - 360; }
}

void ChildRobot::update(){
    update_vel();
    currentMillis = millis();
    if (currentMillis - previousMillis > interval){
        previousMillis = currentMillis;
        update_dist();
        update_bno();
    }
}

void ChildRobot::stop(){
    motors[0].move_motor(0);
    motors[1].move_motor(0);
    motors[2].move_motor(0);
    motors[3].move_motor(0);
}

void ChildRobot::update_state(){
    currentMillis = millis();
    if (currentMillis - previousMillis > interval){
        previousMillis = currentMillis;
        update_dist();
        update_bno();
    }
}
