//
// Created by YawFun on 25-12-7.
//

#include "head_task.h"

#include "rc.h"
#include "imu.h"
#include "pid.h"
#include "motors.h"

#ifndef PI
#define PI (3.1415927f)
#endif

const float rc_y_sensitivity = 1/200.0f;
const float loader_speed_level = 100.0f;

void Head_Task(void)
{
    static float RC_PITCH = 0;
    static int SHOOT_ON = 0;
    float RC_LoadV=0;
    int16_t pitch_voltage=0;
    int16_t shooter_current[2]={0,0};
    int16_t loader_current=0;
    if (rc.sw1 == SW_MID || rc.sw1 == SW_DOWN) {
        if ( (Pitch6020_Angle >= pitch_lookdown_lim) && (rc.RY >= 0.0f) ) { //俯角超出限制
            RC_PITCH = imu.Pitch_Angle;
        }else if ( (Pitch6020_Angle <= pitch_lookup_lim) && (rc.RY <= 0.0f) ){ //仰角超出限制
            RC_PITCH = imu.Pitch_Angle;
        }else {
            RC_PITCH += rc_y_sensitivity * rc.RY;
        }
        pitch_voltage = Pitch6020_PID(RC_PITCH, imu.Pitch_Angle, imu.Pitch_Velocity, 0);

        if (rc.wheel >= 0.3f) {
            SHOOT_ON = 1;
            RC_LoadV = rc.wheel * loader_speed_level;
        }else if (rc.wheel <= -0.3f) {
            RC_LoadV = -10;
        }else {
            RC_LoadV = 0;
        }
        if (1==SHOOT_ON && rc.wheel <= -1){SHOOT_ON = 0;}

        shooter_current[0]=Shoot3508_PID(0, -500*(float)SHOOT_ON - Shoot3508_Velocity[0]);
        shooter_current[1]=Shoot3508_PID(1, +500*(float)SHOOT_ON - Shoot3508_Velocity[1]);

        loader_current= Load2006_PID(RC_LoadV - Load2006_Velocity);
    }else {
        RC_PITCH = Pitch6020_Angle;
        pitch_voltage = 0;
        shooter_current[0] = -0;
        shooter_current[1] = +0;
        loader_current = 0;
    }
    Head_Motors_Tx(pitch_voltage,shooter_current, loader_current);
}
