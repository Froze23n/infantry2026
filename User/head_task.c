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

/*
 * 以1000Hz频率执行此函数
 * 用于同时控制pitch轴、拨弹盘、摩擦轮
 */
void Head_Task(void)
{
    static float RC_PITCH = 0;
    static int SHOOT_ON = 0;
    float RC_LoadV=0;
    int16_t pitch_voltage=0;
    int16_t shooter_current[2]={0,0};
    int16_t loader_current=0;
    if (rc.sw1 == SW_MID || rc.sw1 == SW_DOWN) {
        if ( (Pitch6020_Angle >= pitch_lookdown_lim) && (rc.RY >= 0.0f) ) { //俯角超出限制 (俯角为+)
            RC_PITCH = imu.Pitch_Angle;
        }else if ( (Pitch6020_Angle <= pitch_lookup_lim) && (rc.RY <= 0.0f) ){ //仰角超出限制 (仰角为-)
            RC_PITCH = imu.Pitch_Angle;
        }else {
            RC_PITCH += rc_y_sensitivity * rc.RY;
        }
        pitch_voltage = Pitch6020_PID(RC_PITCH, imu.Pitch_Angle, imu.Pitch_Velocity, 0);

        if (rc.wheel >= 0.3f) {
            SHOOT_ON = 1;
            RC_LoadV = rc.wheel * loader_speed_level;
        }else if (rc.wheel <= -0.3f) {
            RC_LoadV = -10.0f;
        }else {
            RC_LoadV = 0;
        }
        if (1==SHOOT_ON && rc.wheel <= -1){SHOOT_ON = 0;}

        shooter_current[0]=Shoot3508_PID(0, -500.0f*(float)SHOOT_ON - Shoot3508_Velocity[0]);
        shooter_current[1]=Shoot3508_PID(1, +500.0f*(float)SHOOT_ON - Shoot3508_Velocity[1]);

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

// //测试
// float loadV_kp = 100.0f;
// float loadV_ki = 3.0f;
// float iError = 0;
// static int16_t Load3508_PID(float pError)
// {
//     // static float iError = 0;
//     float output;
//     iError += pError;
//     #define IERROR_LIMIT (2000.0f)
//     if(iError >  IERROR_LIMIT){ iError =  IERROR_LIMIT;}
//     if(iError < -IERROR_LIMIT){ iError = -IERROR_LIMIT;}
//     output = pError*loadV_kp + iError*loadV_ki;
//     if(output > +16384){output = +16384;}
//     if(output < -16384){output = -16384;}
//     return (int16_t)output;
// }
//
// float RC_LoadV=0;
// void test_task(void) {
//     int16_t loader_current=0;
//     static int REVERSE = 0;
//     if (rc.sw1 == SW_MID || rc.sw1 == SW_DOWN) {
//         if (iError > IERROR_LIMIT - 1.0f) {
//             REVERSE = 200;
//         }
//         if (REVERSE > 0) {
//             REVERSE --;
//             RC_LoadV = -15.0f;
//         }else {
//             if (rc.wheel >= 0.3f) {
//                 RC_LoadV = rc.wheel * 15;
//             }else if (rc.wheel <= -0.3f) {
//                 RC_LoadV = -10.0f;
//             }else {
//                 RC_LoadV = 0;
//             }
//         }
//         loader_current= Load3508_PID(RC_LoadV - Load2006_Velocity);
//     }else {
//
//         loader_current = 0;
//     }
//     Head_Motors_Tx(0,0, loader_current);
// }