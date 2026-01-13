#ifndef PID_H
#define PID_H

#include "main.h"

typedef struct
{
    float kp;
    float ki;
    float kd;
} PID_T;

int16_t Chas3508_PID(int8_t ID, float expV, float truV);
float Chas_Calc_Z(float relative_angle);
//底盘~云台yaw轴
int16_t Yaw6020_PID(float expA, float truA, float truV, float feed);
float yaw6020_velocity_to_voltage(float expV, float truV);

//云台pitch
int16_t Pitch6020_PID(float expA, float truA, float truV, float feed);
float pitch6020_velocity_to_voltage(float expV, float truV);
//云台拨弹盘,射击
int16_t Load2006_PID(float pError);
int16_t Shoot3508_PID(int8_t ID,float pError);

#endif //PID_H
