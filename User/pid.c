//
// Created by YawFun on 25-12-7.
//

#include "pid.h"
#ifndef PI
#define PI (3.1415927F)
#endif

//'\0'表示不使用
PID_T chasM = {45.0f , 2.0f ,                                              '\0'}; //底盘电机
PID_T chasZ = {4.0f, '\0' ,20.0f}; //底盘旋转

PID_T yawA = {23.0f, '\0' , 200.0f}; //yaw轴角度
PID_T yawV = {6000.0f , 500.0f , '\0'}; //yaw轴速度

PID_T pitchA = {80.0f , '\0' , 250.0f}; //pitch电机
PID_T pitchV = {3000.0f , 500.0f , '\0'}; //pitch电机

PID_T loadV = {25.0f , 0.5f , '\0'}; //拨弹盘
PID_T shootV = {45.0f, 0.2f, '\0'}; //摩擦轮


/*
 * 输入：电机编号，期望速度rad/s，实际速度rad/s
 * 输出：电流±16384 (±20A)
 * 用于麦克纳姆轮/全向轮 PI速度控制
 */
int16_t Chas3508_PID(int8_t ID, float expV, float truV)
{
    static float iError[4] = {0,0,0,0};
    if(ID<0 || ID>3){return 0;}  //防呆保护
    float pError = expV - truV;
    iError[ID] += pError;
    //积分限幅
    #define M3508_I_LIMIT (1000.0f) // M3508_I_LIMIT*ki保持在2000
    if(iError[ID]>= M3508_I_LIMIT){iError[ID] = M3508_I_LIMIT;}
    if(iError[ID]<=-M3508_I_LIMIT){iError[ID] =-M3508_I_LIMIT;}
    //计算电流
    float output = pError * chasM.kp + iError[ID] * chasM.ki ;
    //限制理论输出+-16384(0x4000)对应+-20A
    if(output > +16384.0f){output = +16384.0f;}
    if(output < -16384.0f){output = -16384.0f;}
    //返回结果
    return (int16_t)output;
}

/*
 *输入：云台YAW轴与底盘的角度之差 范围[ -pi , pi )
 *输出：车子应该旋转的角速度rad/s
 *用于底盘自动回正跟随云台。
 */
float Chas_Calc_Z(float relative_angle)
{
    static float oldError = 0;
    float pError,dError;
    float outLevel;
    //calc
    pError = relative_angle;
    dError = pError-oldError;//起步时加速，结束时减速
    oldError = pError;
    //算加和
    outLevel = pError*chasZ.kp + dError*chasZ.kd;
    if(outLevel> 4){outLevel= 4;}
    if(outLevel<-4){outLevel=-4;}
    return outLevel;
}

/*
 *yaw轴双环PID
 *角度 -> 速度 -> 电压
 */
int16_t Yaw6020_PID(float expA, float truA, float truV, float feed)
{
    (void)feed;
    //计算误差
    static float oldError = 0;
    static float dError[2] = {0,0};
    float pError = expA - truA;
    dError[1] = dError[0];dError[0] = pError - oldError;
    oldError = pError;

    //结合前馈，得到预期速度
    float expV = pError * yawA.kp + (dError[0]+dError[1]) * yawA.kd;
    //速度到电压
    float Voltage = yaw6020_velocity_to_voltage(expV, truV);
    return (int16_t)Voltage;
}

float yaw6020_velocity_to_voltage(float expV, float truV)
{
    static float iError = 0;
    float pError = expV - truV;
    float output;
    iError += pError;
    #define YAW6020_V_I_LIMIT (3.5f)
    if(iError > YAW6020_V_I_LIMIT){iError = YAW6020_V_I_LIMIT;}
    if(iError <-YAW6020_V_I_LIMIT){iError =-YAW6020_V_I_LIMIT;}
    //计算加和
    #define YAW6020_V2V 1400.0f
    output = expV * YAW6020_V2V + pError * yawV.kp + iError * yawV.ki;
    //限制理论电压上限+-25000mV
    if(output > +25000.0f){output = +25000.0f;}
    if(output < -25000.0f){output = -25000.0f;}
    return output;
}


/*
 *pitch轴双环PID
 *角度 -> 期望速度 -> 电压
 */
int16_t Pitch6020_PID(float expA, float truA, float truV, float feed)
{
    //角度到速度：
    (void)feed;
    //计算误差
    static float oldError = 0;
    static float dError[2] = {0,0};
    float pError = expA - truA;
    dError[1] = dError[0];dError[0] = pError - oldError;
    oldError = pError;

    //结合前馈，得到预期速度
    float expV = pError * pitchA.kp + (dError[0]+dError[1]) * pitchA.kd;
    //速度到电压
    float Voltage = pitch6020_velocity_to_voltage(expV, truV);
    return (int16_t)Voltage;
}

float pitch6020_velocity_to_voltage(float expV, float truV)
{
    static float iError = 0;
    float pError = expV - truV;
    float output;
    iError += pError;
    #define PITCH6020_I_LIMIT (0.3f)
    if(iError > PITCH6020_I_LIMIT){iError = PITCH6020_I_LIMIT;}
    if(iError <-PITCH6020_I_LIMIT){iError =-PITCH6020_I_LIMIT;}
    //计算加和
    #define PITCH6020_V2V 700.0f
    output = expV * PITCH6020_V2V + pError * pitchV.kp + iError * pitchV.ki;
    //限制理论电压上限+-25000mV
    if(output > +25000.0f){output = +25000.0f;}
    if(output < -25000.0f){output = -25000.0f;}
    return output;
}

/*
 * 拨弹盘速度控制
 */
int16_t Load2006_PID(float pError)
{
    static float iError = 0;
    float output;
    iError += pError;
    #define LOADER_IERROR_LIMIT (300.0f)
    if(iError >  LOADER_IERROR_LIMIT){ iError =  LOADER_IERROR_LIMIT;}
    if(iError < -LOADER_IERROR_LIMIT){ iError = -LOADER_IERROR_LIMIT;}
    output = pError*loadV.kp + iError*loadV.ki;
    if(output > +10000){output = +10000;}
    if(output < -10000){output = -10000;} /* -10000mA  电流  +10000mA*/
    return (int16_t)output;
}

/*
 * 摩擦轮速度控制
 */
int16_t Shoot3508_PID(int8_t ID, float pError)
{
    if((ID!=0) && (ID!=1)){return 0;}
    static float iError[2] = {0,0};
    float output;
    iError[ID] += pError;
    #define SHOOTER_IERROR_LIMIT (300.0f)
    if(iError[ID] >  SHOOTER_IERROR_LIMIT){ iError[ID] =  SHOOTER_IERROR_LIMIT;}
    if(iError[ID] < -SHOOTER_IERROR_LIMIT){ iError[ID] = -SHOOTER_IERROR_LIMIT;}
    output = pError*shootV.kp + iError[ID]*shootV.ki;
    if(output > +16384){output = +16384;}
    if(output < -16384){output = -16384;} /* -20000mV  电流  +20000mV*/
    return (int16_t)output;
}

