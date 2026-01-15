#include "head_task.h"

#include "dbus.h"
#include "imu.h"
#include "pid.h"
#include "motors.h"

#ifndef PI
#define PI (3.1415927f)
#endif

const float rc_y_sensitivity = 1/200.0f;
const float loader_speed_level = 250.0f;

/*
 * 以1000Hz频率执行此函数
 * 用于同时控制pitch轴、拨弹盘、摩擦轮
 */
void Head_Task(void)
{
    static float RC_PITCH = 0; //Pitch轴设定角度
    static int SHOOT_ON = 0; //是否开启摩擦轮
    static float RC_LoadV = 0; //拨弹盘速度

    int16_t pitch_voltage=0; //Pitch轴控制电压
    int16_t shooter_current[2]={0,0}; //摩擦轮控制电流
    static int16_t loader_current=0; //拨弹盘控制电流

    if (dbus.sw1 == SW_MID || dbus.sw1 == SW_DOWN) {
        if ( (Pitch6020_Angle >= pitch_lookdown_lim) && (dbus.RY >= 0.0f) ) { //俯角超出限制 (下俯为电机正方向)
            RC_PITCH = imu.Pitch_Angle;
        }else if ( (Pitch6020_Angle <= pitch_lookup_lim) && (dbus.RY <= 0.0f) ){ //仰角超出限制 (上仰为电机负方向)
            RC_PITCH = imu.Pitch_Angle;
        }else {
            RC_PITCH += rc_y_sensitivity * dbus.RY;
        }
        pitch_voltage = Pitch6020_PID(RC_PITCH, imu.Pitch_Angle, imu.Pitch_Velocity, 0);

        extern float Load2006_iError;
        static int REVERSE_COUNTER = 0;

        if (Load2006_iError > LOADER_IERROR_LIMIT - 1.0f) { REVERSE_COUNTER = 300; } //反转300ms

        if (dbus.wheel >= 0.3f) {
            SHOOT_ON = 1;
            RC_LoadV = dbus.wheel * loader_speed_level;
        }else if (dbus.wheel <= -0.3f) {
            RC_LoadV = -50.0f;
        }else {
            RC_LoadV = 0;
        }

        //反转处理
        if (REVERSE_COUNTER > 0) {
            REVERSE_COUNTER --;
            RC_LoadV = -100.0f;
        }

        //停止摩擦轮
        if (1==SHOOT_ON && dbus.wheel <= -1){SHOOT_ON = 0;}

        shooter_current[0]=Shoot3508_PID(0, -500.0f*(float)SHOOT_ON - Shoot3508_Velocity[0]);
        shooter_current[1]=Shoot3508_PID(1, +500.0f*(float)SHOOT_ON - Shoot3508_Velocity[1]);

        loader_current = Load2006_PID(RC_LoadV - Load2006_Velocity);
    }else {
        RC_PITCH = Pitch6020_Angle; //防止猛抬头
        SHOOT_ON = 0; //关闭摩擦轮
        pitch_voltage = 0;
        shooter_current[0] = -0;
        shooter_current[1] = +0;
        loader_current = 0;
    }
    Head_Motors_Tx(pitch_voltage,shooter_current, loader_current);
}
