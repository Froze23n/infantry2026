//
// Created by YawFun on 25-12-7.
//

#include "neck_task.h"

#include "dbus.h"
#include "imu.h"
#include "pid.h"
#include "motors.h"

#ifndef PI
#define PI (3.1415927F)
#endif

//遥控数据
const float rc_x_sensitivity = -PI/500.0f;

/*
 * 1000Hz执行此任务
 * 云台Yaw轴控制
 */
void Neck_Task(void)
{
    static float RC_YAW = 0;
    int16_t voltage = 0;

    if (SW_MID == dbus.sw1 || SW_DOWN == dbus.sw1) {
        float angle_diff = RC_YAW - imu.Yaw_Angle;

        if (angle_diff > -PI/2 && angle_diff < PI/2) {RC_YAW += rc_x_sensitivity * dbus.RX;}
        if (angle_diff > +PI) {RC_YAW -= 2*PI;}
        if (angle_diff < -PI) {RC_YAW += 2*PI;}

        voltage = Yaw6020_PID(RC_YAW, imu.Yaw_Angle, imu.Yaw_Velocity, 0);
        // voltage = yaw6020_velocity_to_voltage(dbus.RX*(-5), imu.Yaw_Velocity);
    }
    else {
        RC_YAW = imu.Yaw_Angle;
        voltage = 0;
    }
    Neck_GM6020_Tx(voltage);
}
