#include "neck_task.h"
#include "vt.h"
#include "imu.h"
#include "pid.h"
#include "motors.h"

//遥控数据
const float rc_x_sensitivity = PI/500.0f;

/*
 * 1000Hz执行此任务
 * 云台Yaw轴控制
 */
void Neck_Task(void)
{
    static float RC_YAW = 0;
    int16_t voltage = 0;

    if (vt.CNS != MODE_C) {
        float angle_diff = RC_YAW - imu.Yaw_Angle;

        if (angle_diff > -PI/2 && angle_diff < PI/2) { RC_YAW -= rc_x_sensitivity * vt.RX; } //注意方向
        if (angle_diff > +PI) { RC_YAW -= 2*PI; }
        if (angle_diff < -PI) { RC_YAW += 2*PI; }

        voltage = Yaw6020_PID(RC_YAW, imu.Yaw_Angle, imu.Yaw_Velocity, 0);
    }
    else {
        RC_YAW = imu.Yaw_Angle;
        voltage = 0;
    }
    Neck_GM6020_Tx(voltage);
}
