#include "neck_task.h" //分配TIM6 1000Hz
#include "vt.h"
#include "imu.h"
#include "pid.h"
#include "motors.h"

#include "game_task.h"

//遥控数据
const float rc_x_sensitivity = PI/500.0f;

/*
 * 1000Hz执行此任务
 * 云台Yaw轴控制
 */
void Neck_Task(void)
{
    static float RC_Yaw = 0;
    int16_t voltage = 0;

    if (vt.CNS != MODE_C) {
        float angle_diff = RC_Yaw - imu.Yaw_Angle;

        if (angle_diff > -PI/2 && angle_diff < PI/2) { RC_Yaw -= rc_x_sensitivity * (vt.RX + vt.mouse_x); } //注意方向
        if (angle_diff > +PI) { RC_Yaw -= 2*PI; }
        if (angle_diff < -PI) { RC_Yaw += 2*PI; }
        
        if (vt.trigger || vt.mouse_right){
            float direction = (Vision_Yaw_Angle > 0.0f) ? (1.0f) : (-1.0f);
            if(Vision_Yaw_Angle < -8.0f || Vision_Yaw_Angle > 8.0f){
                RC_Yaw += direction * rc_x_sensitivity * 0.16f;
            }else if(Vision_Yaw_Angle < -4.0f || Vision_Yaw_Angle > 4.0f){
                RC_Yaw += direction * rc_x_sensitivity * 0.08f;
            }else if(Vision_Yaw_Angle < -2.0f || Vision_Yaw_Angle > 2.0f){
                RC_Yaw += direction * rc_x_sensitivity * 0.04f;
            }else if(Vision_Yaw_Angle < -1.0f || Vision_Yaw_Angle > 1.0f){
                RC_Yaw += direction * rc_x_sensitivity * 0.02f;
            }else{
                //死区
            }
        }

        voltage = Yaw6020_PID(RC_Yaw, imu.Yaw_Angle, imu.Yaw_Velocity, 0);
    }
    else {
        RC_Yaw = imu.Yaw_Angle;
        voltage = 0;
    }
    Neck_GM6020_Tx(voltage);
}
