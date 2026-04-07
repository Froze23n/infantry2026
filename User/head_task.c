#include "head_task.h" //分配TIM4 1000Hz
#include "vt.h"
#include "imu.h"
#include "pid.h"
#include "motors.h"
#include "referee.h"
#include "game_task.h"

const float rc_y_sensitivity = 1/200.0f;
float loader_speed_level = 250.0f;

/*
 * 以1000Hz频率执行此函数
 * 用于同时控制pitch轴、拨弹盘、摩擦轮
 */
void Head_Task(void)
{
    /* 变量 */
    static float RC_Pitch = 0; //Pitch轴设定角度
    static char Shoot_On = 0; //是否开启摩擦轮
    static float RC_LoadV = 0; //拨弹盘速度

    int16_t pitch_voltage=0; //Pitch轴控制电压
    int16_t shooter_current[2]={0,0}; //摩擦轮控制电流
    static int16_t loader_current=0; //拨弹盘控制电流

    if (vt.CNS != MODE_C) {
        /* Pitch */
        float rcY = (vt.RY + vt.mouse_y);
        if ( (Pitch6020_Angle >= Pitch_Lookdown_Limit) && (rcY >= 0.0f) ) { //俯角超出限制 (下俯为电机正方向)
            RC_Pitch = imu.Pitch_Angle;
        }else if ( (Pitch6020_Angle <= Pitch_Lookup_Limit) && (rcY <= 0.0f) ){ //仰角超出限制 (上仰为电机负方向)
            RC_Pitch = imu.Pitch_Angle;
        }else {
            RC_Pitch += rc_y_sensitivity * rcY;
        }
        /* 自瞄处理 */
        if (vt.trigger || vt.mouse_right){
            float direction = (vision.Pitch_Angle > 0.0f) ? (1.0f) : (-1.0f);

            if(vision.Pitch_Angle < -2.0f || vision.Pitch_Angle > 2.0f){
                RC_Pitch += direction * rc_y_sensitivity * 0.064f;
            }else if(vision.Pitch_Angle < -1.0f || vision.Pitch_Angle > 1.0f){
                RC_Pitch += direction * rc_y_sensitivity * 0.032f;
            }else if(vision.Pitch_Angle < -0.5f || vision.Pitch_Angle > 0.5f){
                RC_Pitch += direction * rc_y_sensitivity * 0.032f;
            }else if(vision.Pitch_Angle < -0.25f || vision.Pitch_Angle > 0.25f){
                RC_Pitch += direction * rc_y_sensitivity * 0.008f;
            }else{
                if(vision.DY > 5.0f || vision.DY < -5.0f){
                    RC_Pitch += direction * rc_y_sensitivity * 0.004f;
                }
            }
        }

        /* 摩擦轮机制 */
        if (vt.pause){
            Shoot_On = 0; //按暂停键停止摩擦轮
        }
        else if (vt.wheel >= 0.1f || vt.mouse_left){
            Shoot_On = 1; //拨轮死区为±0.1
        }

        /* 拨弹盘速度控制 */
        if (vt.wheel >= 0.3f || vt.mouse_left) {
            // if(vt.trigger || vt.mouse_right){ //开启了自瞄
            //     RC_LoadV = (vision.Can_Shoot) ? (loader_speed_level * 2.0f) : 0.0f; //允许射击则速度加倍
            // }else{
                RC_LoadV = loader_speed_level; //操作手操作发射
            // }
        }else if (vt.wheel <= -0.3f || vt.mouse_middle || vt.keyboard.bit.X) {
            RC_LoadV = -50.0f;
        }else {
            RC_LoadV = 0;
        }
        /*防止超热量*/
        if(referee.power_heat_data.shooter_17mm_1_barrel_heat > referee.robot_status.shooter_barrel_heat_limit){
            loader_speed_level = 0.0f;
        }else{
            loader_speed_level = 250.0f;
        }

        /*堵转处理*/
        static int Load_Reserve_Count = 0;
        if(Load2006_Blocked) { Load_Reserve_Count = 100; } //反转100ms
        if (Load_Reserve_Count > 0) {
            Load_Reserve_Count --;
            RC_LoadV = -100.0f; //反转处理
        }

        /* PID 计算 */
        pitch_voltage = Pitch6020_PID(RC_Pitch, imu.Pitch_Angle, imu.Pitch_Velocity, 0);
        shooter_current[0]=Shoot3508_PID(0, -710.0f*(float)Shoot_On - Shoot3508_Velocity[0]);
        shooter_current[1]=Shoot3508_PID(1, +710.0f*(float)Shoot_On - Shoot3508_Velocity[1]);
        loader_current = Load2006_PID(RC_LoadV - Load2006_Velocity);
    }else {
        RC_Pitch = imu.Pitch_Angle; //防止猛抬头
        Shoot_On = 0; //关闭摩擦轮
        pitch_voltage = 0;
        shooter_current[0] = -0;
        shooter_current[1] = +0;
        loader_current = 0;
    }
    Head_Motors_Tx(pitch_voltage,shooter_current, loader_current);
}
