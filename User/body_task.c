//
// Created by YawFun on 25-12-7.
//

#include "body_task.h"

#include "rc.h"
#include "arm_math.h"
#include "pid.h"
#include "motors.h"

#ifndef PI
#define PI (3.1415927F)
#endif

/*
/2   1\
   ^
\3   4/
*/

const float Reduction_Ratio = 19.0f;
const float _180_over_pi_ = 180.0f / PI;
float chassis_speed_level = Reduction_Ratio*10.0f;
float chassis_rotate_level = Reduction_Ratio*5.0f;

/*
 * 以125Hz频率执行此函数（与底盘电机回传频率保持一致）
 */
void Body_Task(void)
{
   float x=0,y=0,z=0;
   float velocity[4]={0,0,0,0};
   int16_t current[4]={0,0,0,0};

   if (SW_MID == rc.sw1 || SW_DOWN == rc.sw1) {
      x = rc.LX;
      y = rc.LY;
      float r;
      arm_sqrt_f32(x*x + y*y, &r); // |r|

      if (r > 0.0f) {
         float sina = y/r, cosa = x/r;
         float sinY, cosY;
         arm_sin_cos_f32(Yaw6020_Angle * _180_over_pi_, &sinY, &cosY);
         if (r > 1.0f) {r = 1.0f;} //避免斜着走比直着走快
         y = r * (sina * cosY + cosa * sinY);
         x = r * (cosa * cosY - sina * sinY);
         // float a = atan2f(y,x);// <r>
         // arm_sin_cos_f32((a+Yaw6020_Angle)*_180_over_pi_, &y, &x);
         // y*=r; x*=r;
      }

      if (SW_MID == rc.sw1) {
         z = Chas_Calc_Z(Yaw6020_Angle);
      }else { //SW_DOWN == rc.sw1
         if (SW_UP == rc.sw2) {
            z = 3.0f;
         }else if (SW_DOWN == rc.sw2) {
            z = -3.0f;
         }else {
            z = 0;
         }
      }
      //设置速度等级
      x *= chassis_speed_level;
      y *= chassis_speed_level;
      z *= chassis_rotate_level/(1+r); //防止电机转速不够
      //速度分配
      velocity[0] = -z - y + x;
      velocity[1] = -z + y + x;
      velocity[2] = -z + y - x;
      velocity[3] = -z - y - x;
      //电流pid计算
      current[0] = Chas3508_PID(0,velocity[0], Chas3508_Velocity[0]);
      current[1] = Chas3508_PID(1,velocity[1], Chas3508_Velocity[1]);
      current[2] = Chas3508_PID(2,velocity[2], Chas3508_Velocity[2]);
      current[3] = Chas3508_PID(3,velocity[3], Chas3508_Velocity[3]);
   }else {
      current[0] = 0; current[1] = 0; current[2] = 0; current[3] = 0;
   }

   //发送数据
   Body_M3508_Tx(current);
}
