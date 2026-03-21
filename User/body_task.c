#include "body_task.h"
#include "vt.h"
#include "arm_math.h"
#include "pid.h"
#include "motors.h"

/*
/2   1\
   ^
\3   4/
*/

const float M3508_Reduction_Ratio = 19.0f;
float chassis_speed_level = M3508_Reduction_Ratio * 20.0f;
float chassis_rotate_level = M3508_Reduction_Ratio * 10.0f;

/*
 * 以125Hz频率执行此函数（与底盘电机回传频率保持一致）
 */
void Body_Task(void)
{
   int16_t current[4]={0,0,0,0};

   if (vt.CNS != MODE_C) {
      float x = vt.LX;
      float y = vt.LY;
      float z = 0.0f;

      float r;
      arm_sqrt_f32(x*x + y*y, &r); // |r|

      if (r > 0.0f) {
         float sina = y/r;
         float cosa = x/r;

         float sinY, cosY;
         arm_sin_cos_f32(Yaw6020_Angle * _180_over_pi_, &sinY, &cosY);

         if (r > 1.0f) { r = 1.0f; } //避免斜着走比直着走快

         y = r * (sina * cosY + cosa * sinY); // sin(a+Y)
         x = r * (cosa * cosY - sina * sinY); // cos(a+Y)
      }

      if (vt.CNS == MODE_N) {
         z = Chas_Calc_Z(Yaw6020_Angle);
      }else {
         //右侧拨杆控制小陀螺模式的底盘速度 注意z的初始值为0  UP(+3) MID(0) DOWN(-3)
         z = 0.0f;
      }

      //设置速度等级
      x *= chassis_speed_level;
      y *= chassis_speed_level;
      z *= chassis_rotate_level/(1.0f+r); //防止电机转速不够, 导致告诉旋转时无法平移

      //速度分配
      float velocity[4];
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
