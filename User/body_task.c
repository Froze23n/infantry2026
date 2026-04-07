#include "body_task.h" //分配TIM7 125Hz
#include "vt.h"
#include "referee.h"
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

static const float iv = 0.024f;
static const float mv = 0.900f;

static float slopeX(int x){
   static float ret = 0;
   switch(x){
      case +1: ret += iv; break;
      case -1: ret -= iv; break;
      case 0 : ret *= mv; break;
   }
   if(ret > +1.0f){ ret = +1.0f;}
   if(ret < -1.0f){ ret = -1.0f;}
   return ret;
}

static float slopeY(int y){
   static float ret = 0;
   switch(y){
      case +1: ret += iv; break;
      case -1: ret -= iv; break;
      case 0 : ret *= mv; break;
   }
   if(ret > +1.0f){ ret = +1.0f;}
   if(ret < -1.0f){ ret = -1.0f;}
   return ret;
}

/*
 * 以125Hz频率执行此函数（与底盘电机回传频率保持一致）
 */
void Body_Task(void)
{
   int16_t current[4]={0,0,0,0};

   if (vt.CNS != MODE_C) {
      float x = vt.LX + slopeX(vt.keyboard.bit.D - vt.keyboard.bit.A);
      float y = vt.LY + slopeY(vt.keyboard.bit.W - vt.keyboard.bit.S);
      static float z = 0.0f;

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
      }else if(vt.CNS == MODE_S) {
         //右侧拨杆控制小陀螺模式的底盘速度 注意z的初始值为0
         if(vt.keyboard.bit.Q || vt.FN_L){
            z += iv; if(z >= +2.0f){z= +2.0f;}
         }
         if(vt.keyboard.bit.E || vt.FN_R){
            z -= iv; if(z <= -2.0f){z= -2.0f;}
         }
         if(vt.keyboard.bit.R || vt.pause || (referee.robot_status.power_management_chassis_output == 0)){
            z *= mv;
         }
      }

      //设置速度等级
      float X = x * chassis_speed_level;
      float Y = y * chassis_speed_level;
      float Z = z * chassis_rotate_level/(1.0f+r); //防止电机转速不够, 导致告诉旋转时无法平移

      //速度分配
      float velocity[4];
      velocity[0] = -Z - Y + X;
      velocity[1] = -Z + Y + X;
      velocity[2] = -Z + Y - X;
      velocity[3] = -Z - Y - X;

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
