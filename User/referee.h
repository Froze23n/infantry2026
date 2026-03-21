#ifndef REFEREE_H
#define REFEREE_H

#include "main.h"

/*------------------------------命令码、数据长度宏定义------------------------------*/
typedef enum
{
    ID_game_status = 0x0001,                // 比赛状态 1Hz
    ID_game_result = 0x0002,		        // 比赛结果 触发
    ID_game_robot_HP = 0x0003,	            // 机器人血量 3Hz

    ID_event_data = 0x0101,		            // 场地事件 1Hz
    ID_referee_warning = 0x104,             // 裁判警告数据 触发+1Hz
    ID_dart_info = 0x105,                   // 飞镖发射相关数据 1Hz

    ID_robot_status = 0x0201,	            // 机器人性能体系数据 10Hz
    ID_power_heat_data = 0x0202,	        // 底盘缓冲能量和射击热量 10Hz
    ID_robot_pos = 0x0203,	                // 机器人位置 1Hz
    ID_buff = 0x0204,			            // 机器人增益和底盘能量 3Hz
    ID_hurt_data = 0x0206,	                // 伤害状态 触发
    ID_shoot_data = 0x0207,		            // 实时射击 触发
    ID_projectile_allowance = 0x208,        // 允许发弹量 10Hz
    ID_rfid_status = 0x209,                 // 机器人RFID模块状态 3Hz
    //0x20A - 0x20E (给飞镖/哨兵/雷达)

    ID_robot_interaction_data = 0x0301,     // 机器人间交互, 发送方触发发送, 频率上限为30Hz
    //0x302 (图传链路)
    //0x303 选手端小地图交互数据 选手端->机器人
    //0x305 (雷达相关)
    //0x306 (自定义控制器)
    //0x307 (哨兵相关)
    //0x308 选手端小地图交互数据 机器人->选手端
    //0x309-0x311 (图传链路)
    //0xA01-0xA06 (雷达无线电路)
} Referee_CMD_ID_ENUM;

#pragma pack(push, 1)
/*------------------------------结构体定义------------------------------*/
typedef struct
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
}game_status_t; // 0x0001

typedef struct
{
    uint8_t winner;
}game_result_t; // 0x0002

typedef struct
{
    uint16_t ally_1_robot_HP;
    uint16_t ally_2_robot_HP;
    uint16_t ally_3_robot_HP;
    uint16_t ally_4_robot_HP;
    uint16_t reserved;
    uint16_t ally_7_robot_HP;
    uint16_t ally_outpost_HP;
    uint16_t ally_base_HP;
} game_robot_HP_t; // 0x0003

/*------------------------------结构体定义------------------------------*/
typedef struct
{
    uint32_t event_data;
}event_data_t; // 0x0101

typedef struct
{
    uint8_t level;
    uint8_t offending_robot_id;
    uint8_t count;
}referee_warning_t; // 0x0104

typedef struct
{
    uint8_t dart_remaining_time;
    uint16_t dart_info;
}dart_info_t; // 0x0105

/*------------------------------结构体定义------------------------------*/
typedef struct
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
}robot_status_t; // 0x0201

typedef struct
{
    uint8_t reserved[8];
    uint16_t buffer_energy;
    uint16_t shooter_17mm_1_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
}power_heat_data_t; // 0x0202

typedef struct
{
    float x;
    float y;
    float angle;
}robot_pos_t; // 0x0203

typedef struct
{
    uint8_t recovery_buff;
    uint16_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;
    uint8_t remaining_energy;
}buff_t; // 0x0204

typedef struct
{
    uint8_t armor_id : 4;
    uint8_t HP_deduction_reason : 4;
}hurt_data_t; // 0x0206

typedef struct
{
    uint8_t bullet_type;
    uint8_t shooter_number;
    uint8_t launching_frequency;
    float initial_speed;
}shoot_data_t; // 0x0207

typedef struct
{
    uint16_t projectile_allowance_17mm;
    uint16_t projectile_allowance_42mm;
    uint16_t remaining_gold_coin;
    uint16_t projectile_allowance_fortress;
}projectile_allowance_t; // 0x0208

typedef struct
{
    uint32_t rfid_status;
    uint8_t rfid_status_2;
}rfid_status_t; // 0x0209

typedef struct
{
    uint8_t figure_name[3];
    
    uint32_t operate_tpye:3;
    uint32_t figure_tpye:3;
    uint32_t layer:4;
    uint32_t color:4;
    uint32_t details_a:9;
    uint32_t details_b:9;

    uint32_t width:10;
    uint32_t start_x:11;
    uint32_t start_y:11;

    uint32_t details_c:10;
    uint32_t details_d:11;
    uint32_t details_e:11;
}interaction_figure_t;

#pragma pack(pop)
/*------------------------------总体结构体定义------------------------------*/
typedef struct {
    game_status_t   game_status;
    game_result_t   game_result;
    game_robot_HP_t game_robot_hp;

    event_data_t        event_data;
    referee_warning_t   referee_warning;
    dart_info_t         dart_info;

    robot_status_t          robot_status;
    power_heat_data_t       power_heat_data;
    robot_pos_t             robot_pos;
    buff_t                  buff;
    hurt_data_t             hurt_data;
    shoot_data_t            shoot_data;
    projectile_allowance_t  projectile_allowance;
    rfid_status_t           rfid_status;
} Referee_Type;

/*  ------------------------------Extern Global Variable------------------------------  */
extern Referee_Type referee;

/*  ------------------------------Function Declarations------------------------------  */
void Referee_Init(void);
void Referee_IRQHandler(void);

void Referee_UI_Update(void);

#endif //REFEREE_H
