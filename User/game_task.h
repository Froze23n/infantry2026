#ifndef GAME_TASK_H
#define GAME_TASK_H

#include "main.h"

#pragma pack(push, 1)
typedef struct{
    uint8_t Ox55;
    uint8_t length;
    uint8_t Ox00_1;
    uint8_t Ox00_2;
    uint8_t CRC8;
    uint8_t Ox01;
    uint8_t Ox00_3;

    uint8_t OxAA;
    float Pitch_Angle;
    float Yaw_Angle;
    float Yaw_Speed;
    int16_t Can_Shoot;
    int16_t Move_State;
    int16_t ifget;
    uint8_t enemy_kind;
    int16_t enemy_x; //单位cm
    int16_t enemy_y; //单位cm
    uint8_t OxA5;
    
    uint16_t CRC16;
} Vision_Wire_Type;
#pragma pack(pop)

typedef struct{
    float Tick;
    float Yaw_Angle;
    float Pitch_Angle;
    float DX; //单位cm
    float DY; //单位cm
    int16_t Can_Shoot;
} Vision_Host_Type;

void Game_Start(void);
void Game_Task(void);

void USB_Tx(void);
void USB_RxHandler(uint8_t* Buf, uint32_t *Len);

extern Vision_Host_Type vision;

#endif
