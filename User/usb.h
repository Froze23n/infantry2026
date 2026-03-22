#ifndef USB_H
#define USB_H

#include "main.h"

#pragma pack(push, 1)

typedef struct{
    //head
    uint8_t Ox55;
    uint8_t length;
    uint8_t Ox90;
    uint8_t Ox88;
    uint8_t CRC8;
    uint8_t Ox01;
    uint8_t Ox00;

    //data
    uint8_t OxAA;
    float pitch;
    float yaw;
    float yaw_speed;
    int32_t canshoot;
    int32_t move_state;
    int32_t ifget;
    uint8_t enemy_kind;
    int32_t enemy_x;
    int32_t enemy_y;
    uint8_t OxA5;
    uint16_t CRC16;
} Vision_RX_Type;

typedef struct{
    uint8_t OxAA;
    uint8_t enemy_color;
    float car_yaw;
    float car_pitch;
    uint8_t grade;
    uint16_t CRC16;
} Vision_TX_Type;

#pragma pack(pop)


void USB_TX(void);
void USB_RX(void);

#endif
