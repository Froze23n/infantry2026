#ifndef VT_H
#define VT_H

#include "main.h"

#pragma pack(push, 1)
typedef struct
{
    uint8_t sof_1; //0xA9
    uint8_t sof_2; //0x53

    union {
        uint64_t raw;
        struct {
            uint64_t ch_0       : 11;
            uint64_t ch_1       : 11;
            uint64_t ch_2       : 11;
            uint64_t ch_3       : 11;
            uint64_t mode_sw    : 2;
            uint64_t pause      : 1;
            uint64_t fn_1       : 1;
            uint64_t fn_2       : 1;
            uint64_t wheel      : 11;
            uint64_t trigger    : 1;
            uint64_t _unused    : 3;
        } bit;
    } rc;

    union {
        uint8_t raw[7];
        struct {
            int16_t mouse_x;
            int16_t mouse_y;
            int16_t mouse_z;

            uint8_t mouse_left   : 2;
            uint8_t mouse_right  : 2;
            uint8_t mouse_middle : 2;
            uint8_t _unused      : 2;
        } bit;
    } mouse;

    union {
        uint16_t raw;
        struct {
            uint16_t W      : 1;
            uint16_t S      : 1;
            uint16_t A      : 1;
            uint16_t D      : 1;
            uint16_t SHIFT  : 1;
            uint16_t CTRL   : 1;
            uint16_t Q      : 1;
            uint16_t E      : 1;
            uint16_t R      : 1;
            uint16_t F      : 1;
            uint16_t G      : 1;
            uint16_t Z      : 1;
            uint16_t X      : 1;
            uint16_t C      : 1;
            uint16_t V      : 1;
            uint16_t B      : 1;
        } bit;
    } keyboard;

    uint16_t crc16;

} VT_Type;
#pragma pack(pop)

/* ------------------------------ Function Declaration (used in other .c files) ------------------------------ */
void VT_Init(void); //放在main
void VT_IRQHandler(void); //放在中断处理函数中

#endif
