#ifndef GAME_TASK_H
#define GAME_TASK_H

#include "main.h"

void Game_Start(void);
void Game_Task(void);

void USB_Tx(void);
void USB_RxHandler(uint8_t* Buf, uint32_t *Len);

extern float Vision_Pitch_Angle;
extern float Vision_Yaw_Angle;
extern int16_t Vision_Shoot;

#endif
