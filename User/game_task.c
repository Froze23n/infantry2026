#include "game_task.h" //分配TIM13 200Hz
#include "referee.h"
#include "motors.h"
#include "usbd_cdc_if.h"
#include "tim.h"

void Game_Start(void){
    HAL_TIM_Base_Start_IT(&htim13); //200Hz
}

void Game_Task(void){
    //200Hz
    static uint8_t cnt = 0;
    cnt = (cnt + 1) % 10;
    if(cnt == 0){
        //20Hz
        Referee_UI_Update();
        Capacitor_Tx(referee.robot_status.chassis_power_limit, referee.power_heat_data.buffer_energy);
    }
}

void USB_TX(void){
    char content[] = "This is STM32F407IGH6 for Infantry.";
    CDC_Transmit_FS((uint8_t *)content, sizeof(content));
}

void UBS_RX(void){
    
}
