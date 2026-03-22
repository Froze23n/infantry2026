#include "usb.h"
#include "usbd_cdc_if.h"

void USB_TX(void){
    char content[] = "This is STM32F407IGH6 for Infantry.";
    CDC_Transmit_FS((uint8_t *)content, sizeof(content));
}

void UBS_RX(void){
    
}
