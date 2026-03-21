#include "vt.h"
#include "usart.h"
#include "iwdg.h"
#include <string.h>

//裁判系统图传链路
/* ------------------------------ 宏定义 变量 内部函数声明 ------------------------------ */
#define VT_HUART huart1           // 定义遥控器串口句柄
#define VT_HDMA_RX huart1.hdmarx  // 定义遥控器DMA句柄
#define VT_HUART_BASE USART1      //串口外设基地址
#define VT_RX_BUF_SIZE 64        //最大接收长度
#define VT_RC_LEN 21 //遥控器数据长度
#define VT_CH_OFFSET 1024
#define VT_CH_RATIO 660.0f

VT_Host_Type vt;
static uint8_t VT_RxBuf[2][VT_RX_BUF_SIZE];  // 接收到的原始数据

struct
{
    unsigned a,b,c,d,e,f;
} vt_error;
unsigned vt_pack=0;


static void VT_Data_Process(uint8_t* buffer, int32_t length);
static uint16_t get_crc16_check_sum(uint8_t *p_msg, uint16_t len);

void VT_Init(void)
{
    __HAL_UART_ENABLE_IT(&VT_HUART, UART_IT_IDLE);  // 使能空闲中断
    
	SET_BIT(VT_HUART.Instance->CR3,USART_CR3_DMAR); //使能DMA串口接收
    
	do{
        __HAL_DMA_DISABLE(VT_HDMA_RX); //失能DMA
    }while (VT_HDMA_RX->Instance->CR & DMA_SxCR_EN); //检查是否失能
    
	//配置DMA
    VT_HDMA_RX->Instance->PAR = (uint32_t) &(VT_HUART_BASE->DR); //传输源地址为串口的数据寄存器
    VT_HDMA_RX->Instance->M0AR= (uint32_t)(VT_RxBuf[0]); //内存缓冲区0
    VT_HDMA_RX->Instance->M1AR= (uint32_t)(VT_RxBuf[1]); //内存缓冲区1
    VT_HDMA_RX->Instance->NDTR= VT_RX_BUF_SIZE;
    SET_BIT(VT_HDMA_RX->Instance->CR, DMA_SxCR_DBM); //使能双缓冲区
    
	__HAL_DMA_ENABLE(VT_HDMA_RX); //启动！
}

void VT_IRQHandler(void)
{
    if(VT_HUART.Instance->SR & UART_FLAG_IDLE) //仅处理空闲中断
    {
        static uint16_t currxlen = 0; //静态变量有利于控制中断栈大小
        __HAL_UART_CLEAR_PEFLAG(&VT_HUART); //清零空闲中断标志

        do{
            __HAL_DMA_DISABLE(VT_HDMA_RX); //失能DMA
        }while (VT_HDMA_RX->Instance->CR & DMA_SxCR_EN); //轮询是否失能

        currxlen = VT_RX_BUF_SIZE - VT_HDMA_RX->Instance->NDTR; //记录数据长度
        
        VT_HDMA_RX->Instance->NDTR = VT_RX_BUF_SIZE; //重设长度

        if( (VT_HDMA_RX->Instance->CR & DMA_SxCR_CT) == 0 )
        {
            VT_HDMA_RX->Instance->CR |= DMA_SxCR_CT; //0->1 更换缓冲区
            __HAL_DMA_ENABLE(VT_HDMA_RX); //启动接收
            VT_Data_Process(VT_RxBuf[0], (int32_t)currxlen); //数据处理
        }
        else
        {
            VT_HDMA_RX->Instance->CR &= ~DMA_SxCR_CT; //1->0 更换缓冲区
            __HAL_DMA_ENABLE(VT_HDMA_RX); //启动接收
            VT_Data_Process(VT_RxBuf[1], (int32_t)currxlen); //数据处理
        }
    }
}

static void VT_Data_Process(uint8_t* buffer, int32_t length){
	if(length > VT_RX_BUF_SIZE){vt_error.f++;} //数据长度过大
    
    while(length!=0)
    {
        vt_pack++;
        if(length<0){vt_error.a++; return;} //最后一个数据包残缺

		if(buffer[0] == 0xA9 && buffer[1] == 0x53){
			uint16_t crc16 = get_crc16_check_sum(buffer, VT_RC_LEN - 2);
			if(crc16 == (buffer[VT_RC_LEN - 2] | (buffer[VT_RC_LEN - 1] << 8))){ //CRC16校验
	            HAL_IWDG_Refresh(&hiwdg); //成功接收到遥控器数据，喂狗防止程序重启
				
                VT_Wire_Type *vt_wire = (VT_Wire_Type *)buffer;
                
                vt.LX = (vt_wire->rc.bit.ch_3 - VT_CH_OFFSET) / VT_CH_RATIO;
                vt.LY = (vt_wire->rc.bit.ch_2 - VT_CH_OFFSET) / VT_CH_RATIO;
                vt.RX = (vt_wire->rc.bit.ch_0 - VT_CH_OFFSET) / VT_CH_RATIO;
                vt.RY = (vt_wire->rc.bit.ch_1 - VT_CH_OFFSET) / VT_CH_RATIO;
                vt.wheel = (vt_wire->rc.bit.wheel - VT_CH_OFFSET) / VT_CH_RATIO;
                
                vt.CNS = vt_wire->rc.bit.mode_sw;
                vt.pause = vt_wire->rc.bit.pause;
                vt.FN_L = vt_wire->rc.bit.fn_1;
                vt.FN_R = vt_wire->rc.bit.fn_2;
                vt.trigger = vt_wire->rc.bit.trigger;
                
                vt.mouse_x = (float)vt_wire->mouse.bit.mouse_x / 327.680f;
                vt.mouse_y = -(float)vt_wire->mouse.bit.mouse_y / 327.680f;
                vt.mouse_z = (float)vt_wire->mouse.bit.mouse_z / 327.680f;
                vt.mouse_left = vt_wire->mouse.bit.mouse_left;
                vt.mouse_right = vt_wire->mouse.bit.mouse_right;
                vt.mouse_middle = vt_wire->mouse.bit.mouse_middle;

                vt.keyboard.raw = vt_wire->keyboard.raw;
			}else{
				vt_error.b++; //CRC16校验失败
			}
			buffer += VT_RC_LEN; //移动到下一个数据包
			length -= VT_RC_LEN; //更新剩余长度
		}else if(buffer[0] == 0xA5){
			//裁判系统图传链路数据
            uint16_t data_length = buffer[1] | buffer[2]<<8; //数据段长度
            //步兵不需要自定义控制器数据，直接丢弃
            buffer += (data_length + 9);
            length -= (data_length + 9);
		}else{
			return;
		}
    }
}

static const uint16_t crc16_tab[256] =
{
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

static uint16_t get_crc16_check_sum(uint8_t *p_msg, uint16_t len)
{
	uint16_t crc16 = 0xffff;
    uint8_t data;

    if(p_msg == NULL)
    {
        return 0xffff;
    }

    while(len--)
    {
        data = *p_msg++;
        (crc16) = ((uint16_t)(crc16) >> 8) ^ crc16_tab[((uint16_t)(crc16) ^ (uint16_t)(data)) & 0x00ff];
    }

    return crc16;
}
