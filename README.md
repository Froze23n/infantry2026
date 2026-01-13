## 控制逻辑

`body_task`、`neck_task`、`head_task` 模块通过综合 `imu`、`dbus`、`motors` 
的数据 (在对应.h中被 **extern** )，
借助 `pid` 计算出各个电机控制电流 / 电压，并通过 `motors` 提供的发送接口对电机进行控制。

## 定时器中断任务

- `void Body_Task(void)`
    - 由 **TIM7** 定时执行
    - 执行频率：**125 Hz**


- `void Neck_Task(void)`
    - 由 **TIM6** 定时执行
    - 执行频率：**1000 Hz**


- `void Head_Task(void)`
    - 由 **TIM4** 定时执行
    - 执行频率：**1000 Hz**


- `void IMU_Task(void)`
    - 由 **TIM10** 定时执行
    - 执行频率：**1000 Hz**

## 其他中断任务

- `void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)`
- `void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan)`
  - CAN接收中断触发时由HAL库自行调用 频率取决于电机


- `void Dbus_UART_IRQHandler(void)`
  - 串口空闲中断触发，由**USART3_IRQHandler**执行，周期为14ms