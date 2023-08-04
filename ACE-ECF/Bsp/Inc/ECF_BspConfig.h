#ifndef __BSP_CONFIG_H_
#define __BSP_CONFIG_H_




/**************** CAN  part ****************/
#define CONFIG_CAN_FIFO_ENABLE 0

#define CONFIG_CAN1_FIFO_ENABLE 1
#define CONFIG_CAN2_FIFO_ENABLE 1

//CAN FIFO 容量设置
//发送FIFO的单元数量，要2的幂次方
#define CAN1_TX_FIFO_UNIT_NUM (256)
#define CAN2_TX_FIFO_UNIT_NUM (256)

//CAN接收回调函数指针。
// 入参：CAN_RxHeaderTypeDef *header, uint8_t *data
// out: void
#define ECF_CAN1_Rx_Callback_Fun NULL
#define ECF_CAN2_Rx_Callback_Fun NULL

#define CONFIG_CAN_SEND_MOTOR_USE_FIFO 1



/**************** DR16 part ****************/
#define CONFIG_DR16_USART_HANDLE huart3

#define DR16_LOST_DETECT_ENABLE 1

#if DR16_LOST_DETECT_ENABLE
/* 超过这个时间没有收到新的遥控器数据就认为已经失控 */
#define REMOTE_LOST_TIME ((uint32_t)50) //50ms
#endif

#endif

