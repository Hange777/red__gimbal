/************************** Dongguan-University of Technology -ACE**************************
* @file dr16.h
* @author pansyhou侯文辉 & 郑杰瀚 (1677195845lyb@gmail.com)
* @brief简介 遥控器相关头文件
* @version 0.1
* @date 2022-01-26
*
* @copyright Copyright (c) 2022
*
************************** Dongguan-University of Technology -ACE************************** */
#ifndef  __BSP_DR16_H
#define  __BSP_DR16_H
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "usart.h"
#include <struct_typedef.h>
#include <ECF_BspConfig.h>

extern UART_HandleTypeDef CONFIG_DR16_USART_HANDLE;

/* 遥控文档内容-BEGIN */
/* ----------------------- RC Channel Definition---------------------------- */
#define RC_CH_VALUE_MIN            ((uint16_t)364 )			//通道最小值
#define RC_CH_VALUE_OFFSET         ((uint16_t)1024)			//通道中间值
#define RC_CH_VALUE_MAX            ((uint16_t)1684)			//通道最大值
/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_ERROR                ((uint16_t)0) //出现严重错误
#define RC_SW_UP                   ((uint16_t)1)
#define RC_SW_MID                  ((uint16_t)3)
#define RC_SW_DOWN                 ((uint16_t)2)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W       ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S       ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A       ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D       ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT   ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL    ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q       ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E       ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R       ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F       ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G       ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z       ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X       ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C       ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V       ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B       ((uint16_t)1 << 15)
/* ----------------------- Data Struct ------------------------------------- */


/*
* 遥控器接收数据结构体*/
//__packed为字节对齐
typedef struct{
    struct{
        int16_t ch[5];
        uint8_t s1;
        uint8_t s2;
    } rc;
    struct{
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;
    /* keyboard key information */
    union
    {
        uint16_t key_code;
        struct
        {
            uint16_t W : 1;
            uint16_t S : 1;
            uint16_t A : 1;
            uint16_t D : 1;
            uint16_t SHIFT : 1;
            uint16_t CTRL : 1;
            uint16_t Q : 1;
            uint16_t E : 1;
            uint16_t R : 1;
            uint16_t F : 1;
            uint16_t G : 1;
            uint16_t M : 1;//uint16_t Z : 1;
            uint16_t N : 1;//uint16_t X : 1;
            uint16_t C : 1;
            uint16_t V : 1;
            uint16_t B : 1;
        } bit;
    } kb;
    int8_t Flag;
} RC_ctrl_t;

#ifdef  DR16_LOST_DETECT_ENABLE

uint32_t chassis_rc_lost_time(void);
void rc_lost_time_refresh(void);

#endif

void    ECF_RC_Init(void);
void    ECF_RC_UART_Handler(void);

RC_ctrl_t *RC_Get_RC_Pointer(void);
bool_t  RC_Check_Data_IS_ERROR(void);
void    RC_Restart(UART_HandleTypeDef *huart,uint16_t dma_buf_num);
void    RC_DataReload(void);


#endif

