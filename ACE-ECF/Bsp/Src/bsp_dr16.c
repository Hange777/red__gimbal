


/**
*****************************东莞理工学院ACE实验室*****************************
* @file 		remote_control.c
*
* @brief 		包括遥控器初始化，遥控器数据获取，遥控器通讯协议的解析
*
* @note  		遥控器数据接收采用串口加DMA的模式
* @history              全新升级，支持hal库
*
@verbatim
==============================================================================

//bool_t (*data_is_error_fun)(void); 数值错误判断函数 -> bool_t  RC_Check_Data_IS_ERROR(void);
//void (*solve_lost_fun)(void);      问题解决处理函数 -> void RC_Restart(UART_HandleTypeDef *huart,uint16_t dma_buf_num);
//void (*solve_data_error_fun)(void); //数值错误处理函数 -> void RC_DataReload(void);

==============================================================================
@endverbatim
*****************************东莞理工学院ACE实验室******************************/

#include "bsp_dr16.h"

#if DR16_LOST_DETECT_ENABLE
#include "freertos.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"
#endif

#define Sbus_RX_Buffer_Num 36
#define RC_FRAME_LENGTH 18
#define RC_CHANNAL_ERROR_VALUE 700  //遥控出错上限


static TickType_t RCLostTime = 0;

// 遥控器数据结构体变量
static RC_ctrl_t rc_ctl;
// 接收数据缓存，有两个缓冲区，每一缓冲区有18个字节（8*18=144），为防止DMA传输越界给36
uint8_t Sbus_RX_Buffer[2][Sbus_RX_Buffer_Num];


#if DR16_LOST_DETECT_ENABLE
uint32_t chassis_rc_lost_time(void)
{
    return RCLostTime;
}


void rc_lost_time_refresh()
{
    /* 当接收到数据时，；刷新失联倒计时   xTaskGetTickCountFromISR  xTaskGetTickCount  */
    RCLostTime = xTaskGetTickCount() + REMOTE_LOST_TIME;
}
#endif
/**
 * @brief      遥控器数据死区限制
 * @param[in]  input
 * @param[in]  dealine
 * @retval
 */
int16_t rc_deadline_limit(int16_t input, int16_t dealine)
{
    if (input > dealine || input < -dealine)
    {
        return input;
    }
    else
    {
        return 0;
    }
}
/************************** Dongguan-University of Technology*-ACE**************************
 * @brief 获取遥控器数据指针
 *
 * @return const RC_ctrl_t*
 ************************** Dongguan-University of Technology*-ACE***************************/
RC_ctrl_t *RC_Get_RC_Pointer(void) { return &rc_ctl; }

/************************** Dongguan-University of Technology*-ACE**************************
 * @brief简介 遥控器初始化函数
 *      dma双缓冲接收遥控器发来的数据，数据帧为18字节
 *      有机会就试试fifo无锁队列
 * @param入参 huart huart指针
 * @param入参 Rx1_Buff 缓冲区1基地址
 * @param入参 Rx2_Buff 缓冲区2基地址
 * @param入参 Data_Buff_Lenth 数据长度
 ************************** Dongguan-University of Technology*-ACE************************** */
void RC_Init(UART_HandleTypeDef *huart, uint8_t *Rx1_Buff, uint8_t *Rx2_Buff,
             uint16_t Data_Buff_Lenth) {
    // 使能DMA串口接收
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
    // 使能空闲中断
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    //    //设置DMA传输，将串口1的数据搬运到recvive_buff中
    //    HAL_UART_Receive_DMA(&huart1, sbus_rx_buf[0], 36 );
    // 失效DMA
    __HAL_DMA_DISABLE(huart->hdmarx);
    while (huart->hdmarx->Instance->CR & DMA_SxCR_EN) {
    __HAL_DMA_DISABLE(huart->hdmarx);
    }
    huart->hdmarx->Instance->PAR = (uint32_t) & (huart->Instance->DR);
    // 内存缓冲区1
    huart->hdmarx->Instance->M0AR = (uint32_t)(Rx1_Buff);
    // 内存缓冲区2
    huart->hdmarx->Instance->M1AR = (uint32_t)(Rx2_Buff);
    // 数据长度
    huart->hdmarx->Instance->NDTR = Data_Buff_Lenth;
    // 使能双缓冲区
    SET_BIT(huart->hdmarx->Instance->CR, DMA_SxCR_DBM);
    // 使能DMA
    __HAL_DMA_ENABLE(huart->hdmarx);
}

/************************** Dongguan-University of Technology*-ACE**************************
 * @brief 遥控器原始数据协议解析
 *
 * @param pData 原始数据数组sbus
 * @param RC_CTRL 遥控器统一结构体
 * @return int
 ************************** Dongguan-University of Technology*-ACE***************************/
int RC_DataProcess(volatile const uint8_t *pData, RC_ctrl_t *RC_CTRL) {

#if DR16_LOST_DETECT_ENABLE
    rc_lost_time_refresh();
#endif

    RC_CTRL->rc.ch[0] = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;        //!< Channel 0
    RC_CTRL->rc.ch[1] = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF; //!< Channel 1
    RC_CTRL->rc.ch[2] = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |          //!< Channel 2
                         ((int16_t)pData[4] << 10)) & 0x07FF;

    RC_CTRL->rc.ch[3] = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5] << 7)) & 0x07FF; //!< Channel 3

    RC_CTRL->rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;                  //!< Switch left
    RC_CTRL->rc.s2 = ((pData[5] >> 4) & 0x0003);                       //!< Switch right

    RC_CTRL->mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);   //!< Mouse X axis
    RC_CTRL->mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);   //!< Mouse Y axis
    RC_CTRL->mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8); //!< Mouse Z axis

    RC_CTRL->mouse.press_l = pData[12]; //!< Mouse Left Is Press ?
    RC_CTRL->mouse.press_r = pData[13]; //!< Mouse Right Is Press ?

    RC_CTRL->kb.key_code = pData[14] | (pData[15] << 8);              //!< KeyBoard value
    RC_CTRL->rc.ch[4] = ((int16_t)pData[16]) | ((int16_t)pData[17] << 8); //左上角滑轮

    RC_CTRL->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    RC_CTRL->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    RC_CTRL->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    RC_CTRL->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    RC_CTRL->rc.ch[4] -= RC_CH_VALUE_OFFSET;
		
		RC_CTRL->rc.ch[0] = rc_deadline_limit(RC_CTRL->rc.ch[0], 5); //死区限制
    RC_CTRL->rc.ch[1] = rc_deadline_limit(RC_CTRL->rc.ch[1], 5); //死区限制
    RC_CTRL->rc.ch[2] = rc_deadline_limit(RC_CTRL->rc.ch[2], 5); //死区限制
    RC_CTRL->rc.ch[3] = rc_deadline_limit(RC_CTRL->rc.ch[3], 5); //死区限制

    return 0;
}

// 将变量拉出来看看(debug用)
static uint16_t this_time_rx_len = 0;
/************************** Dongguan-University of Technology*-ACE**************************
 * @brief 自定义的一个串口中断，加在stm32f4xx_it.c的用户自定义串口中断里了
 *
 * @param huart
 ************************** Dongguan-University of Technology*-ACE***************************/
void RC_UART_IRQHandler(UART_HandleTypeDef *huart) {
    if (huart->Instance->SR & UART_FLAG_RXNE) // 接收到数据
                      // //SR寄存器是状态寄存器，若其与UART_FLAG_RXNE（00010100）与运算有1，则说明有接收到数据
    {
    __HAL_UART_CLEAR_PEFLAG(huart);
    } else if (huart->Instance->SR & UART_FLAG_IDLE) // 串口处于空闲状态  （UART_FLAG_IDLE =
                             // 0：未检测到空闲线路 1：检测到空闲线路）
    { // 在空闲中断里判断数据帧的传送是否正确
    // 当串口开始接收数据后，检测到1字节数据的时间内没有数据发送，则认为串口空闲了。

    __HAL_UART_CLEAR_PEFLAG(huart);

    if ((huart->hdmarx->Instance->CR & DMA_SxCR_CT) == RESET) {
      /* Current memory buffer used is Memory 0 */

      // disable DMA
      // 失效DMA
      __HAL_DMA_DISABLE(huart->hdmarx);

      // get receive data length, length = set_data_length - remain_length
      // 获取接收数据长度,长度 = 设定长度 - 剩余长度
      this_time_rx_len = Sbus_RX_Buffer_Num - huart->hdmarx->Instance->NDTR;
      // reset set_data_lenght
      // 重新设定数据长度
      huart->hdmarx->Instance->NDTR = Sbus_RX_Buffer_Num;

      // set memory buffer 1
      // 设定缓冲区1
      huart->hdmarx->Instance->CR |= DMA_SxCR_CT;

      // enable DMA
      // 使能DMA
      __HAL_DMA_ENABLE(huart->hdmarx);

      if (this_time_rx_len == RC_FRAME_LENGTH) {
        RC_DataProcess(Sbus_RX_Buffer[0], &rc_ctl);
      }
    } else {
      /* Current memory buffer used is Memory 1 */
      // disable DMA
      // 失效DMA
      __HAL_DMA_DISABLE(huart->hdmarx);

      // get receive data length, length = set_data_length - remain_length
      // 获取接收数据长度,长度 = 设定长度 - 剩余长度
      this_time_rx_len = Sbus_RX_Buffer_Num - huart->hdmarx->Instance->NDTR;

      // reset set_data_lenght
      // 重新设定数据长度
      huart->hdmarx->Instance->NDTR = Sbus_RX_Buffer_Num;

      // set memory buffer 0
      // 设定缓冲区0
      huart->hdmarx->Instance->CR &= ~(DMA_SxCR_CT);

      // enable DMA
      // 使能DMA
      __HAL_DMA_ENABLE(huart->hdmarx);

      if (this_time_rx_len == RC_FRAME_LENGTH) {
        // 处理遥控器数据
        RC_DataProcess(Sbus_RX_Buffer[1], &rc_ctl);
      }
    }
  }
}

/************************** Dongguan-University of Technology*-ACE**************************
 * @brief 遥控器绝对值处理
 *
 * @param num
 * @return int16_t
 ************************** Dongguan-University of Technology*-ACE***************************/
static int16_t RC_abs(int16_t num) {
  if (num < 0)
    return -num;
  else
    return num;
}


/************************** Dongguan-University of Technology*-ACE**************************
 * @brief 重启遥控器
 *
 * @param入参 dma_buf_num
 ************************** Dongguan-University of Technology*-ACE***************************/
void RC_Restart(UART_HandleTypeDef *huart, uint16_t dma_buf_num) {
    __HAL_UART_DISABLE(huart);
    __HAL_DMA_DISABLE(huart->hdmarx);

    huart->hdmarx->Instance->NDTR = dma_buf_num; // 重新写入长度

    __HAL_UART_ENABLE(huart);
    __HAL_DMA_ENABLE(huart->hdmarx);
}

/************************** Dongguan-University of Technology*-ACE**************************
 * @brief 遥控数据清零
 *
 ************************** Dongguan-University of Technology*-ACE***************************/
void RC_DataReload(void) {
    rc_ctl.rc.ch[0] = 0;
    rc_ctl.rc.ch[1] = 0;
    rc_ctl.rc.ch[2] = 0;
    rc_ctl.rc.ch[3] = 0;
    rc_ctl.rc.ch[4] = 0;
    rc_ctl.mouse.x = 0;
    rc_ctl.mouse.y = 0;
    rc_ctl.mouse.z = 0;
    rc_ctl.mouse.press_l = 0;
    rc_ctl.mouse.press_r = 0;
    rc_ctl.kb.key_code  = 0;
}

/************************** Dongguan-University of Technology*-ACE**************************
 * @brief 判断遥控器是否出错，采用go to 语句统一将错误数据归零
 *
 * @return uint8_t
************************** Dongguan-University of Technology*-ACE***************************/
bool_t RC_Check_Data_IS_ERROR(void) {
    if (RC_abs(rc_ctl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE) {
        goto error;
    }
    if (RC_abs(rc_ctl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE) {
        goto error;
    }
    if (RC_abs(rc_ctl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE) {
        goto error;
    }
    if (RC_abs(rc_ctl.rc.ch[4]) > RC_CHANNAL_ERROR_VALUE) {
        goto error;
    }
    if (rc_ctl.rc.s1 == 0) {
        goto error;
    }
    if (rc_ctl.rc.s2 == 0) {
        goto error;
    }

    return 0;
error:
    rc_ctl.rc.ch[0] = 0;
    rc_ctl.rc.ch[1] = 0;
    rc_ctl.rc.ch[2] = 0;
    rc_ctl.rc.ch[3] = 0;
    rc_ctl.rc.ch[4] = 0;

    rc_ctl.rc.s1 = RC_SW_DOWN;
    rc_ctl.rc.s2 = RC_SW_DOWN;
    rc_ctl.mouse.x = 0;
    rc_ctl.mouse.y = 0;
    rc_ctl.mouse.z = 0;
    rc_ctl.mouse.press_l = 0;
    rc_ctl.mouse.press_r = 0;
    rc_ctl.kb.key_code = 0;
    return 1;
}

/************************** Dongguan-University of Technology*-ACE**************************
 * @brief ECF版本遥控器初始化，默认C板
 *
 ************************** Dongguan-   University of Technology*-ACE***************************/
void ECF_RC_Init(void) {
    RC_Init(&CONFIG_DR16_USART_HANDLE, Sbus_RX_Buffer[0], Sbus_RX_Buffer[1], Sbus_RX_Buffer_Num);
}

/************************** Dongguan-University of Technology*-ACE**************************
 * @brief ECF版本遥控器中断服务函数，记得扔it中断里（建议在HAL库清楚标志位前放这个）
 *
 ************************** Dongguan-   University of Technology*-ACE***************************/
void ECF_RC_UART_Handler(void){
    RC_UART_IRQHandler(&CONFIG_DR16_USART_HANDLE);
}


