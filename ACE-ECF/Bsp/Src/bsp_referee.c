/**
*****************************东莞理工学院ACE实验室*****************************
* @file 		bsp_referee.c
*
* @brief 		包括裁判系统初始化，裁判系统数据获取，裁判系统通讯协议的解析
* @author   叶彦均
* @note  		
* @history  全新升级，支持hal库
* @version  1.0

@verbatim
==============================================================================

ECF_referee_uart_init()丢到初始化
REFEREE_UART_IRQHandler(UART_HandleTypeDef *huart)丢到uart6中断就行了

==============================================================================
@endverbatim
*****************************东莞理工学院ACE实验室******************************/


#include "bsp_referee.h"

#define RX_Buffer_Num 256

extern UART_HandleTypeDef huart6;

// 接收数据缓存，有两个缓冲区
uint8_t Referee_RX_Buffer[2][RX_Buffer_Num];
uint8_t Referee_Data[256];

REFEREE_t referee;

static uint16_t this_time_rx_len  = 0;

/*裁判数据接收数据处理*/
void RefereeDataDeal(REFEREE_t *referee);
/*比赛状态*/
static void GAME_STATUS(REFEREE_t *referee, unsigned char k);
/*比赛结果数据*/
static void GAME_RESULT(REFEREE_t *referee, unsigned char k);
/*机器人血量状态数据*/
static void ROBOT_HP(REFEREE_t *referee, unsigned char k);
/*补给站动作标识*/
static void SUPPLY_PROJECTILE_ACTION(REFEREE_t *referee, unsigned char k);
/*裁判警告数据*/
static void REFEREE_WARNING(REFEREE_t *referee, unsigned char k);
/*飞镖发射口倒计时*/
static void DART_REAMAINING_TIME(REFEREE_t *referee, unsigned char k);
/*机器人状态*/
static void ROBOT_STATUS(REFEREE_t *referee, unsigned char k);
/*功率热量*/
static void POWER_HEAT(REFEREE_t *referee, unsigned char k);
/*机器人位置数据*/
static void ROBOT_POSITION(REFEREE_t *referee, unsigned char k);
/*机器人增益*/
static void ROBOT_BUFF(REFEREE_t *referee, unsigned char k);
/*场地事件数据*/
static void EVENT_DATA(REFEREE_t *referee, unsigned char k);
/*无人机能量状态数据*/
static void UAV_ENERGY_TIME(REFEREE_t *referee, unsigned char k);
/*伤害状态数据*/
static void HURT_DATA(REFEREE_t *referee, unsigned char k);
/*实时射击数据*/
static void SHOOT_DATA(REFEREE_t *referee, unsigned char k);
/*剩余弹丸和金币数据*/
static void BULLET_DATA(REFEREE_t *referee, unsigned char k);
/*机器人RFID状态*/
static void RFID_STATUS(REFEREE_t *referee, unsigned char k);
/*飞镖机器人客户端指令*/
static void DART_CLIENT(REFEREE_t *referee, unsigned char k);
/*交互数据接收信息*/
static void INTERACT_HEADER(REFEREE_t *referee, unsigned char k);
/*客户端下发信息*/
static void CLIENT_DATA(REFEREE_t *referee, unsigned char k);
/*客户端接受信息*/
static void CLIENT_MAP_DATA(REFEREE_t *referee, unsigned char k);
	
	
REFEREE_t *Get_referee_Address(void)
{
	return &referee;
}

/************************** Dongguan-University of Technology*-ACE**************************
* @brief 自定义的一个串口中断，加在stm32f4xx_it.c的用户自定义串口中断里了
*
* @param huart
************************** Dongguan-University of Technology*-ACE***************************/

void referee_uart_init(UART_HandleTypeDef *huart,uint8_t *rx_buffer1, uint8_t *rx_buffer2, uint8_t buff_num)
{
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
    huart->hdmarx->Instance->M0AR = (uint32_t)(rx_buffer1);
    // 内存缓冲区2
    huart->hdmarx->Instance->M1AR = (uint32_t)(rx_buffer2);
    // 数据长度
    huart->hdmarx->Instance->NDTR = buff_num;
    // 使能双缓冲区
    SET_BIT(huart->hdmarx->Instance->CR, DMA_SxCR_DBM);
    // 使能DMA
    __HAL_DMA_ENABLE(huart->hdmarx);
}

void ECF_referee_uart_init()
{
	referee_uart_init(&huart6 ,Referee_RX_Buffer[0] ,Referee_RX_Buffer[1], 255);
}

void REFEREE_UART_IRQHandler(UART_HandleTypeDef *huart) 
	{
		//SR寄存器是状态寄存器，若其与UART_FLAG_RXNE（00010100）与运算有1，则说明有接收到数据
		if (huart->Instance->SR & UART_FLAG_RXNE) // 接收到数据
		{
		__HAL_UART_CLEAR_PEFLAG(huart);
		} else  
		if (huart->Instance->SR & UART_FLAG_IDLE) // 串口处于空闲状态  （UART_FLAG_IDLE =
														// 0：未检测到空闲线路 1：检测到空闲线路）
		{ // 在空闲中断里判断数据帧的传送是否正确
		// 当串口开始接收数据后，检测到1字节数据的时间内没有数据发送，则认为串口空闲了。

		__HAL_UART_CLEAR_PEFLAG(huart);

			if ((huart->hdmarx->Instance->CR & DMA_SxCR_CT) == RESET) 
			{
				/* Current memory buffer used is Memory 0 */
	
				// disable DMA
				// 失效DMA
				__HAL_DMA_DISABLE(huart->hdmarx);
	
				// get receive data length, length = set_data_length - remain_length
				// 获取接收数据长度,长度 = 设定长度 - 剩余长度
				this_time_rx_len = RX_Buffer_Num - huart->hdmarx->Instance->NDTR;
				// reset set_data_lenght
				// 重新设定数据长度
				huart->hdmarx->Instance->NDTR = RX_Buffer_Num;
	
				// set memory buffer 1
				// 设定缓冲区1
				huart->hdmarx->Instance->CR |= DMA_SxCR_CT;
	
				// enable DMA
				// 使能DMA
				__HAL_DMA_ENABLE (huart->hdmarx);
				
					referee.DataLen= this_time_rx_len;
					
					for (int i = 0; i < this_time_rx_len; i++)
					{
						referee.RefereeData[i] = Referee_RX_Buffer[0][i];					
					}

					RefereeDataDeal(&referee);
			} 
			else 
			{
				/* Current memory buffer used is Memory 1 */
				// disable DMA
				// 失效DMA
				__HAL_DMA_DISABLE (huart->hdmarx);
	
				// get receive data length, length = set_data_length - remain_length
				// 获取接收数据长度,长度 = 设定长度 - 剩余长度
				this_time_rx_len = RX_Buffer_Num - huart->hdmarx->Instance->NDTR;
	
				// reset set_data_lenght
				// 重新设定数据长度
				huart->hdmarx->Instance->NDTR = RX_Buffer_Num ; 
	
				// set memory buffer 0
				// 设定缓冲区0
				huart->hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
	
				// enable DMA
				// 使能DMA
				__HAL_DMA_ENABLE	(huart->hdmarx);
	
				referee.DataLen= this_time_rx_len;
					
				for (int i = 0; i < this_time_rx_len; i++)
				{
					referee.RefereeData[i] = Referee_RX_Buffer[1][i];
				}
				
				RefereeDataDeal(&referee);
			}
		}	
}

/*裁判数据接收数据处理*/
void RefereeDataDeal(REFEREE_t *referee)
{
	uint8_t i;
	for (i = 0; i < referee->DataLen; i++)
	{
		if (referee->RefereeData[i] == 0xA5) //帧头
		{
			if (Verify_CRC8_Check_Sum(referee->RefereeData, HEADER_LEN) == 1) //CRC8校验
			{
				referee->RealLen = ((referee->RefereeData[i + 1]) | (referee->RefereeData[i + 2] << 8));					  //数据长度
				referee->Cmd_ID = ((referee->RefereeData[i + HEADER_LEN]) | (referee->RefereeData[i + HEADER_LEN + 1] << 8)); //命令码ID

				switch (referee->Cmd_ID)
				{
				case ID_STATE:
					GAME_STATUS(referee, i); 
					i = i + (DATA_STATUS_LEN + 9) + 9 - 1;
					break;
				
				case ID_RESULT:
					GAME_RESULT(referee, i);         
					i = i + (DATA_RESULT_LEN + 9) - 1;
					break;
				
				case ID_ROBOT_HP:					
					ROBOT_HP(referee, i);       
					i = i + (DATA_ROBOT_HP_LEN + 9) - 1;				
					break;
				
				case ID_EVENT_DATA:
					EVENT_DATA(referee, i);             //无RFID，待测试
					i = i + (DATA_EVENT_DATA_LEN + 9) - 1;
					break;
				
				case ID_SUPPLY_PROJECTILE_ACTION:
					SUPPLY_PROJECTILE_ACTION(referee, i);    
					i = i + (DATA_SUPPLY_PROJECTILE_ACTION_LEN + 9) - 1;
					break;
				case ID_REFEREE_WARNING:
					REFEREE_WARNING(referee, i);       
					i = i + (DATA_REFEREE_WARNING_LEN + 9) - 1;
					break;
				case ID_DART_REMAINING_TIME:
					DART_REAMAINING_TIME(referee, i);       
					i = i + (DATA_DART_REMAINING_TIME_LEN + 9) - 1;
					break;
				case ID_ROBOT_STATE:
					ROBOT_STATUS(referee, i);            
					i = i + (DATA_ROBOT_STATUS_LEN + 9) - 1;
					break;
				case ID_POWER_HEAT_DATA:
					POWER_HEAT(referee, i);   
					i = i + (DATA_POWER_HEAT_DATA_LEN + 9) - 1;
					break;
				case ID_ROBOT_POS:
					ROBOT_POSITION(referee, i);   
					i = i + (DATA_ROBOT_POS_LEN + 9) - 1;
					break;
				case ID_BUFF:
					ROBOT_BUFF(referee, i);   //无RFID，待测试
					i = i + (DATA_BUFF_LEN + 9) - 1;
					break;
				case ID_AERIAL_ROBOT_ENERGY:
					UAV_ENERGY_TIME(referee, i);   //联盟赛无无人机
					i = i + (DATA_AERIAL_ROBOT_ENERGY_LEN + 9) - 1;
					break;
				case ID_ROBOT_HURT:
					HURT_DATA(referee, i);          
					i = i + (DATA_ROBOT_HURT_LEN + 9) - 1;
					break;
				case ID_SHOOT_DATA:
					SHOOT_DATA(referee, i);                   
					i = i + (DATA_SHOOT_DATA_LEN + 9) - 1;
					break;
				case ID_BULLET_REMAINING:
					BULLET_DATA(referee, i);                 
					i = i + (DATA_BULLET_REMAINING_LEN + 9) - 1;
					break;
				case ID_RFID_STATUS:
					RFID_STATUS(referee, i);           //无RFID，待测试
					i = i + (DATA_RFID_STATUS_LEN + 9) - 1;
					break;
				case ID_DART_CLIENT_CMD:
				 DART_CLIENT(referee, i);  
			  	i = i + (DATA_DART_CLIENT_CMD_LEN + 9) -1;
					break;
				case ID_DIY_CONTROLLER:
					INTERACT_HEADER(referee, i);  
					i = i + (DATA_DIY_CONTROLLER + 9) -1;
					break;
				case ID_CLIENT_DOWMLOAD:
					CLIENT_DATA(referee, i);  
					i = i + (DATA_CLIENT_DOWMLOAD + 9) -1;
					break;
				case ID_PICTURE_TRANSMISSION:
					
					i = i + (DATA_PICTURE_TRANSMISSION + 9) -1;
					break;
				case ID_CLIENT_RECEIVE:
					CLIENT_MAP_DATA(referee, i);
					i = i + (DATA_CLIENT_RECEIVE + 9) -1;
					break;
				default:
					break;
				}
			}
		}
	}
}

/*比赛状态数据*/
static void GAME_STATUS(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_STATUS_LEN + 9); //数据转移

	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_STATUS_LEN + 9) == 1) //CRC16校验
	{
		memcpy(&referee->Game_Status, referee->RealData + 7, DATA_STATUS_LEN);
		referee->Game_Status.error = 0;
	}
	else
	{
		referee->Game_Status.error = 1;
	}
}

/*比赛结果数据*/
static void GAME_RESULT(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_RESULT_LEN + 9); //数据转移

	if (Verify_CRC16_Check_Sum(referee->RealData,  DATA_RESULT_LEN + 9) == 1) //CRC16校验
	{
		memcpy(&referee->Game_Result, referee->RealData + 7, DATA_STATUS_LEN);
		referee->Game_Result.error = 0;
	}
	else
	{
		referee->Game_Result.error = 1;
	}
}

/*机器人血量状态数据*/
static void ROBOT_HP(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_ROBOT_HP_LEN + 9); //数据转移

	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_ROBOT_HP_LEN + 9) == 1) //CRC16校验
	{
		memcpy(&referee->Robot_HP, referee->RealData + 7, DATA_STATUS_LEN);
		referee->Robot_HP.error = 0;
	}
	else
	{
		referee->Robot_HP.error = 1;
	}
}

/*场地事件数据*/
static void EVENT_DATA(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_EVENT_DATA_LEN + 9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_EVENT_DATA_LEN + 9) == 1)
	{
		memcpy(&referee->Event_Data, referee->RealData + 7, DATA_EVENT_DATA_LEN);
		referee->Event_Data.error = 0;
	}
	else
	{
		referee->Event_Data.error = 1;
	}
}

/*补给站动作标识*/
static void SUPPLY_PROJECTILE_ACTION(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_SUPPLY_PROJECTILE_ACTION_LEN + 9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_SUPPLY_PROJECTILE_ACTION_LEN + 9) == 1)
	{
		memcpy(&referee->Supply_Action, referee->RealData + 7, DATA_SUPPLY_PROJECTILE_ACTION_LEN);
		referee->Supply_Action.error = 0;
	}
	else
	{
		referee->Supply_Action.error = 1;
	}
}

/*裁判警告数据*/
static void REFEREE_WARNING(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_REFEREE_WARNING_LEN + 9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_REFEREE_WARNING_LEN + 9) == 1)  //CRC16校验
	{
		memcpy(&referee->Referee_Warning, referee->RealData + 7, DATA_REFEREE_WARNING_LEN);
		referee->Referee_Warning.error = 0;
	}
	else
	{
		referee->Referee_Warning.error = 1;
	}
}

/*飞镖发射口倒计时*/
static void DART_REAMAINING_TIME(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_DART_REMAINING_TIME_LEN + 9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_DART_REMAINING_TIME_LEN + 9) == 1)  //CRC16校验
	{
		memcpy(&referee->Dart_Remaining_Time, referee->RealData + 7, DATA_DART_REMAINING_TIME_LEN);
		referee->Dart_Remaining_Time.error = 0;
	}
	else
	{
		referee->Dart_Remaining_Time.error = 1;
	}
}

/*机器人状态数据*/
static void ROBOT_STATUS(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_ROBOT_STATUS_LEN + 9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_ROBOT_STATUS_LEN + 9) == 1)  //CRC16校验
	{
		memcpy(&referee->Robot_Status, referee->RealData + 7, DATA_ROBOT_STATUS_LEN);
		referee->Robot_Status.error = 0;
	}
	else
	{
		referee->Robot_Status.error = 1;
	}
}

/*热量功率数据*/
static void POWER_HEAT(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_POWER_HEAT_DATA_LEN + 9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_POWER_HEAT_DATA_LEN + 9) == 1)  //CRC16校验
	{
		memcpy(&referee->Power_Heat, referee->RealData + 7, DATA_POWER_HEAT_DATA_LEN);
		referee->Power_Heat.error = 0;
	}
	else
	{
		referee->Power_Heat.error = 1;
	}
}

/*机器人位置数据*/
static void ROBOT_POSITION(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_ROBOT_POS_LEN + 9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_ROBOT_POS_LEN + 9) == 1)  //CRC16校验
	{
		memcpy(&referee->Robot_Position, referee->RealData + 7, DATA_ROBOT_POS_LEN);
		referee->Robot_Position.error = 0;
	}
	else
	{
		referee->Robot_Position.error = 1;
	}
}

/*机器人增益*/
static void ROBOT_BUFF(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_BUFF_LEN + 9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_BUFF_LEN + 9) == 1)  //CRC16校验
	{
		memcpy(&referee->Buff, referee->RealData + 7, DATA_BUFF_LEN);
		referee->Buff.error = 0;
	}
	else
	{
		referee->Buff.error = 1;
	}
}

/*无人机能量状态数据*/
static void UAV_ENERGY_TIME(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_AERIAL_ROBOT_ENERGY_LEN + 9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_AERIAL_ROBOT_ENERGY_LEN + 9) == 1)
	{
		memcpy(&referee->Aerial_Energy, referee->RealData + 7, DATA_AERIAL_ROBOT_ENERGY_LEN);
		referee->Aerial_Energy.error = 0;
	}
	else
	{
		referee->Aerial_Energy.error = 1;
	}
}

/*伤害状态数据*/
static void HURT_DATA(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_ROBOT_HURT_LEN + 9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_ROBOT_HURT_LEN + 9) == 1)
	{
		memcpy(&referee->Robot_Hurt, referee->RealData + 7, DATA_ROBOT_HURT_LEN);
		referee->Robot_Hurt.error = 0;
	}
	else
	{
		referee->Robot_Hurt.error = 1;
	}
}

/*子弹剩余数据*/
static void BULLET_DATA(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_BULLET_REMAINING_LEN + 9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_BULLET_REMAINING_LEN + 9) == 1)
	{
		memcpy(&referee->Bullet_Num, referee->RealData + 7, DATA_BULLET_REMAINING_LEN);
		referee->Bullet_Num.error = 0;
	}
	else
	{
		referee->Bullet_Num.error = 1;
	}
}

/*实时射击数据*/
static void SHOOT_DATA(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_SHOOT_DATA_LEN + 9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_SHOOT_DATA_LEN + 9) == 1)
	{
		memcpy(&referee->Shoot_Data, referee->RealData + 7, DATA_SHOOT_DATA_LEN);
		referee->Shoot_Data.error = 0;
	}
	else
	{
		referee->Shoot_Data.error = 1;
	}
}

/*机器人RFID状态*/
static void RFID_STATUS(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_RFID_STATUS_LEN + 9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_RFID_STATUS_LEN + 9) == 1)
	{
		memcpy(&referee->RFID_Status, referee->RealData + 7, DATA_RFID_STATUS_LEN);
		referee->RFID_Status.error = 0;
	}
	else
	{
		referee->RFID_Status.error = 1;
	}
}

/*飞镖机器人客户端指令*/
static void DART_CLIENT(REFEREE_t *referee, unsigned char k)
{
		memcpy(referee->RealData, referee->RefereeData + k, DATA_DART_CLIENT_CMD_LEN +9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_DART_CLIENT_CMD_LEN + 9) == 1)
	{
		memcpy(&referee->Dart_Client, referee->RealData + 7, DATA_DART_CLIENT_CMD_LEN);
		referee->Dart_Client.error = 0;
	}
	else
	{
		referee->Dart_Client.error = 1;
	}
}

/*交互数据接收信息*/
static void INTERACT_HEADER(REFEREE_t *referee, unsigned char k)
{
		memcpy(referee->RealData, referee->RefereeData + k, DATA_DIY_CONTROLLER + 9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_DIY_CONTROLLER + 9) == 1)
	{
		memcpy(&referee->Interact_Header, referee->RealData + 7, DATA_DIY_CONTROLLER);
		referee->Interact_Header.error = 0;
	}
	else
	{
		referee->Interact_Header.error = 1;
	}
}

/*客户端下发信息*/
static void CLIENT_DATA(REFEREE_t *referee, unsigned char k)
{
		memcpy(referee->RealData, referee->RefereeData + k, DATA_CLIENT_DOWMLOAD + 9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_CLIENT_DOWMLOAD + 9) == 1)
	{
		memcpy(&referee->Client_Data, referee->RealData + 7, DATA_CLIENT_DOWMLOAD);
		referee->Client_Data.error = 0;
	}
	else
	{
		referee->Client_Data.error = 1;
	}
}

/*客户端接受信息*/
static void CLIENT_MAP_DATA(REFEREE_t *referee, unsigned char k)
{
		memcpy(referee->RealData, referee->RefereeData + k, DATA_CLIENT_RECEIVE + 9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_CLIENT_RECEIVE + 9) == 1)
	{
		memcpy(&referee->ClientMapData, referee->RealData + 7, DATA_CLIENT_RECEIVE);
		referee->ClientMapData.error = 0;
	}
	else
	{
		referee->ClientMapData.error = 1;
	}
}
