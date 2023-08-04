/**
*****************************��ݸ��ѧԺACEʵ����*****************************
* @file 		bsp_referee.c
*
* @brief 		��������ϵͳ��ʼ��������ϵͳ���ݻ�ȡ������ϵͳͨѶЭ��Ľ���
* @author   Ҷ���
* @note  		
* @history  ȫ��������֧��hal��
* @version  1.0

@verbatim
==============================================================================

ECF_referee_uart_init()������ʼ��
REFEREE_UART_IRQHandler(UART_HandleTypeDef *huart)����uart6�жϾ�����

==============================================================================
@endverbatim
*****************************��ݸ��ѧԺACEʵ����******************************/


#include "bsp_referee.h"

#define RX_Buffer_Num 256

extern UART_HandleTypeDef huart6;

// �������ݻ��棬������������
uint8_t Referee_RX_Buffer[2][RX_Buffer_Num];
uint8_t Referee_Data[256];

REFEREE_t referee;

static uint16_t this_time_rx_len  = 0;

/*�������ݽ������ݴ���*/
void RefereeDataDeal(REFEREE_t *referee);
/*����״̬*/
static void GAME_STATUS(REFEREE_t *referee, unsigned char k);
/*�����������*/
static void GAME_RESULT(REFEREE_t *referee, unsigned char k);
/*������Ѫ��״̬����*/
static void ROBOT_HP(REFEREE_t *referee, unsigned char k);
/*����վ������ʶ*/
static void SUPPLY_PROJECTILE_ACTION(REFEREE_t *referee, unsigned char k);
/*���о�������*/
static void REFEREE_WARNING(REFEREE_t *referee, unsigned char k);
/*���ڷ���ڵ���ʱ*/
static void DART_REAMAINING_TIME(REFEREE_t *referee, unsigned char k);
/*������״̬*/
static void ROBOT_STATUS(REFEREE_t *referee, unsigned char k);
/*��������*/
static void POWER_HEAT(REFEREE_t *referee, unsigned char k);
/*������λ������*/
static void ROBOT_POSITION(REFEREE_t *referee, unsigned char k);
/*����������*/
static void ROBOT_BUFF(REFEREE_t *referee, unsigned char k);
/*�����¼�����*/
static void EVENT_DATA(REFEREE_t *referee, unsigned char k);
/*���˻�����״̬����*/
static void UAV_ENERGY_TIME(REFEREE_t *referee, unsigned char k);
/*�˺�״̬����*/
static void HURT_DATA(REFEREE_t *referee, unsigned char k);
/*ʵʱ�������*/
static void SHOOT_DATA(REFEREE_t *referee, unsigned char k);
/*ʣ�൯��ͽ������*/
static void BULLET_DATA(REFEREE_t *referee, unsigned char k);
/*������RFID״̬*/
static void RFID_STATUS(REFEREE_t *referee, unsigned char k);
/*���ڻ����˿ͻ���ָ��*/
static void DART_CLIENT(REFEREE_t *referee, unsigned char k);
/*�������ݽ�����Ϣ*/
static void INTERACT_HEADER(REFEREE_t *referee, unsigned char k);
/*�ͻ����·���Ϣ*/
static void CLIENT_DATA(REFEREE_t *referee, unsigned char k);
/*�ͻ��˽�����Ϣ*/
static void CLIENT_MAP_DATA(REFEREE_t *referee, unsigned char k);
	
	
REFEREE_t *Get_referee_Address(void)
{
	return &referee;
}

/************************** Dongguan-University of Technology*-ACE**************************
* @brief �Զ����һ�������жϣ�����stm32f4xx_it.c���û��Զ��崮���ж�����
*
* @param huart
************************** Dongguan-University of Technology*-ACE***************************/

void referee_uart_init(UART_HandleTypeDef *huart,uint8_t *rx_buffer1, uint8_t *rx_buffer2, uint8_t buff_num)
{
	    // ʹ��DMA���ڽ���
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
    // ʹ�ܿ����ж�
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    //    //����DMA���䣬������1�����ݰ��˵�recvive_buff��
    //    HAL_UART_Receive_DMA(&huart1, sbus_rx_buf[0], 36 );
    // ʧЧDMA
    __HAL_DMA_DISABLE(huart->hdmarx);
    while (huart->hdmarx->Instance->CR & DMA_SxCR_EN) {
    __HAL_DMA_DISABLE(huart->hdmarx);
    }
    huart->hdmarx->Instance->PAR = (uint32_t) & (huart->Instance->DR);
    // �ڴ滺����1
    huart->hdmarx->Instance->M0AR = (uint32_t)(rx_buffer1);
    // �ڴ滺����2
    huart->hdmarx->Instance->M1AR = (uint32_t)(rx_buffer2);
    // ���ݳ���
    huart->hdmarx->Instance->NDTR = buff_num;
    // ʹ��˫������
    SET_BIT(huart->hdmarx->Instance->CR, DMA_SxCR_DBM);
    // ʹ��DMA
    __HAL_DMA_ENABLE(huart->hdmarx);
}

void ECF_referee_uart_init()
{
	referee_uart_init(&huart6 ,Referee_RX_Buffer[0] ,Referee_RX_Buffer[1], 255);
}

void REFEREE_UART_IRQHandler(UART_HandleTypeDef *huart) 
	{
		//SR�Ĵ�����״̬�Ĵ�����������UART_FLAG_RXNE��00010100����������1����˵���н��յ�����
		if (huart->Instance->SR & UART_FLAG_RXNE) // ���յ�����
		{
		__HAL_UART_CLEAR_PEFLAG(huart);
		} else  
		if (huart->Instance->SR & UART_FLAG_IDLE) // ���ڴ��ڿ���״̬  ��UART_FLAG_IDLE =
														// 0��δ��⵽������· 1����⵽������·��
		{ // �ڿ����ж����ж�����֡�Ĵ����Ƿ���ȷ
		// �����ڿ�ʼ�������ݺ󣬼�⵽1�ֽ����ݵ�ʱ����û�����ݷ��ͣ�����Ϊ���ڿ����ˡ�

		__HAL_UART_CLEAR_PEFLAG(huart);

			if ((huart->hdmarx->Instance->CR & DMA_SxCR_CT) == RESET) 
			{
				/* Current memory buffer used is Memory 0 */
	
				// disable DMA
				// ʧЧDMA
				__HAL_DMA_DISABLE(huart->hdmarx);
	
				// get receive data length, length = set_data_length - remain_length
				// ��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
				this_time_rx_len = RX_Buffer_Num - huart->hdmarx->Instance->NDTR;
				// reset set_data_lenght
				// �����趨���ݳ���
				huart->hdmarx->Instance->NDTR = RX_Buffer_Num;
	
				// set memory buffer 1
				// �趨������1
				huart->hdmarx->Instance->CR |= DMA_SxCR_CT;
	
				// enable DMA
				// ʹ��DMA
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
				// ʧЧDMA
				__HAL_DMA_DISABLE (huart->hdmarx);
	
				// get receive data length, length = set_data_length - remain_length
				// ��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
				this_time_rx_len = RX_Buffer_Num - huart->hdmarx->Instance->NDTR;
	
				// reset set_data_lenght
				// �����趨���ݳ���
				huart->hdmarx->Instance->NDTR = RX_Buffer_Num ; 
	
				// set memory buffer 0
				// �趨������0
				huart->hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
	
				// enable DMA
				// ʹ��DMA
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

/*�������ݽ������ݴ���*/
void RefereeDataDeal(REFEREE_t *referee)
{
	uint8_t i;
	for (i = 0; i < referee->DataLen; i++)
	{
		if (referee->RefereeData[i] == 0xA5) //֡ͷ
		{
			if (Verify_CRC8_Check_Sum(referee->RefereeData, HEADER_LEN) == 1) //CRC8У��
			{
				referee->RealLen = ((referee->RefereeData[i + 1]) | (referee->RefereeData[i + 2] << 8));					  //���ݳ���
				referee->Cmd_ID = ((referee->RefereeData[i + HEADER_LEN]) | (referee->RefereeData[i + HEADER_LEN + 1] << 8)); //������ID

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
					EVENT_DATA(referee, i);             //��RFID��������
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
					ROBOT_BUFF(referee, i);   //��RFID��������
					i = i + (DATA_BUFF_LEN + 9) - 1;
					break;
				case ID_AERIAL_ROBOT_ENERGY:
					UAV_ENERGY_TIME(referee, i);   //�����������˻�
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
					RFID_STATUS(referee, i);           //��RFID��������
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

/*����״̬����*/
static void GAME_STATUS(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_STATUS_LEN + 9); //����ת��

	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_STATUS_LEN + 9) == 1) //CRC16У��
	{
		memcpy(&referee->Game_Status, referee->RealData + 7, DATA_STATUS_LEN);
		referee->Game_Status.error = 0;
	}
	else
	{
		referee->Game_Status.error = 1;
	}
}

/*�����������*/
static void GAME_RESULT(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_RESULT_LEN + 9); //����ת��

	if (Verify_CRC16_Check_Sum(referee->RealData,  DATA_RESULT_LEN + 9) == 1) //CRC16У��
	{
		memcpy(&referee->Game_Result, referee->RealData + 7, DATA_STATUS_LEN);
		referee->Game_Result.error = 0;
	}
	else
	{
		referee->Game_Result.error = 1;
	}
}

/*������Ѫ��״̬����*/
static void ROBOT_HP(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_ROBOT_HP_LEN + 9); //����ת��

	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_ROBOT_HP_LEN + 9) == 1) //CRC16У��
	{
		memcpy(&referee->Robot_HP, referee->RealData + 7, DATA_STATUS_LEN);
		referee->Robot_HP.error = 0;
	}
	else
	{
		referee->Robot_HP.error = 1;
	}
}

/*�����¼�����*/
static void EVENT_DATA(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_EVENT_DATA_LEN + 9); //����ת��
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

/*����վ������ʶ*/
static void SUPPLY_PROJECTILE_ACTION(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_SUPPLY_PROJECTILE_ACTION_LEN + 9); //����ת��
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

/*���о�������*/
static void REFEREE_WARNING(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_REFEREE_WARNING_LEN + 9); //����ת��
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_REFEREE_WARNING_LEN + 9) == 1)  //CRC16У��
	{
		memcpy(&referee->Referee_Warning, referee->RealData + 7, DATA_REFEREE_WARNING_LEN);
		referee->Referee_Warning.error = 0;
	}
	else
	{
		referee->Referee_Warning.error = 1;
	}
}

/*���ڷ���ڵ���ʱ*/
static void DART_REAMAINING_TIME(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_DART_REMAINING_TIME_LEN + 9); //����ת��
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_DART_REMAINING_TIME_LEN + 9) == 1)  //CRC16У��
	{
		memcpy(&referee->Dart_Remaining_Time, referee->RealData + 7, DATA_DART_REMAINING_TIME_LEN);
		referee->Dart_Remaining_Time.error = 0;
	}
	else
	{
		referee->Dart_Remaining_Time.error = 1;
	}
}

/*������״̬����*/
static void ROBOT_STATUS(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_ROBOT_STATUS_LEN + 9); //����ת��
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_ROBOT_STATUS_LEN + 9) == 1)  //CRC16У��
	{
		memcpy(&referee->Robot_Status, referee->RealData + 7, DATA_ROBOT_STATUS_LEN);
		referee->Robot_Status.error = 0;
	}
	else
	{
		referee->Robot_Status.error = 1;
	}
}

/*������������*/
static void POWER_HEAT(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_POWER_HEAT_DATA_LEN + 9); //����ת��
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_POWER_HEAT_DATA_LEN + 9) == 1)  //CRC16У��
	{
		memcpy(&referee->Power_Heat, referee->RealData + 7, DATA_POWER_HEAT_DATA_LEN);
		referee->Power_Heat.error = 0;
	}
	else
	{
		referee->Power_Heat.error = 1;
	}
}

/*������λ������*/
static void ROBOT_POSITION(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_ROBOT_POS_LEN + 9); //����ת��
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_ROBOT_POS_LEN + 9) == 1)  //CRC16У��
	{
		memcpy(&referee->Robot_Position, referee->RealData + 7, DATA_ROBOT_POS_LEN);
		referee->Robot_Position.error = 0;
	}
	else
	{
		referee->Robot_Position.error = 1;
	}
}

/*����������*/
static void ROBOT_BUFF(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_BUFF_LEN + 9); //����ת��
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_BUFF_LEN + 9) == 1)  //CRC16У��
	{
		memcpy(&referee->Buff, referee->RealData + 7, DATA_BUFF_LEN);
		referee->Buff.error = 0;
	}
	else
	{
		referee->Buff.error = 1;
	}
}

/*���˻�����״̬����*/
static void UAV_ENERGY_TIME(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_AERIAL_ROBOT_ENERGY_LEN + 9); //����ת��
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

/*�˺�״̬����*/
static void HURT_DATA(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_ROBOT_HURT_LEN + 9); //����ת��
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

/*�ӵ�ʣ������*/
static void BULLET_DATA(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_BULLET_REMAINING_LEN + 9); //����ת��
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

/*ʵʱ�������*/
static void SHOOT_DATA(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_SHOOT_DATA_LEN + 9); //����ת��
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

/*������RFID״̬*/
static void RFID_STATUS(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_RFID_STATUS_LEN + 9); //����ת��
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

/*���ڻ����˿ͻ���ָ��*/
static void DART_CLIENT(REFEREE_t *referee, unsigned char k)
{
		memcpy(referee->RealData, referee->RefereeData + k, DATA_DART_CLIENT_CMD_LEN +9); //����ת��
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

/*�������ݽ�����Ϣ*/
static void INTERACT_HEADER(REFEREE_t *referee, unsigned char k)
{
		memcpy(referee->RealData, referee->RefereeData + k, DATA_DIY_CONTROLLER + 9); //����ת��
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

/*�ͻ����·���Ϣ*/
static void CLIENT_DATA(REFEREE_t *referee, unsigned char k)
{
		memcpy(referee->RealData, referee->RefereeData + k, DATA_CLIENT_DOWMLOAD + 9); //����ת��
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

/*�ͻ��˽�����Ϣ*/
static void CLIENT_MAP_DATA(REFEREE_t *referee, unsigned char k)
{
		memcpy(referee->RealData, referee->RefereeData + k, DATA_CLIENT_RECEIVE + 9); //����ת��
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
