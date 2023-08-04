#ifndef __BSP_REFEREE_H
#define __BSP_REFEREE_H

#include "CRC.h"
#include "stdint.h"
#include "string.h"
#include "stm32f4xx_hal.h"

#define NULL 0
#define Referee_Data_len 128

//֡ͷ����
#define HEADER_LEN 5
//ָ���
#define CMDID_LEN 2
//CRC�����볤��1
#define CRC16_LEN 2

//���ݶγ���
#define DATA_STATUS_LEN													11						//!����״̬���ݳ���(�ٷ�����)
#define DATA_RESULT_LEN													1						 	//����������ݳ���
#define DATA_ROBOT_HP_LEN												32					 	//!����������Ѫ�����ݳ���(�ٷ�����)
//#define DATA_DART_STATUS_LEN									3							//���ڷ���״̬����
#define DATA_EVENT_DATA_LEN											4					 		//�����¼����ݳ���
#define DATA_SUPPLY_PROJECTILE_ACTION_LEN				4		 					//!���ز���վ������ʶ���ݳ���(�ٷ�����)
#define DATA_REFEREE_WARNING_LEN								2				 			//���о������ݳ���
#define DATA_DART_REMAINING_TIME_LEN						1			 				//���ڷ���ڵ���ʱ
#define DATA_ROBOT_STATUS_LEN										27				 		//!������״̬����(�ٷ�����)
#define DATA_POWER_HEAT_DATA_LEN								16						//!ʵʱ������������(�ٷ�����)
#define DATA_ROBOT_POS_LEN											16					 	//������λ������
#define DATA_BUFF_LEN														1							//��������������
#define DATA_AERIAL_ROBOT_ENERGY_LEN						2							//!���л���������״̬����,ֻ�п��л��������ط���(�ٷ�����)
#define DATA_ROBOT_HURT_LEN											1							//�˺�״̬����
#define DATA_SHOOT_DATA_LEN											7					 		//!ʵʱ�������(�ٷ�����)
#define DATA_BULLET_REMAINING_LEN								6							//!�ӵ�ʣ�෢����(�ٷ�����)
#define DATA_RFID_STATUS_LEN										4							//������ RFID ״̬
#define DATA_DART_CLIENT_CMD_LEN								6							//���ڻ����˿ͻ���ָ����
//#define DATA_STUDENT_INTERACTIVE_HEADER_DATA  6             //UI
#define DATA_DIY_CONTROLLER                     30            //�Զ��������
#define DATA_CLIENT_DOWMLOAD                    15            //С��ͼ�·�λ����Ϣ
#define DATA_PICTURE_TRANSMISSION               12            //ͼ��ң����Ϣ
#define DATA_CLIENT_RECEIVE                     10            //С��ͼ����λ����Ϣ
 
//������ID
#define ID_STATE 																0x0001				//����״̬����
#define ID_RESULT 															0x0002				//�����������
#define ID_ROBOT_HP 														0x0003				//���������˻�����Ѫ������
//#define ID_DART_STATUS 												0x0004				//���ڷ���״̬
#define ID_EVENT_DATA 													0x0101				//�����¼�����
#define ID_SUPPLY_PROJECTILE_ACTION 						0x0102	   		//���ز���վ������ʶ����
#define ID_SUPPLY_PROJECTILE_BOOKING 						0x0103	   		//���ز���վԤԼ�ӵ�����
#define ID_REFEREE_WARNING		 									0x0104			  //���о�������
#define ID_DART_REMAINING_TIME 									0x0105		   	//���ڷ���ڵ���ʱ
#define ID_ROBOT_STATE 													0x0201				//������״̬����
#define ID_POWER_HEAT_DATA	 										0x0202			  //ʵʱ������������
#define ID_ROBOT_POS 														0x0203				//������λ������
#define ID_BUFF 																0x0204				//��������������
#define ID_AERIAL_ROBOT_ENERGY 									0x0205		   	//���л���������״̬����
#define ID_ROBOT_HURT 													0x0206				//�˺�״̬����
#define ID_SHOOT_DATA 													0x0207				//ʵʱ�������
#define ID_BULLET_REMAINING											0x0208			  //�ӵ�ʣ�෢����
#define ID_RFID_STATUS 													0x0209				//������ RFID ״̬
#define ID_DART_CLIENT_CMD                      0x020A        //���ڻ����˿ͻ���ָ������
//#define ID_STUDENT_INTERACTIVE_HEADER_DATA    0x0301       // UI
#define ID_DIY_CONTROLLER  											0x0302 				//�Զ��������
#define ID_CLIENT_DOWMLOAD  										0x0303     		//С��ͼ�·�λ����Ϣ
#define ID_PICTURE_TRANSMISSION 								0x0304				//ͼ��ң����Ϣ
#define ID_CLIENT_RECEIVE  											0x0305     		//С��ͼ����λ����Ϣ



/*����״̬����*/
typedef __packed struct
{
	uint8_t	 game_type : 4;
	uint8_t  game_progress : 4;
	uint16_t stage_remain_time;
	uint64_t SyncTimeStamp;
	uint8_t  error;
} Game_Type_Data;

/*�����������*/
typedef __packed struct
{
	uint8_t winner;
	uint8_t error;
}	Game_Result;

/*Ѫ������*/
typedef __packed struct
{
	uint16_t red_1_robot_HP;
	uint16_t red_2_robot_HP;
	uint16_t red_3_robot_HP;
	uint16_t red_4_robot_HP;
	uint16_t red_5_robot_HP;
	uint16_t red_7_robot_HP;
	uint16_t red_outpost_HP;
	uint16_t red_base_HP;
	uint16_t blue_1_robot_HP;
	uint16_t blue_2_robot_HP;
	uint16_t blue_3_robot_HP;
	uint16_t blue_4_robot_HP;
	uint16_t blue_5_robot_HP;
	uint16_t blue_7_robot_HP;
	uint16_t blue_outpost_HP;
	uint16_t blue_base_HP;
	uint8_t error;
} Robot_Hp_Data;

/*�����¼�����*/
typedef __packed struct
{
  uint32_t event_type;
  uint32_t BloodPoint_1 : 1;
  uint32_t BloodPoint_2 : 1;
  uint32_t BloodPoint_3 : 1;	
  uint32_t ENERGY : 3;
  uint32_t Annular_HighLand : 1;
  uint32_t R3_or_B3_HighLand : 1;
  uint32_t R4_or_B4_HighLand : 1;
  uint8_t error;
} Area_Data;

/*����վ������ʶ*/
typedef __packed struct
{
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id;
	uint8_t supply_projectile_step;
	uint8_t supply_projectile_num;
	uint8_t error;
} Supply_Data;

/*���о�����Ϣ*/
typedef __packed struct
{
	uint8_t level;
	uint8_t foul_robot_id;
	uint8_t error;
}Referee_Warning;

/*���ڷ��������*/
typedef __packed struct
{
	uint8_t dart_remaining_time;
	uint8_t error;
} Dart_Launch_Data;

/*������״̬����*/
typedef __packed struct
{
	uint8_t robot_id;						 //1���췽Ӣ�ۻ����ˣ�101������Ӣ�ۻ����ˣ�
	uint8_t robot_level;					 //�����˵ȼ���1��һ����2��������3��������
	uint16_t remain_HP;						 //������ʣ��Ѫ��
	uint16_t max_HP;						 //����������Ѫ��
	uint16_t shooter_id1_17mm_cooling_rate;	 //������ 1 �� 17mm ǹ��ÿ����ȴֵ
	uint16_t shooter_id1_17mm_cooling_limit; //������ 1 �� 17mm ǹ����������
	uint16_t shooter_id1_17mm_speed_limit;	 //������ 1 �� 17mm ǹ�������ٶ� ��λ m/s
	uint16_t shooter_id2_17mm_cooling_rate;	 //������ 2 �� 17mm ǹ��ÿ����ȴֵ
	uint16_t shooter_id2_17mm_cooling_limit; //������ 2 �� 17mm ǹ����������
	uint16_t shooter_id2_17mm_speed_limit;	 //������ 2 �� 17mm ǹ�������ٶ� ��λ m/s
	uint16_t shooter_id1_42mm_cooling_rate;	 //������ 42mm ǹ��ÿ����ȴֵ
	uint16_t shooter_id1_42mm_cooling_limit; //������ 42mm ǹ����������
	uint16_t shooter_id1_42mm_speed_limit;	 //������ 42mm ǹ�������ٶ� ��λ m/s
	uint16_t chassis_power_limit;			 //�����˵��̹�����������
	uint8_t mains_power_gimbal_output : 1;	 //gimbal ������� 1 Ϊ�� 24V �����0 Ϊ�� 24v �����
	uint8_t mains_power_chassis_output : 1;	 //chassis �������1 Ϊ�� 24V �����0 Ϊ�� 24v ���
	uint8_t mains_power_shooter_output : 1;	 //shooter �������1 Ϊ�� 24V �����0 Ϊ�� 24v �����
	uint8_t error;
} Robot_Situation_Data;

/*������������*/
typedef __packed struct
{
	uint16_t chassis_volt;
	uint16_t chassis_current;
	float chassis_power;
	uint16_t chassis_power_buffer;
	uint16_t shooter_id1_17mm_cooling_heat;
	uint16_t shooter_id2_17mm_cooling_heat;
	uint16_t shooter_id1_42mm_cooling_heat;
	uint8_t error;
} Robot_Power_Heat_Data;

/*������λ��*/
typedef __packed struct
{
	float x;
	float y;
	float z;
	float yaw;
	uint8_t error;
} Robot_Position_Data;

/*��������������*/
typedef __packed struct
{
  uint8_t power_rune_buff;
	uint8_t error;
} Area_Buff_Data;

/*���л���������״̬*/
typedef __packed struct
{
	uint8_t attack_time;
	uint8_t error;
} UAV_Data;

/*�˺�״̬*/
typedef __packed struct
{
	uint8_t armor_id : 4;
	uint8_t hurt_type : 4;
	uint8_t error;
} Robot_Hurt_Data;

/*ʵʱ�����Ϣ*/
typedef __packed struct
{
	uint8_t bullet_type;
	uint8_t shooter_id;
	uint8_t bullet_freq;
	float bullet_speed;
	uint8_t error;
} Robot_Shoot_Data;

/*�ӵ�ʣ�෢����*/
typedef __packed struct
{
	uint16_t bullet_remaining_num_17mm; //17mm �ӵ�ʣ�෢����Ŀ
	uint16_t bullet_remaining_num_42mm; //42mm �ӵ�ʣ�෢����Ŀ
	uint16_t coin_remaining_num;		//ʣ��������
	uint8_t  error;
} Robot_RaminingBullet_Data;

/*RFID״̬*/
typedef __packed struct
{
	uint32_t BaseLand : 1;
	uint32_t HighLand : 1;
	uint32_t Energy : 1;
	uint32_t OverSlope : 1;
	uint32_t Sentry : 1;
	uint32_t Resources : 1;
	uint32_t BloodPoint : 1;
	uint32_t BloodCard : 1;
	uint32_t other : 24;
	uint8_t error;
} RFID_Situation_Data;

/*���ڻ����˿ͻ���ָ������*/
typedef __packed struct
{
  uint8_t dart_launch_opening_status;
  uint8_t dart_attack_target;
  uint16_t target_change_time;
  uint16_t operate_launch_cmd_time;
	uint8_t error;
} Dart_Client_Cmd;

/*�������ݽ�����Ϣ*/
typedef __packed struct
{
  uint16_t data_cmd_id;
  uint16_t sender_ID;
  uint16_t receiver_ID;
//uint16_t data[];	
	uint8_t error;
} student_interactive_header_data_t;

/*ѧ�������˼�ͨ��*/
//δʹ�ù���������
//������UI�ͳ���ͨ��
typedef __packed struct
{
	uint8_t data[1];
	uint8_t error;
} robot_interactive_data_t;

/*�ͻ����·���Ϣ*/
typedef __packed struct
{
	float target_position_x;
	float target_position_y;
	float target_position_z;
	uint8_t commd_keyboard;
	uint16_t target_robot_ID;
	uint8_t error;
} Robot_Command;

/*�ͻ��˽�����Ϣ*/
//�״�վ���͵�������Ϣ���Ա����м����������ڵ�һ�ӽ�С��ͼ������
typedef __packed struct
{
	uint16_t target_robot_ID;
	float target_position_x;
	float target_position_y;
	uint8_t error;
} Client_Map_Command_Data;

/*����ϵͳ����*/
typedef __packed struct
{
	uint8_t RefereeData[256];
	uint8_t RealData[45];
	int16_t DataLen;
	int16_t RealLen;
	int16_t Cmd_ID;
	uint8_t RECEIVE_FLAG;
	Game_Type_Data 															Game_Status;
	Game_Result                 							  Game_Result;
	Robot_Hp_Data 															Robot_HP;
	Area_Data 																	Event_Data;
	Supply_Data 																Supply_Action;
	Referee_Warning             							  Referee_Warning;
	Dart_Launch_Data 														Dart_Remaining_Time;
	Robot_Situation_Data 												Robot_Status;
	Robot_Power_Heat_Data 											Power_Heat;
	Robot_Position_Data         							  Robot_Position;
	Area_Buff_Data 															Buff;
	UAV_Data 																		Aerial_Energy;
	Robot_Hurt_Data 														Robot_Hurt;
	Robot_Shoot_Data 														Shoot_Data;
	Robot_RaminingBullet_Data 									Bullet_Num;
	RFID_Situation_Data 												RFID_Status;
	Dart_Client_Cmd             							  Dart_Client;
	student_interactive_header_data_t           Interact_Header;
	robot_interactive_data_t                    Interact_Data;
	Robot_Command         			     					  Client_Data;
	Client_Map_Command_Data          					  ClientMapData;
} REFEREE_t;

//����ϵͳ��ʼ��
void referee_uart_init(UART_HandleTypeDef *huart,uint8_t *rx_buffer1, uint8_t *rx_buffer2, uint8_t buff_num);
void ECF_referee_uart_init(void);
void REFEREE_UART_IRQHandler(UART_HandleTypeDef *huart);
REFEREE_t *Get_referee_Address(void);

#endif
