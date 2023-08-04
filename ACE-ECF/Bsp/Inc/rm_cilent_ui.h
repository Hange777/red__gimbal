#ifndef __RM_CILENT_UI_H__
#define __RM_CILENT_UI_H__
#include "struct_typedef.h"
#include "stdarg.h"
#include "stm32f4xx_hal.h"
#define Robot_ID UI_Data_RobotID_BStandard2
#define Cilent_ID UI_Data_CilentID_BStandard2 //�����˽�ɫ����


#pragma pack(1) //��1�ֽڶ���

#define NULL 0
#define __FALSE 100

/****************************��ʼ��־*********************/
#define UI_SOF 0xA5
/****************************CMD_ID����********************/
#define UI_CMD_Robo_Exchange 0x0301
/****************************����ID����********************/
#define UI_Data_ID_Del 0x100
#define UI_Data_ID_Draw1 0x101
#define UI_Data_ID_Draw2 0x102
#define UI_Data_ID_Draw5 0x103
#define UI_Data_ID_Draw7 0x104
#define UI_Data_ID_DrawChar 0x110
/****************************�췽������ID********************/
#define UI_Data_RobotID_RHero 1
#define UI_Data_RobotID_REngineer 2
#define UI_Data_RobotID_RStandard1 3
#define UI_Data_RobotID_RStandard2 4
#define UI_Data_RobotID_RStandard3 5
#define UI_Data_RobotID_RAerial 6
#define UI_Data_RobotID_RSentry 7
#define UI_Data_RobotID_RRadar 9
/****************************����������ID********************/
#define UI_Data_RobotID_BHero 101
#define UI_Data_RobotID_BEngineer 102
#define UI_Data_RobotID_BStandard1 103
#define UI_Data_RobotID_BStandard2 104
#define UI_Data_RobotID_BStandard3 105
#define UI_Data_RobotID_BAerial 106
#define UI_Data_RobotID_BSentry 107
#define UI_Data_RobotID_BRadar 109
/**************************�췽������ID************************/
#define UI_Data_CilentID_RHero 0x0101
#define UI_Data_CilentID_REngineer 0x0102
#define UI_Data_CilentID_RStandard1 0x0103
#define UI_Data_CilentID_RStandard2 0x0104
#define UI_Data_CilentID_RStandard3 0x0105
#define UI_Data_CilentID_RAerial 0x0106
/***************************����������ID***********************/
#define UI_Data_CilentID_BHero 0x0165
#define UI_Data_CilentID_BEngineer 0x0166
#define UI_Data_CilentID_BStandard1 0x0167
#define UI_Data_CilentID_BStandard2 0x0168
#define UI_Data_CilentID_BStandard3 0x0169
#define UI_Data_CilentID_BAerial 0x016A
/***************************ɾ������***************************/
#define UI_Data_Del_NoOperate 0
#define UI_Data_Del_Layer 1
#define UI_Data_Del_ALL 2
/***************************ͼ�����ò���__ͼ�β���********************/
#define UI_Graph_ADD 1
#define UI_Graph_Change 2
#define UI_Graph_Del 3
/***************************ͼ�����ò���__ͼ������********************/
#define UI_Graph_Line 0      //ֱ��
#define UI_Graph_Rectangle 1 //����
#define UI_Graph_Circle 2    //��Բ
#define UI_Graph_Ellipse 3   //��Բ
#define UI_Graph_Arc 4       //Բ��
#define UI_Graph_Float 5     //������
#define UI_Graph_Int 6       //����
#define UI_Graph_Char 7      //�ַ���
/***************************ͼ�����ò���__ͼ����ɫ********************/
#define UI_Color_Main 0 //������ɫ
#define UI_Color_Yellow 1
#define UI_Color_Green 2
#define UI_Color_Orange 3
#define UI_Color_Purplish_red 4 //�Ϻ�ɫ
#define UI_Color_Pink 5
#define UI_Color_Cyan 6 //��ɫ
#define UI_Color_Black 7
#define UI_Color_White 8

#define FRAMEHEAD_LEN 7
#define UI_DATA_OPERATE_LEN 6
#define GRAPH_DATA_LEN 15
#define STRING_DATA_LEN 45
#define ALL_GRAPH_LEN 30
#define ALL_STRING_LEN 60



typedef struct
{
    uint8_t SOF;          //��ʼ�ֽ�,�̶�0xA5
    uint16_t Data_Length; //֡���ݳ���
    uint8_t Seq;          //�����
    uint8_t CRC8;         //CRC8У��ֵ
    uint16_t CMD_ID;      //����ID
} UI_Packhead;       //֡ͷ

typedef struct
{
    uint16_t Data_ID;     //����ID
    uint16_t Sender_ID;   //������ID
    uint16_t Receiver_ID; //������ID
} UI_Data_Operate;   //��������֡

typedef struct
{
    uint8_t Delete_Operate; //ɾ������
    uint8_t Layer;          //ɾ��ͼ��
} UI_Data_Delete;      //ɾ��ͼ��֡

typedef struct
{
    uint8_t graphic_name[3];
    uint32_t operate_tpye : 3;
    uint32_t graphic_tpye : 3;
    uint32_t layer : 4;
    uint32_t color : 4;
    uint32_t start_angle : 9;
    uint32_t end_angle : 9;
    uint32_t width : 10;
    uint32_t start_x : 11;
    uint32_t start_y : 11;
    int graph_Float; //��������
} Float_Data;

typedef struct
{
    uint8_t graphic_name[3];
    uint32_t operate_tpye : 3;
    uint32_t graphic_tpye : 3;
    uint32_t layer : 4;
    uint32_t color : 4;
    uint32_t start_angle : 9;
    uint32_t end_angle : 9;
    uint32_t width : 10;
    uint32_t start_x : 11;
    uint32_t start_y : 11;
    uint32_t radius : 10;
    uint32_t end_x : 11;
    uint32_t end_y : 11; //ͼ������
} Graph_Data;   //

typedef struct
{
    Graph_Data Graph_Control;
    char show_Data[30];
} String_Data; //��ӡ�ַ�������

void UI_Delete(uint8_t Del_Operate, uint8_t Del_Layer);
void Line_Draw(Graph_Data *image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y);
int UI_ReFresh(int cnt, ...);
unsigned char Get_CRC8_Check_Sum_UI(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8);
uint16_t Get_CRC16_Check_Sum_UI(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
void Circle_Draw(Graph_Data *image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t Graph_Radius);
void Rectangle_Draw(Graph_Data *image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y);
void Float_Draw(Float_Data *image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Size, uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, float Graph_Float);
void Char_Draw(String_Data *image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Size, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, const char *Char_Data);
void Arc_Draw(Graph_Data *image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_StartAngle, uint32_t Graph_EndAngle, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t x_Length, uint32_t y_Length);
extern void ui_client(char *char_data);
//ֻ��Ⱦһ��ͼ��(�򻯴���)
extern void My_Graph_Refresh(Graph_Data *Graph);
extern void My_Char_Refresh(String_Data string);
#endif
