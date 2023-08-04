#include "rm_cilent_ui.h"

#include "bsp_referee.h"

#include "stdlib.h"
#include "stdio.h"
#include "crc.h"


#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"


unsigned char UI_Seq = 0; //包序号

extern UART_HandleTypeDef huart6;
extern REFEREE_t referee;

/****************************************串口驱动映射************************************/
void UI_SendByte(unsigned char ch)
{
	HAL_UART_Transmit(&huart6,&ch,sizeof(unsigned char),1);
}

/********************************************删除操作*************************************
**参数：Del_Operate  对应头文件删除操作
        Del_Layer    要删除的层 取值0-9
*****************************************************************************************/
void UI_Delete(uint8_t Del_Operate, uint8_t Del_Layer)
{
    unsigned char *framepoint;   //读写指针
    uint16_t frametail = 0xFFFF; //CRC16校验值
    int loop_control;            //For函数循环控制

    UI_Packhead framehead;
    UI_Data_Operate datahead;
    UI_Data_Delete del;

    framepoint = (unsigned char *)&framehead;

    framehead.SOF = UI_SOF;
    framehead.Data_Length = 8;
    framehead.Seq = UI_Seq;
    framehead.CRC8 = Get_CRC8_Check_Sum(framepoint, 4, 0xFF);
    framehead.CMD_ID = UI_CMD_Robo_Exchange; //填充包头数据

    datahead.Data_ID = UI_Data_ID_Del;
    datahead.Sender_ID = Robot_ID;
    datahead.Receiver_ID = Cilent_ID; //填充操作数据

    del.Delete_Operate = Del_Operate;
    del.Layer = Del_Layer; //控制信息

    frametail = Get_CRC16_Check_Sum(framepoint, sizeof(framehead), frametail);
    framepoint = (unsigned char *)&datahead;
    frametail = Get_CRC16_Check_Sum(framepoint, sizeof(datahead), frametail);
    framepoint = (unsigned char *)&del;
    frametail = Get_CRC16_Check_Sum(framepoint, sizeof(del), frametail); //CRC16校验值计算

    framepoint = (unsigned char *)&framehead;

    for (loop_control = 0; loop_control < sizeof(framehead); loop_control++)
    {
        UI_SendByte(*framepoint);
        framepoint++;
    }

    framepoint = (unsigned char *)&datahead;

    for (loop_control = 0; loop_control < sizeof(datahead); loop_control++)
    {
        UI_SendByte(*framepoint);
        framepoint++;
    }

    framepoint = (unsigned char *)&del;

    for (loop_control = 0; loop_control < sizeof(del); loop_control++)
    {
        UI_SendByte(*framepoint);
        framepoint++;
    } //发送所有帧

    framepoint = (unsigned char *)&frametail;

    for (loop_control = 0; loop_control < sizeof(frametail); loop_control++)
    {
        UI_SendByte(*framepoint);
        framepoint++; //发送CRC16校验值
    }
		
    UI_Seq++; //包序号+1
}
/************************************************绘制直线*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Start_x、Start_x    开始坐标
        End_x、End_y   结束坐标
**********************************************************************************************************/
void Line_Draw(
    Graph_Data *image,
    char imagename[3],
    uint32_t Graph_Operate,
    uint32_t Graph_Layer,
    uint32_t Graph_Color,
    uint32_t Graph_Width,
    uint32_t Start_x,
    uint32_t Start_y,
    uint32_t End_x,
    uint32_t End_y)
{
    int i;

    for (i = 0; i < 3 && imagename[i] != '\0'; i++)
        image->graphic_name[2 - i] = imagename[i];

    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->end_x = End_x;
    image->end_y = End_y;
}

/************************************************绘制矩形*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Start_x、Start_x    开始坐标
        End_x、End_y   结束坐标（对顶角坐标）
**********************************************************************************************************/
void Rectangle_Draw(
    Graph_Data *image,
    char imagename[3],
    uint32_t Graph_Operate,
    uint32_t Graph_Layer,
    uint32_t Graph_Color,
    uint32_t Graph_Width,
    uint32_t Start_x,
    uint32_t Start_y,
    uint32_t End_x,
    uint32_t End_y)
{
    int i;

    for (i = 0; i < 3 && imagename[i] != '\0'; i++)
        image->graphic_name[2 - i] = imagename[i];

    image->graphic_tpye = UI_Graph_Rectangle;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->end_x = End_x;
    image->end_y = End_y;
}

/************************************************绘制整圆*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Start_x、Start_x    圆心坐标
        Graph_Radius  图形半径
**********************************************************************************************************/
void Circle_Draw(
    Graph_Data *image,
    char imagename[3],
    uint32_t Graph_Operate,
    uint32_t Graph_Layer,
    uint32_t Graph_Color,
    uint32_t Graph_Width,
    uint32_t Start_x,
    uint32_t Start_y,
    uint32_t Graph_Radius)
{
    int i;

    for (i = 0; i < 3 && imagename[i] != '\0'; i++)
        image->graphic_name[2 - i] = imagename[i];

    image->graphic_tpye = UI_Graph_Circle;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->radius = Graph_Radius;
}

/************************************************绘制圆弧*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Graph_StartAngle,Graph_EndAngle    开始，终止角度
        Start_y,Start_y    圆心坐标
        x_Length,y_Length   x,y方向上轴长，参考椭圆
**********************************************************************************************************/
void Arc_Draw(
    Graph_Data *image,
    char imagename[3],
    uint32_t Graph_Operate,
    uint32_t Graph_Layer,
    uint32_t Graph_Color,
    uint32_t Graph_StartAngle,
    uint32_t Graph_EndAngle,
    uint32_t Graph_Width,
    uint32_t Start_x,
    uint32_t Start_y,
    uint32_t x_Length,
    uint32_t y_Length)
{
    int i;

    for (i = 0; i < 3 && imagename[i] != '\0'; i++)
        image->graphic_name[2 - i] = imagename[i];

    image->graphic_tpye = UI_Graph_Arc;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->start_angle = Graph_StartAngle;
    image->end_angle = Graph_EndAngle;
    image->end_x = x_Length;
    image->end_y = y_Length;
}

/************************************************绘制浮点型数据*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Graph_Size     字号
        Graph_Digit    小数位数
        Start_x、Start_x    开始坐标
        Graph_Float   要显示的变量
**********************************************************************************************************/
void Float_Draw(
    Float_Data *image,
    char imagename[3],
    uint32_t Graph_Operate,
    uint32_t Graph_Layer,
    uint32_t Graph_Color,
    uint32_t Graph_Size,
    uint32_t Graph_Digit,
    uint32_t Graph_Width,
    uint32_t Start_x,
    uint32_t Start_y,
    float Graph_Float)
{
    int i;

    for (i = 0; i < 3 && imagename[i] != '\0'; i++)
        image->graphic_name[2 - i] = imagename[i];

    image->graphic_tpye = UI_Graph_Float;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->start_angle = Graph_Size;
    image->end_angle = Graph_Digit;
    image->graph_Float = Graph_Float;
}

/************************************************绘制字符型数据*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Graph_Size     字号
        Start_x、Start_y    开始坐标
        *Char_Data          待发送字符串开始地址
**********************************************************************************************************/
void Char_Draw(
    String_Data *image,
    char imagename[3],
    uint32_t Graph_Operate,
    uint32_t Graph_Layer,
    uint32_t Graph_Color,
    uint32_t Graph_Size,
    uint32_t Graph_Width,
    uint32_t Start_x,
    uint32_t Start_y,
    const char *Char_Data)
{
    int i;

    for (i = 0; i < 3 && imagename[i] != '\0'; i++)
        image->Graph_Control.graphic_name[2 - i] = imagename[i];

    image->Graph_Control.graphic_tpye = UI_Graph_Char;
    image->Graph_Control.operate_tpye = Graph_Operate;
    image->Graph_Control.layer = Graph_Layer;
    image->Graph_Control.color = Graph_Color;
    image->Graph_Control.width = Graph_Width;
    image->Graph_Control.start_x = Start_x;
    image->Graph_Control.start_y = Start_y;
    image->Graph_Control.start_angle = Graph_Size;
    image->Graph_Control.end_angle = strlen(Char_Data);

    strcpy(image->show_Data,Char_Data);
}

//只渲染一个图形(简化代码)
void My_Graph_Refresh(Graph_Data *Graph)
{
    static uint8_t msg[ALL_GRAPH_LEN];
    UI_Packhead framehead;
    framehead.SOF = UI_SOF;
    framehead.Data_Length = UI_DATA_OPERATE_LEN + GRAPH_DATA_LEN;
    framehead.Seq = UI_Seq;
    framehead.CRC8 = Get_CRC8_Check_Sum((uint8_t *)&framehead, 4, 0xff);
    framehead.CMD_ID = 0x0301;

    UI_Data_Operate data_operate;
    data_operate.Data_ID = UI_Data_ID_Draw1;
    data_operate.Sender_ID = referee.Robot_Status.robot_id ;
    if (data_operate.Sender_ID == UI_Data_RobotID_BStandard1) //根据机器人ID选择发送
        data_operate.Receiver_ID = UI_Data_CilentID_BStandard1;
    else if (data_operate.Sender_ID == UI_Data_RobotID_RStandard1)
        data_operate.Receiver_ID = UI_Data_CilentID_RStandard1;
	else if (data_operate.Sender_ID == UI_Data_RobotID_BStandard2)
        data_operate.Receiver_ID = UI_Data_CilentID_BStandard2;
	else if (data_operate.Sender_ID == UI_Data_RobotID_RStandard2)
        data_operate.Receiver_ID = UI_Data_CilentID_RStandard2;
	else if (data_operate.Sender_ID == UI_Data_RobotID_BStandard3)
        data_operate.Receiver_ID = UI_Data_CilentID_BStandard3;
	else if (data_operate.Sender_ID == UI_Data_RobotID_RStandard3)
        data_operate.Receiver_ID = UI_Data_CilentID_RStandard3;

    memcpy(msg, &framehead, FRAMEHEAD_LEN);
    memcpy(msg + FRAMEHEAD_LEN, &data_operate, UI_DATA_OPERATE_LEN);
    memcpy(msg + FRAMEHEAD_LEN + UI_DATA_OPERATE_LEN, Graph, GRAPH_DATA_LEN);

    Append_CRC16_Check_Sum(msg, ALL_GRAPH_LEN);
  	HAL_UART_Transmit_DMA(&huart6,msg,ALL_GRAPH_LEN);
//  	uart4_dma_send(msg, ALL_GRAPH_LEN); 这里用发送api
    //DMA_printf(DMA1_Stream4, UART4, msg, ALL_GRAPH_LEN);
    UI_Seq++;
    vTaskDelay(10);
}

//只渲染一个字符串
void My_Char_Refresh(String_Data string)
{
    static uint8_t msg[ALL_STRING_LEN];
    UI_Packhead framehead;
    framehead.SOF = UI_SOF;
    framehead.Data_Length = UI_DATA_OPERATE_LEN + STRING_DATA_LEN;
    framehead.Seq = UI_Seq;
    framehead.CRC8 = Get_CRC8_Check_Sum((uint8_t *)&framehead, 4, 0xff);
    framehead.CMD_ID = 0x0301;

    UI_Data_Operate data_operate;
    data_operate.Data_ID = UI_Data_ID_DrawChar;
    data_operate.Sender_ID = referee.Robot_Status.robot_id ;
    if (data_operate.Sender_ID == UI_Data_RobotID_BStandard1) //根据机器人ID选择发送
        data_operate.Receiver_ID = UI_Data_CilentID_BStandard1;
    else if (data_operate.Sender_ID == UI_Data_RobotID_RStandard1)
        data_operate.Receiver_ID = UI_Data_CilentID_RStandard1;
	else if (data_operate.Sender_ID == UI_Data_RobotID_BStandard2)
        data_operate.Receiver_ID = UI_Data_CilentID_BStandard2;
	else if (data_operate.Sender_ID == UI_Data_RobotID_RStandard2)
        data_operate.Receiver_ID = UI_Data_CilentID_RStandard2;
	else if (data_operate.Sender_ID == UI_Data_RobotID_BStandard3)
        data_operate.Receiver_ID = UI_Data_CilentID_BStandard3;
	else if (data_operate.Sender_ID == UI_Data_RobotID_RStandard3)
        data_operate.Receiver_ID = UI_Data_CilentID_RStandard3;
		
		string.Graph_Control.end_angle = strlen(string.show_Data);
		memset(string.show_Data + string.Graph_Control.end_angle, 0, STRING_DATA_LEN - string.Graph_Control.end_angle);
    memcpy(msg, &framehead, FRAMEHEAD_LEN);
    memcpy(msg + FRAMEHEAD_LEN, &data_operate, UI_DATA_OPERATE_LEN);
    memcpy(msg + FRAMEHEAD_LEN + UI_DATA_OPERATE_LEN, &string, STRING_DATA_LEN);

    Append_CRC16_Check_Sum(msg, ALL_STRING_LEN);
		HAL_UART_Transmit_DMA(&huart6,msg,ALL_STRING_LEN);
//	uart4_dma_send(msg, ALL_STRING_LEN);这里用发送api
    UI_Seq++;
    vTaskDelay(10);
}



