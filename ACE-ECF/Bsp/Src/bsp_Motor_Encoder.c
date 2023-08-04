/************************** Dongguan-University of Technology -ACE**************************
* @file bsp_Motor_Encoder.c
* @brief
* @author pansyhou侯文辉 (1677195845lyb@gmail.com)
* @version 1.0
* @date 2022-11-24
*
*
* @history
* <table>
* Date       Version Author Description
* 2022-11-24   1.0   侯文辉  加入电机编码器处理
* 2022-12-6    1.5   侯文辉  加入布瑞特编码器控制
* @verbatim
* ==============================================================================
*  已经把常用的速度、位置、加速度、多圈编码、堵转检测巴拉巴拉的混进去了。
*  现在还加了个布瑞特的协议控制。
*  目前在结构体能用的部分：
*      Encode_Record_Val多圈编码器记录值
*      Radian转过的弧度
*      User_Radian弧度自定义，自己调PID的时候可以设置
*      Speed速度
*      AccSpeed加速度
*      position位置
*      State电机状态
*
*  使用方法：
*      将CAN_DATA_Encoder_Deal扔CAN接收部分
*      在初始化阶段加入Encoder_Init
*      done
*      需要将数值清零的可以用EncoderValZero(Encoder_t *Encoder)函数
*      该函数不会将编码器的编码器单圈码盘值设置为0
*
*      布瑞特的编码器
*          看手册，用函数。
*  注意事项：
*      改减速比的电机只能直接硬改结构体的gear_Ratio
*      Max_Block_Angle_Num、Max_Block_Num这两个都是调整堵转灵敏度的，需要自己在初始化阶段调整
*      CAN_ENCODER_NUM宏定义目前设置为10，方便工程有很多电机和编码器
*  最新版本已经将工程的去除Encoder的枚举里，他的编码器居然是在范围内的绝对值，那就不用我绝对值转多圈了
*
* ==============================================================================
* @endverbatim
************************** Dongguan-University of Technology -ACE***************************/
#include <string.h>
#include "bsp_Motor_Encoder.h"
#include "can.h"
//版本1是家华写的，用的是最原始的临界值判断的方案，但是感觉没有差值好使
//版本2是差值+溢出 计算的多圈编码器，频率是够用的

//函数声明
int16_t angle_limiting_int16(int16_t Angl_Err, int16_t lap_encoder);
int32_t check_codevalue(int32_t value, int16_t gear_Ratio, int16_t lap_encoder);
void Encoder_Struct_Zero(Encoder_t *Encoder);


Encoder_t CAN_Encoder[CAN_ENCODER_NUM];

/**
* 编码器初始化，返回指针，默认不开堵转检测，需要在初始化后手动开启，将Encoder结构体的Block_Detect_Enable置除0以外的
* @param Encoder_Type 编码器种类
* @param ch encoder编号（从1开始）
* @return
*/
Encoder_t *Encoder_Init(Encoder_Type_e Encoder_Type , uint8_t ch){

   if(ch>=CAN_ENCODER_NUM)return NULL;

   Encoder_t *Encoder=&CAN_Encoder[ch-1];
   Encoder_Struct_Zero(Encoder);
   Encoder->ch = ch;
   Encoder->Encoder_Type = Encoder_Type;
   Encoder->State = NORM;      //状态
   Encoder->Block_Detect_Enable = 0;
   switch (Encoder_Type) {
       case M3508: {
           Encoder->lap_encoder = 8192;//编码器单圈码盘值
           Encoder->gear_Ratio = 19;   //3508减速比
           Encoder->Max_Block_Angle_Num = 10;
           Encoder->Max_Block_Num = 256;
       }break;

       case GM6020:{
           Encoder->lap_encoder = 8192;//编码器单圈码盘值
           Encoder->gear_Ratio = 1;   //减速比
       }break;
       case M2006:{
           Encoder->lap_encoder = 8192;//编码器单圈码盘值
           Encoder->gear_Ratio = 36;   //减速比
       }break;
       default:
           break;
   }

   return Encoder;
}


/**
* CAN编码器处理函数
* @param position 位置
* @param speed 速度
* @param Encoder_Num 编码器编号（从1开始
*/
void CAN_DATA_Encoder_Deal(int16_t position, int16_t speed, uint8_t Encoder_Num) {
   //返回指针
   if(Encoder_Num>=CAN_ENCODER_NUM) return;
   Encoder_t *Encoder = &CAN_Encoder[Encoder_Num-1];
   Encoder->position = position;    //未处理的Can原始码盘
   /*速度处理*/
   Encoder->Speed[1] = speed;
   //加速度计算
   Encoder->AccSpeed = Encoder->Speed[1] - Encoder->Speed[0];
   Encoder->Speed[0] = Encoder->Speed[1];
   //第一次进入初始化position
   if (Encoder->Init_Flag == 0) {
       Encoder->Init_Flag = 1;
       Encoder->last_position = Encoder->position;
       Encoder->Encode_Actual_Val = Encoder->position;
   }

   //多圈码盘值，不做360度后归零处理
   volatile int erro=angle_limiting_int16(Encoder->position - Encoder->last_position,Encoder->lap_encoder); //差值累加

   Encoder->Encode_Record_Val += erro;
   //    用于360度的限幅（好像是超过360归零的，但是我不需要）
   //    Encoder->Encode_Actual_Val = check_codevalue(Encoder->Encode_Actual_Val , radio, Encoder->lap_encoder);             //过临界值复位码盘值

   Encoder->Encode_Actual_Val += erro;

   Encoder->last_position = Encoder->position;

   Encoder->Radian=(float)((Encoder->Encode_Actual_Val * Encoder->gear_Ratio) / Encoder->lap_encoder);
   Encoder->Total_Radian = (float)((Encoder->Encode_Record_Val * Encoder->gear_Ratio) / Encoder->lap_encoder);

   //电机堵转检测
   Encoder->State=Block_Detect(erro, Encoder);
   erro = 0;
}


/**
* 码盘值数值清零处理,注意，真实码盘会变为当前真实值，并不清0
* @param Encoder
*/
void EncoderValZero(Encoder_t *Encoder) {
   Encoder->Encode_Record_Val = 0;
   Encoder->Encode_Actual_Val = 0;
   Encoder->Radian = 0;
   Encoder->Total_Radian = 0;
   Encoder->State = NORM;
   Encoder->Init_Flag = 0;
}

/**
* 结构体清零
* @param Encoder
*/
void Encoder_Struct_Zero(Encoder_t *Encoder){
   memset((void *)Encoder, 0x0, sizeof(Encoder_t));
}

/**
* 堵转检测 TODO：还没测试，还没有考虑解堵转后
* @param error 上一次和这次位置的差值
* @param Encoder
* @return
*/
STATE_e Block_Detect(int16_t error, Encoder_t *Encoder ) {
   //其他类型的编码器比如拉线，就不用考虑堵转
   if(Encoder->Encoder_Type>=Type_End||Encoder->Block_Detect_Enable==0)return NORM;

   //绝对值小于Max_Block_Angle_Num堵转最大角度容忍值时计数器狂加
   if (error > Encoder->Max_Block_Angle_Num) {
       Encoder->Block_Count++;
   } else if (error < -Encoder->Max_Block_Angle_Num) {
       Encoder->Block_Count++;
   } else {
       Encoder->Block_Count = 0;
   }

   //最后判断是不是堵转咯
   if (Encoder->Block_Count > Encoder->Max_Block_Num)
       return BLOCK;
   else
       return NORM;

}

//临角处理16位（对应角度正值）
int16_t angle_limiting_int16(int16_t Angl_Err, int16_t lap_encoder) {
   //|当前值 - 上一次值| > 编码器最大值/2 时说明向上溢出
   if (Angl_Err < -(lap_encoder / 2))
   {
       Angl_Err += (lap_encoder - 1);
   }
   if (Angl_Err > (lap_encoder / 2)) {
       Angl_Err -= (lap_encoder - 1);
   }
   return Angl_Err;
}

//过临界值复位码盘值 （限制于360度的码盘值循环 DJI电机）
int32_t check_codevalue(int32_t value, int16_t gear_Ratio, int16_t lap_encoder) {
   if (value > (gear_Ratio * lap_encoder) / 2) {
       value = value - (gear_Ratio * lap_encoder);
   }
   if (value < (-(gear_Ratio * lap_encoder) / 2)) {
       value = (gear_Ratio * lap_encoder) - value;
   }
   return value;
}


/********************布瑞特电子编码器部分********************/
extern CAN_HandleTypeDef hcan1;
void Briter_Encoder_Code_Set(uint8_t CAN_STD_ID, Briter_Encoder_Code_e code,uint32_t data);

uint8_t temp[8] = {0x04, 0x01, 0x01, 0x00};
/**
* 布瑞特电子编码器CAN协议设置（看手册吧）
* @param CAN_STD_ID 编码器的CAN ID ,初始化为1
* @param code 命令码
* @param data 设置的数据
*/
void Briter_Encoder_Code_Set(uint8_t CAN_STD_ID, Briter_Encoder_Code_e code,uint32_t data){
   switch (code) {
       case Read_Value:{
           temp[0] = 0x04;
           temp[1] = 0x01;
           temp[2] = 0x01;
           temp[3] = 0x00;
       } break;
       case Set_Encoder_ID:{
           if (data > 255 ) {
               return ;
           }
           temp[0] = 0x04;
           temp[1] = 0x01;
           temp[2] = Set_Encoder_ID;
           temp[3] = data;
       } break;
       case Set_Baud_Rate:{
           if (data > 0x04 ) {
               return ;
           }
           temp[0] = 0x04;
           temp[1] = 0x01;
           temp[2] = Set_Baud_Rate;
           temp[3] = data;
       } break;
       case Set_Encoder_Mode:{
           if (!(data==0xAA||data==0)) {
               return ;
           }
           temp[0] = 0x04;
           temp[1] = 0x01;
           temp[2] = Set_Encoder_Mode;
           temp[3] = data;
       } break;
       case Set_Auto_Return_Time:{
           //TODO:也没测试,特别是这个高低位
           temp[0] = 0x05;
           temp[1] = 0x01;
           temp[2] = Set_Auto_Return_Time;
           temp[3] = (uint8_t)data;//低位
           temp[4] = (uint8_t)data>>8;//高位
       } break;
       case Set_ZeroPoint:{
           temp[0] = 0x04;
           temp[1] = 0x01;
           temp[2] = Set_ZeroPoint;
           temp[3] = 0x00;
       } break;
       case Set_Direction:{
           if (!(data == 0x01 || data == 0)) {
               return;
           }
           temp[0] = 0x04;
           temp[1] = 0x01;
           temp[2] = Set_Direction;
           temp[3] = data;
       } break;
       case Set_MidPoint:{
           temp[0] = 0x04;
           temp[1] = 0x01;
           temp[2] = Set_MidPoint;
           temp[3] = 0x00;
       } break;
       case Set_Current_Value:{
           //TODO:还没测试
           temp[0] = 0x07;
           temp[1] = 0x01;
           temp[2] = Set_Current_Value;
           temp[3] = (uint8_t)data>>24;
           temp[4] = (uint8_t)data>>16;
           temp[5] = (uint8_t)data>>8;
           temp[6] = (uint8_t)data;
       } break;
       case Set_Five_Loops:{
           temp[0] = 0x04;
           temp[1] = 0x01;
           temp[2] = Set_Five_Loops;
           temp[3] = 0x01;
       } break;
   }
   uint32_t Tx_MailBox;//发送邮箱
   CAN_TxHeaderTypeDef Txmessage;
   Txmessage.StdId = CAN_STD_ID;
   Txmessage.IDE = CAN_ID_STD;
   Txmessage.RTR = CAN_RTR_DATA;
   Txmessage.DLC = 8;
   while (( HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)) == 0);
   HAL_CAN_AddTxMessage(&hcan1, &Txmessage, temp, &Tx_MailBox);
}
// unused
//float Motor_Torque_Deal(Motor_t *motor)
//{
//   float torque = (9550 * (20 * (motor->anper / 20))) / motor->Encoder->Speed[1];
//   motor->torque = torque;
//   return torque;
//}

