#ifndef __ENCODER_H_
#define __ENCODER_H_

#include "struct_typedef.h"

#define CAN_ENCODER_NUM 10

/*数据状态枚举*/
typedef enum {
    NORM,  //正常
    BLOCK, //堵转
    WRONG  //异常
} STATE_e;

typedef enum {
    M3508,//如果有改减速比的，在初始化之后自己硬改
    GM6020,
    M2006,
    M3508_EngineeringPitch,
    Type_End
} Encoder_Type_e;


typedef enum {
    Read_Value = 0x01,
    Set_Encoder_ID = 0x02,
    Set_Baud_Rate = 0x03,
    Set_Encoder_Mode = 0x04,
    Set_Auto_Return_Time = 0x05,
    Set_ZeroPoint = 0x06,
    Set_Direction = 0x07,
    Set_MidPoint = 0x0C,
    Set_Current_Value = 0x0D,
    Set_Five_Loops = 0x0F,
} Briter_Encoder_Code_e;

/*CAN数据处理-码盘处理*/
typedef __packed struct {
    uint8_t ch; //目前encoder的通道
    int32_t Encode_Record_Val;//累积码盘值(从0开始记)
    int32_t Encode_Actual_Val;//真实码盘值(从当前电机编码器值开始记）

    float Radian;       //真实弧度
    float Lock_Radian;
    float User_Radian;
    float Total_Radian; //记录的弧度

    uint8_t Init_Flag;
    int16_t Speed[2];    //0为旧速度，1为新速度
    int16_t AccSpeed;    //加速度
    int16_t position;    //未处理的Can原始码盘
    int16_t last_position;    //未处理的上次的Can原始码盘
    int16_t lap_encoder;      //编码器单圈码盘值（8192=12bit）
    Encoder_Type_e Encoder_Type;//编码器种类
    bool_t Block_Detect_Enable; //堵转检测开启否
    STATE_e State;              //电机状态
    int16_t gear_Ratio;         //减速比
    int16_t Max_Block_Angle_Num;      //堵转最大角度容忍值，可以调整这个来控制堵转灵敏度
    int16_t Max_Block_Num;      //堵转计数器最大值
    int32_t Block_Count;
    void (*User_Fun)(void);     //用户自定义函数
} Encoder_t;




extern STATE_e Block_Detect(int16_t error, Encoder_t *Encoder );

extern Encoder_t *Encoder_Init(Encoder_Type_e Encoder_Type,uint8_t ch);


/*CAN返回码盘值处理*/
extern void CAN_DATA_Encoder_Deal( int16_t position, int16_t speed,  uint8_t Encoder_Num);

/*码盘值数值清零处理*/
extern void EncoderValZero(Encoder_t *Encoder);

extern void Briter_Encoder_Code_Set(uint8_t CAN_STD_ID, Briter_Encoder_Code_e code, uint32_t data);

#endif
