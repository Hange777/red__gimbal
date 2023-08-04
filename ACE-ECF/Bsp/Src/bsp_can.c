/************************** Dongguan-University of Technology -ACE**************************
 * @file bsp_can.c
 * @brief
 * @author pansyhou侯文辉 (1677195845lyb@gmail.com)
 * @version 1.0
 * @date 2022-07-24

 * @history
 * Date       Version Author Description
 *                     dji
 * 2022-07-24   1.0   侯文辉
 * 2022-12-06   2.0   侯文辉
 * @verbatim
 * ==============================================================================
 *  前言：
 *      这份文件是由官方开源代码修改来的，为解决工程需要收发大量电机数据、CAN发送邮箱较少只能强迫丢包等问题。
 *      2.0之后适配ECF，合并原有CAN
 *  介绍：
 *      FIFO:
 *      利用环形队列FIFO生成缓冲区，在使用发送函数的时候先将数据存入缓冲区，待有空闲邮箱时将环形队列数据提出并发送
 *
 *      Block:
 *      CAN自带的mailboxes只有三个，如果出现三个邮箱都会满的情况，Hal库的HAL_CAN_AddTxMessage就会自动丢包
 *      堵塞的解决方法就是while到有空闲mailbox为止，再发送
 *      No_Block自己承担丢包风险。
 *
 *      电机封装函数:
 *      在config里记得选择发送方式。
 *
 *      CAN初始化:
 *      这个初始化会开启很多的中断，同时开启回调函数、CANStart、CAN过滤器配置，如果要绑定回调函数，记得用register函数注册。
 *
 *
 *
 *  流程：
 *  文件内容：
 *      1.CAN管理结构体与CAN控制器初始化
 *      2.设置对应的CAN接收回调函数函数
 *      3.CAN发送函数（三种。1.堵塞式 2.不堵塞丢包式 3.环形队列）
 *      4.空闲中断处理函数
 *      5.更多杂七杂八的回调函数
 *      6.电机数值发送函数
 *      7.CANFIFO压力输出
 *  如何使用?建议?
 *      1.将ECF_CAN_Init()初始化函数丢进硬件初始化区，自带启用CAN1、2（HAL_CAN_Start）
 *          PS：需要接收的就在初始化里用ECF_CAN_Rx_Callback_Register //TODO:记得自己加
 *      2.发送：将处理好的数据放入一个8个u8的数组里，处理好后用ECF_CAN_Send_Msg_FIFO函数即可（）
 *      3.接收：在初始化里设置好数据处理函数后就可以了，不设置默认跳过，可以根据需求将初始化的中断关闭
 *      4.建议四个电机打包成一个包后再发，不然带宽肯定还是很难达到高利用率
 *      5.应该除了工程之外很少有需要CAN+环形队列的，需要高时效性的就不要使用这个（比如DR16的串口
 *      接收，基本上用的都是DMA双缓或者纯DMA
 *  可能留下的坑，还没进行足够的测试：
 *      1.这个FIFO文件和之前串口的环形队列文件不同，这个需要将CAN信息打包，需要用到void来收打包
 *        后的指针（可以尝试去看看这个FIFO文件）
 *        这样可以收各种类型的东西（比如说各种结构体，以一个个结构体为单位打包进队列，达到类似于java的泛型的效果）
 *      2.这个FIFO用到一些奇奇怪怪的中断临界区进出和互斥锁的宏定义，个人不是想知道怎么实现，可能有问题也查不出来
 *      3.缓冲区压力还没经过测试，可以到时候自行查看或者调整缓冲区大小
 *      4.还没试过发送数组小于8的时候，我个人试的时候好像嘎了//TODO:
 * ==============================================================================
 * @endverbatim
 ************************** Dongguan-University of Technology -ACE***************************/
#include "bsp_can.h"
#include "bsp_usb.h"
#include "main.h"
#include "ECF_BspConfig.h"

#define CAN1_TX_FIFO_UNIT_NUM (256) //发送FIFO的一个单元大小，要2的幂次方
#define CAN1_TX_FIFO_SIZE (CAN1_TX_FIFO_UNIT_NUM * sizeof(struct can_std_msg))

#define CAN2_TX_FIFO_UNIT_NUM (256)
#define CAN2_TX_FIFO_SIZE (CAN2_TX_FIFO_UNIT_NUM * sizeof(struct can_std_msg))

// CAN1/2发送结构体定义
can_manage_obj can1_manage;
can_manage_obj can2_manage;

static uint8_t can1_tx_fifo_buff[CAN1_TX_FIFO_SIZE];
static uint8_t can2_tx_fifo_buff[CAN2_TX_FIFO_SIZE];

void CAN_FIFO_Stress_Watch(void);



/**************堵塞式发送函数**************/
void ECF_CAN1_Send_Msg_Block(int16_t stdid, uint8_t *data, uint16_t len);
void ECF_CAN2_Send_Msg_Block(int16_t stdid, uint8_t *data, uint16_t len);
/**************普通可能会丢包的发送函数**************/
void ECF_CAN1_Send_Msg_NO_Block(int16_t stdid, uint8_t *data, uint16_t len);
void ECF_CAN2_Send_Msg_NO_Block(int16_t stdid, uint8_t *data, uint16_t len);

uint16_t TxmailboxesNum;
void CAN_FIFO_Stress_Watch(void) {
    //    printf_usb("CAN1:  free_num=%d    used_num=%d", can1_manage.tx_fifo.free_num, can1_manage.tx_fifo.used_num);
    //    printf_usb("CAN2:  free_num=%d    used_num=%d", can2_manage.tx_fifo.free_num, can2_manage.tx_fifo.used_num);
    //    printf_usb("    Txmailboxes=  %d        ", HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));
    TxmailboxesNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
}

void ECF_CAN1_Send_Msg_Block(int16_t stdid, uint8_t *data, uint16_t len)
{
    if (len > 8) {
        return;
    }
    uint32_t Tx_MailBox;
    CAN_TxHeaderTypeDef Txmessage;
    Txmessage.StdId = stdid;
    Txmessage.IDE = CAN_ID_STD;
    Txmessage.RTR = CAN_RTR_DATA;
    Txmessage.DLC = len;
    while ((HAL_CAN_GetTxMailboxesFreeLevel(&hcan1))==0);
    HAL_CAN_AddTxMessage(&hcan1, &Txmessage, data, &Tx_MailBox);
}

void ECF_CAN2_Send_Msg_Block(int16_t stdid, uint8_t *data, uint16_t len)
{
    if (len > 8) {
        return;
    }
    uint32_t Tx_MailBox;
    CAN_TxHeaderTypeDef Txmessage;
    Txmessage.StdId = stdid;
    Txmessage.IDE = CAN_ID_STD;
    Txmessage.RTR = CAN_RTR_DATA;
    Txmessage.DLC = len;
    while ((HAL_CAN_GetTxMailboxesFreeLevel(&hcan2))==0);
    HAL_CAN_AddTxMessage(&hcan2, &Txmessage, data, &Tx_MailBox);
}


void ECF_CAN1_Send_Msg_NO_Block(int16_t stdid, uint8_t *data, uint16_t len)
{
    if (len > 8) {
        return;
    }
    uint32_t Tx_MailBox;
    CAN_TxHeaderTypeDef Txmessage;
    Txmessage.StdId = stdid;
    Txmessage.IDE = CAN_ID_STD;
    Txmessage.RTR = CAN_RTR_DATA;
    Txmessage.DLC = len;
    HAL_CAN_AddTxMessage(&hcan1, &Txmessage, data, &Tx_MailBox);
}

void ECF_CAN2_Send_Msg_NO_Block(int16_t stdid, uint8_t *data, uint16_t len)
{
    if (len > 8) {
        return;
    }
    uint32_t Tx_MailBox;
    CAN_TxHeaderTypeDef Txmessage;
    Txmessage.StdId = stdid;
    Txmessage.IDE = CAN_ID_STD;
    Txmessage.RTR = CAN_RTR_DATA;
    Txmessage.DLC = len;
    HAL_CAN_AddTxMessage(&hcan2, &Txmessage, data, &Tx_MailBox);
}


// CAN管理结构体与CAN控制器初始化
void ECF_CAN_Init(void) {
    // CAN管理结构体初始化
    can1_manage.is_sending = 0;
    can2_manage.is_sending = 0;

    can1_manage.hcan = &hcan1;
    can2_manage.hcan = &hcan2;

    can1_manage.can_rec_callback = NULL; //接收中断的数据处理函数，我先置NULL，后期自己注册
    can2_manage.can_rec_callback = NULL;

    //发送环形队列初始化
    fifo_init(&(can1_manage.tx_fifo), can1_tx_fifo_buff, sizeof(can_std_msg), CAN1_TX_FIFO_UNIT_NUM);
    fifo_init(&(can2_manage.tx_fifo), can2_tx_fifo_buff, sizeof(can_std_msg), CAN2_TX_FIFO_UNIT_NUM);

    // CAN过滤器初始化
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    //开启CAN1传输
    HAL_CAN_Start(&hcan1);
    //开启对应的CAN中断
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);

    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_ERROR);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_ERROR_WARNING);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_BUSOFF);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_ERROR_PASSIVE);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_LAST_ERROR_CODE);

    //CAN2部分
    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_TX_MAILBOX_EMPTY);

    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_ERROR);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_ERROR_WARNING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_BUSOFF);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_ERROR_PASSIVE);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_LAST_ERROR_CODE);

    return;
}


/************************** Dongguan-University of Technology -ACE**************************
* @brief 设置对应的CAN接收回调函数函数
*
* @param m_obj 对应的CAN管理器结构体
* @param fun 你自己的数据处理函数
* @return int32_t
************************** Dongguan-University of Technology -ACE***************************/
int32_t ECF_CAN_Rx_Callback_Register(can_manage_obj *m_obj, can_stdmsg_rx_callback_t fun) {
    m_obj->can_rec_callback = fun;
    return 1;
}


/************************** Dongguan-University of Technology -ACE**************************
* @brief   CAN发送函数
*
* @param hcan  can句柄
* @param std_id  id
* @param data 缓存区首地址（存放发送的数据的首地址
* @param len  数据长度
* @return uint32_t
************************** Dongguan-University of Technology -ACE***************************/
uint32_t ECF_CAN_Send_Msg_FIFO(CAN_HandleTypeDef *hcan, uint16_t std_id, uint8_t *data, uint16_t len) {
    if (len > 8) {
        return 0;
    }
    uint8_t *send_ptr;    //缓存区发送数据的指针（ptr：pointer)
    uint16_t send_num;    //
    can_manage_obj m_obj; // CAN管理器结构体
    can_std_msg msg;      // CAN发送数据包

    send_ptr = data;
    msg.std_id = std_id;
    send_num = 0;

    //判断是CAN1还是CAN2发送
    if (hcan == &hcan1) {
        m_obj = can1_manage;
    } else if (hcan == &hcan2) {
        m_obj = can2_manage;
    } else {
        return 0;
    }

    //
    while (send_num < len) {
        //判断队列是否满
        if (fifo_is_full(&(m_obj.tx_fifo))) {
            // can is error
            m_obj.is_sending = 0;
            break;
        }

        if (len - send_num >= 8) //一般发送电机数据len都为8，8-0=0后，数据长度=8
        {
            msg.dlc = 8;
        } else // dlc严格小于8
        {
            msg.dlc = len - send_num;
        }

        // memcpy(msg.data, data, msg.dlc);
        *((uint32_t *) (msg.data)) = *((uint32_t *) (send_ptr)); //内容传递
        *((uint32_t *) (msg.data + 4)) = *((uint32_t *) (send_ptr + 4));

        send_ptr += msg.dlc; //跳过已经传完的数据
        send_num += msg.dlc;

        fifo_put(&(m_obj.tx_fifo), &msg); //将搞好的数据包存入fifo
    }
    //
    if ((m_obj.is_sending) == 0 && (!(fifo_is_empty(&(m_obj.tx_fifo))))) {
        CAN_TxHeaderTypeDef header;
        uint32_t send_mail_box;

        header.StdId = std_id;
        header.IDE = CAN_ID_STD;
        header.RTR = CAN_RTR_DATA;

        //当有空闲邮箱+队列不为空时发送
        while (HAL_CAN_GetTxMailboxesFreeLevel(m_obj.hcan) && (!(fifo_is_empty(&(m_obj.tx_fifo))))) {
            fifo_get(&(m_obj.tx_fifo), &msg);
            header.DLC = msg.dlc;
            HAL_CAN_AddTxMessage(m_obj.hcan, &header, msg.data, &send_mail_box);

            m_obj.is_sending = 1;
        }
    }

    return send_num;
}


/************************** Dongguan-University of Technology -ACE**************************
* @brief  空闲中断处理函数
* 已经开启了CAN邮箱空闲中断（CAN_IT_TX_MAILBOX_EMPTY)，空闲的时候发送FIFO里的存货
*        没货就把对应的管理器is_sending置0
* @param m_obj 对应的CAN管理结构体
************************** Dongguan-University of Technology -ACE***************************/
static void can_tx_mailbox_complete_hanle(can_manage_obj m_obj) {
    can_std_msg msg;    //发送信息数据包
    CAN_TxHeaderTypeDef header;
    uint32_t send_mail_box;
    //进入临界区
    CRITICAL_SETCION_ENTER();

        if (!fifo_is_empty(&(m_obj.tx_fifo))) {
            while (!fifo_is_empty(&(m_obj.tx_fifo))) {
                //当FIFO里面有存货时，赶紧发送清库存
                if (HAL_CAN_GetTxMailboxesFreeLevel(m_obj.hcan)) {

                    fifo_get_noprotect(&(m_obj.tx_fifo), &msg);

                    header.StdId = msg.std_id;
                    header.DLC = msg.dlc;
                    header.IDE = CAN_ID_STD;
                    header.RTR = CAN_RTR_DATA;

                    HAL_CAN_AddTxMessage(m_obj.hcan, &header, msg.data, &send_mail_box);
                } else {
                    m_obj.is_sending = 0;
                }
            }
        } else {
            m_obj.is_sending = 0;
        }
        //退出临界区
    CRITICAL_SETCION_EXIT();

    return;
}


//注册CAN邮箱0发送完成回调函数
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
    if (hcan == &hcan1) {
        can_tx_mailbox_complete_hanle(can1_manage);
    } else if (hcan == &hcan2) {
        can_tx_mailbox_complete_hanle(can2_manage);
    }
}

//注册CAN邮箱1发送完成回调函数
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {
    if (hcan == &hcan1) {
        can_tx_mailbox_complete_hanle(can1_manage);
    } else if (hcan == &hcan2) {
        can_tx_mailbox_complete_hanle(can2_manage);
    }
}

//注册CAN邮箱2发送完成回调函数
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) {
    if (hcan == &hcan1) {
        can_tx_mailbox_complete_hanle(can1_manage);
    } else if (hcan == &hcan2) {
        can_tx_mailbox_complete_hanle(can2_manage);
    }
}

//注册CAN错误回调函数
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
    if (hcan == &hcan1) {
        can_tx_mailbox_complete_hanle(can1_manage);
    } else if (hcan == &hcan2) {
        can_tx_mailbox_complete_hanle(can2_manage);
    }
    HAL_CAN_ResetError(hcan);
}

//接收到数据就进入中断的回调函数,如果只是发送就不用管
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if (hcan == &hcan1) {
        if (can1_manage.can_rec_callback != NULL) {
            (*(can1_manage.can_rec_callback))(&rx_header, rx_data);
        }
    } else if (hcan == &hcan2) {
        if (can2_manage.can_rec_callback != NULL) {
            (*(can2_manage.can_rec_callback))(&rx_header, rx_data);
        }
    }
}



/************************** Dongguan-University of Technology -ACE**************************
 * @brief 通过CAN1实现C6020电调下的四个电机控制,1号电机到4号
 *
 * @param ESC_201
 * @param ESC_202
 * @param ESC_203
 * @param ESC_204
************************** Dongguan-University of Technology -ACE***************************/
void CAN1_C620_OR_C610_201_TO_204_SendMsg(int16_t ESC_201, int16_t ESC_202, int16_t ESC_203, int16_t ESC_204) {
    uint8_t Can_Send_Data1[8];
    Can_Send_Data1[0] = ESC_201 >> 8;
    Can_Send_Data1[1] = ESC_201;
    Can_Send_Data1[2] = ESC_202 >> 8;
    Can_Send_Data1[3] = ESC_202;
    Can_Send_Data1[4] = ESC_203 >> 8;
    Can_Send_Data1[5] = ESC_203;

    Can_Send_Data1[6] = ESC_204 >> 8;
    Can_Send_Data1[7] = ESC_204;
#if  CONFIG_CAN_SEND_MOTOR_USE_FIFO == 1
    ECF_CAN_Send_Msg_FIFO(&hcan1, C620_OR_C610_1_TO_4_ID, Can_Send_Data1, 8);
#elif CONFIG_CAN_SEND_MOTOR_USE_FIFO == 0
    ECF_CAN1_Send_Msg_Block(C620_OR_C610_1_TO_4_ID,Can_Send_Data1, 8);
#endif
}

//5号到8号电机
void CAN1_C620_OR_C610_205_TO_208_SendMsg(int16_t ESC_201, int16_t ESC_202, int16_t ESC_203, int16_t ESC_204) {
    uint8_t Can_Send_Data2[8];
    Can_Send_Data2[0] = ESC_201 >> 8;
    Can_Send_Data2[1] = ESC_201;
    Can_Send_Data2[2] = ESC_202 >> 8;
    Can_Send_Data2[3] = ESC_202;
    Can_Send_Data2[4] = ESC_203 >> 8;
    Can_Send_Data2[5] = ESC_203;
    Can_Send_Data2[6] = ESC_204 >> 8;
    Can_Send_Data2[7] = ESC_204;
#if  CONFIG_CAN_SEND_MOTOR_USE_FIFO == 1
    ECF_CAN_Send_Msg_FIFO(&hcan1, C620_OR_C610_5_TO_8_ID, Can_Send_Data2, 8);
#elif CONFIG_CAN_SEND_MOTOR_USE_FIFO == 0
    ECF_CAN1_Send_Msg_Block(C620_OR_C610_5_TO_8_ID,Can_Send_Data2, 8);
#endif
}


/************************** Dongguan-University of Technology -ACE**************************
 * @brief 通过CAN1实现GM6020的四个电机控制,1号电机到4号,注意！，1到四号的电机和C6020电调里的4-8中的ID冲突
 * 所以一般将电机调到4-8号里发
 *
 * @param ESC_201
 * @param ESC_202
 * @param ESC_203
 * @param ESC_204
************************** Dongguan-University of Technology -ACE***************************/
void CAN1_GM6020_1_TO_4_SendMsg(int16_t ESC_201, int16_t ESC_202, int16_t ESC_203, int16_t ESC_204) {
    static uint8_t Can_Send_Data3[8];
    Can_Send_Data3[0] = ESC_201 >> 8;
    Can_Send_Data3[1] = ESC_201;
    Can_Send_Data3[2] = ESC_202 >> 8;
    Can_Send_Data3[3] = ESC_202;
    Can_Send_Data3[4] = ESC_203 >> 8;
    Can_Send_Data3[5] = ESC_203;
    Can_Send_Data3[6] = ESC_204 >> 8;
    Can_Send_Data3[7] = ESC_204;
#if  CONFIG_CAN_SEND_MOTOR_USE_FIFO == 1
    ECF_CAN_Send_Msg_FIFO(&hcan1, GM6020_1_TO_4_ID, Can_Send_Data3, 8);
#elif CONFIG_CAN_SEND_MOTOR_USE_FIFO == 0
    ECF_CAN1_Send_Msg_Block(GM6020_1_TO_4_ID,Can_Send_Data3, 8);
#endif
}

//5号到8号电机
void CAN1_GM6020_5_TO_8_SendMsg(int16_t ESC_201, int16_t ESC_202, int16_t ESC_203, int16_t ESC_204) {
    static uint8_t Can_Send_Data4[8];
    Can_Send_Data4[0] = ESC_201 >> 8;
    Can_Send_Data4[1] = ESC_201;
    Can_Send_Data4[2] = ESC_202 >> 8;
    Can_Send_Data4[3] = ESC_202;
    Can_Send_Data4[4] = ESC_203 >> 8;
    Can_Send_Data4[5] = ESC_203;
    Can_Send_Data4[6] = ESC_204 >> 8;
    Can_Send_Data4[7] = ESC_204;

#if  CONFIG_CAN_SEND_MOTOR_USE_FIFO == 1
    ECF_CAN_Send_Msg_FIFO(&hcan1, GM6020_5_TO_7_ID, Can_Send_Data4, 8);
#elif CONFIG_CAN_SEND_MOTOR_USE_FIFO == 0
    ECF_CAN1_Send_Msg_Block(GM6020_5_TO_7_ID,Can_Send_Data4, 8);
#endif

}
