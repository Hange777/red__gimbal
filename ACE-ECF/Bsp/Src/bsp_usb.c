/************************** Dongguan-University of Technology -ACE**************************
 * @file bsp_usb.c
 * @brief 
 * @author pansyhou侯文辉 (1677195845lyb@gmail.com)
 * @version 1.0
 * @date 2022-10-11
 * 
 * 
 * @history
 * <table>
 * Date       Version Author Description
 * 2022-10-11   1.0   侯文辉     
 * @verbatim 
 * ==============================================================================
 * ==============================================================================
 * @endverbatim
************************** Dongguan-University of Technology -ACE***************************/
#include "bsp_usb.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "stdarg.h"

/**
 * 重写printf,通过usbcdc虚拟串口
 * 注意：buff可能要足够大，不然发送乱码或者不全
 * @param format
 * @param ...
 */
void printf_usb(const char *format, ...){
    va_list  args;
    uint32_t length;
    uint8_t buff[APP_TX_DATA_SIZE];

    va_start(args, format);
    length = vsnprintf((char *)buff, APP_TX_DATA_SIZE, (char *)format, args);
    va_end(args);
    CDC_Transmit_FS(buff, length);
}

uint8_t Rx_Data[64];
/**
 *  其实usb可能要考虑双缓冲
 * @param buf
 */
void Usb_RX(uint8_t *buf,uint8_t *len)
{
    for (int i = 0; i < *len; i++) {
      Rx_Data[i]=*buf;
      buf++;
    }
}

