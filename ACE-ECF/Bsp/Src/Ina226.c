//
// Created by 16771 on 2023/1/8.
//
#include "Ina226.h"
#include "i2c.h"
#include "stm32f4xx_hal_i2c.h"
#include "freertos.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"
#include "string.h"
//uint32_t ina219_currentDivider_mA;
//uint32_t ina219_powerMultiplier_mW;
uint32_t ina226_calValue;
#define BUFFERLEN 4096
int16_t contBuffer[BUFFERLEN];
unsigned int bufferPos;
/* Holds currently type of continuous measurement (voltage, power, current).
 * Used to avoid floating point numbers prolification */
int measureType;

void Ina226_Init();
void Ina226_Init(){
    //set calibration

    // VBus_Max = 36V
    // VShunt_Max = 0.08v = 81.92mv (����
    // RShunt=0.005

    // 1.ȷ��������
    // MaxPossible_I = VShunt_Max/RShunt = 16A

    // 2.ȷ�������������
    // MaxExpected_I = 5A

    // 3.����LSBs�ķ�Χ(Min = 15-bit, Max = 12-bit)
    // MinimumLSB = MaxExpected_I/ 2^15-1
    // MinimumLSB = 0.00015259A     (152uA per bit)
    // MaximumLSB = MaxExpected_I/ 2^12-1
    // MaximumLSB = 0.0012207A      (1220uA per bit)
    // CurrentLSB = 0.0002  (200uA per bit)

    // 4.����У׼�Ĵ�����ֵ
    // Cal = trunc(0.00512 / (CurrentLSB * RShunt))
    // cal = 5120
    ina226_calValue = 5120;

}