#ifndef __GYROSCOPE_H_
#define __GYROSCOPE_H_


#include <string.h>
#include <stdio.h>
#include "sys.h"
#include "wit_c_sdk.h"
#include "delay.h"
#include "usart.h"
#include "uart2.h"
#include "motorencoder.h"
#include "Stepper.h"
#include "magnet.h"

#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80

extern float fAcc[3], fGyro[3], fAngle[3]; //定义float数组为下面计算输出做准备

void CmdProcess(void);
void CopeCmdData(unsigned char ucData);//UART1(串口1)接收数据并代入该函数进行处理
void AutoScanSensor(void);
void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
void Delayms(uint16_t ucMs);
void Gyro_read(void);

#endif
