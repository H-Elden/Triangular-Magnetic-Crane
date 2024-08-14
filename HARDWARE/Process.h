/*----- 整机运行过程 -----*/

#ifndef __PROCESS_H_
#define __PROCESS_H_

#include "delay.h"
#include "led.h"
#include "key.h"
#include "motorencoder.h"
#include "HC-SR04.h"
#include "Stepper.h"
#include "magnet.h"
#include "uart2.h"
#include "usart3.h"
#include "wit_c_sdk.h"
#include "gyroscope.h"

extern int way;
extern float PointDis[5][3];

//初始化函数
void Init(void);
void PointDis_Init(void);

//操作函数
void Catch(char Line, u8 odd, u8 mid, u8 even);
void Place_Side(void);

//过程函数
void BLine(void);
void CLine(void);
void DPoint(void);
void E1_Check(void);
void ELine0(void);
void HLine(void);
void Back(void);
void ILine(void);

#endif
