/*----- 整机运行过程 -----*/

#ifndef __PROCESS_H_
#define __PROCESS_H_

#include "delay.h"
#include "usart1.h"
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

void Init(void);
void PointDis_Init(void);

void BLine(void);
void CLine(void);
void Cyline(void);
void ELine1(void);
void ELine21(void);
void ELine22(void);
void ELine221(void);
void FLine1(void);
void FLine2(void);
void HLine1(void);
void HLine2(void);
void GLine1(void);
void DPoint1(void);
void DPoint2(void);
void CLine1(void);
void BLine1(void);
void ILine(void);

void Cyline(void);
void DPoint2(void); 

#endif
