
/*-----STEPPER 步进电机的板级支持包-----*/

#ifndef __BSP_STEPPER_H
#define __BSP_STEPPER_H

#include "sys.h"
#include "delay.h"

#define CLOCKWISE 		1				//顺时针方向转动
#define ANTICLOCKWISE 0				//逆时针方向转动
#define STEPANGLE			1.8			//步进电机的步进角，角度制
#define SUBDIV				32			//步进电机驱动器的细分数

#define WAI1 	CLOCKWISE
#define NEI1 	ANTICLOCKWISE
#define WAI2 	ANTICLOCKWISE
#define NEI2 	CLOCKWISE
#define UP3 	CLOCKWISE
#define DOWN3 ANTICLOCKWISE
#define UP4 	ANTICLOCKWISE
#define DOWN4 CLOCKWISE
#define UP0 	ANTICLOCKWISE
#define DOWN0 CLOCKWISE

#define S1 1550
#define S2 2080
#define C1 500
#define C2 1000
#define Z0 675

extern GPIO_TypeDef * PUL_GPIO[5];
extern u16 PUL_Pin[5];
extern GPIO_TypeDef * DIR_GPIO[5];
extern u16 DIR_Pin[5];

/* ---- 电机操作定义 ---- */
static inline void CW(u8 i) {GPIO_ResetBits(DIR_GPIO[i], DIR_Pin[i]);}		//第i个电机设置顺时针转动
static inline void ACW(u8 i) {GPIO_SetBits(DIR_GPIO[i], DIR_Pin[i]);}			//第i个电机设置逆时针转动
static inline void Stepper_Pul_HIGH(u8 i)	{GPIO_SetBits(PUL_GPIO[i], PUL_Pin[i]);}			//第i个电机设置脉冲为高电平
static inline void Stepper_Pul_LOW(u8 i)	{GPIO_ResetBits(PUL_GPIO[i], PUL_Pin[i]);}		//第i个电机设置脉冲为低电平
static inline void Stepper_Pul_TGL(u8 i)	{(GPIO_ReadInputDataBit(PUL_GPIO[i], PUL_Pin[i]) == Bit_SET) ? 
																						Stepper_Pul_LOW(i) : Stepper_Pul_HIGH(i);}	//翻转第i个电机的脉冲电平

extern u16 PulsePeriod[5];
extern u16 RotationRate[5];
extern u8 StartStepper[5];

void Stepper_Init_TIM5(void);
void Stepper_Turn(u8 i,u8 dir, float angle);				//转动步进电机

void Stepper_SetRPM(u8 i,u16 rpm);									//设置步进电机转速
void Stepper_SetPulsePeriod(u8 i,u16 n);						//设置脉冲周期

#endif
