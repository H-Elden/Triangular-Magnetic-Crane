#ifndef __STEPPER_H
#define __STEPPER_H

#include "usart3.h"

#define		ABS(x)		((x) > 0 ? (x) : -(x))

#define SVEL_C 400						//设置垂直步进的速度
#define SACC_C 240						//设置垂直步进的加速度
#define SVEL_S 400						//设置水平步进的速度
#define SACC_S 220						//设置水平步进的加速度

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
#define Z0 550

/* 基于
速度SVEL_S = 400 SVEL_C = 400
加速度SACC_S = 220 SACC_C = 240
的测量结果如下 */
#define TIME_S1		13
#define TIME_S2		15
#define TIME_S2_1	7
#define TIME_C1		5
#define TIME_C2		7
#define TIME_Z0		5

//带负载垂直上升
#define TIME_C1_W	7
#define TIME_C2_W	13
#define TIME_Z0_W	9

extern u8 weight[3];			//抓手是否带负载

typedef enum {
	S_VER   = 0,			/* 读取固件版本和对应的硬件版本 */
	S_RL    = 1,			/* 读取读取相电阻和相电感 */
	S_PID   = 2,			/* 读取PID参数 */
	S_VBUS  = 3,			/* 读取总线电压 */
	S_CPHA  = 5,			/* 读取相电流 */
	S_ENCL  = 7,			/* 读取经过线性化校准后的编码器值 */
	S_TPOS  = 8,			/* 读取电机目标位置角度 */
	S_VEL   = 9,			/* 读取电机实时转速 */
	S_CPOS  = 10,			/* 读取电机实时位置角度 */
	S_PERR  = 11,			/* 读取电机位置误差角度 */
	S_FLAG  = 13,			/* 读取使能/到位/堵转状态标志位 */
	S_Conf  = 14,			/* 读取驱动参数 */
	S_State = 15,			/* 读取系统状态参数 */
	S_ORG   = 16,     /* 读取正在回零/回零失败状态标志位 */
} SysParams_t;


void TIM5_Init(void);
void Stepper_Turn(uint8_t addr, uint8_t dir, float angle);
uint8_t Stepper_GetStatus(uint8_t addr);

#endif
