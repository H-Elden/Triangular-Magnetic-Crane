/*----- HC-SR04 超声波传感器的板级支持包-----*/

/*
		使用芯片：STM32F103ZET6
		使用定时器：TIM7：计数器；TIM6：每10ms产生一次中断，进行一次测距
		超声波传感器：使用5V电源
		引脚配置：	
				超声波0：(前轮向内测)
							PG0    ------> Echo0
							PG1    ------> Trig0
				超声波1：（左前轮）
							PE7    ------> Echo1
							PE8    ------> Trig1
				超声波2：（右前轮）
							PE9    ------> Echo2
							PE10   ------> Trig2
				超声波3：（左后轮）
							PE11   ------> Echo3
							PE12   ------> Trig3
*/

#ifndef __BSP_HCSR04_H
#define __BSP_HCSR04_H

#include "sys.h"
#include "delay.h"
#include "motorencoder.h" 
#include "led.h"
#include "Process.h"

#define CYCLES 15						//循环次数，即统计近多少次的超声波数据，范围[0,32)

extern uint8_t Sensor_open;
extern uint32_t ones[3];

//内联函数的定义直接写在头文件内
static inline void SensorON(u8 i) {ones[i] = 0; Sensor_open |= (1 << i);}				//开启第i个传感器
static inline void SensorOFF(u8 i) {Sensor_open &= ~(1 << i);}									//关闭第i个传感器

extern uint8_t obj[8];
extern char isStore;
extern float dist[3];
extern uint8_t isStop;

void TIM7_Init(void);
void HCSR04_Init(void);
void Getdis(void);
void SetGoaldis(u8 i, u8 _min, u8 _max);
uint8_t countEffect(uint8_t i,uint8_t input);
void EXTI_Sensor(uint8_t i);

#endif	/* __BSP_HCSR04_H */
