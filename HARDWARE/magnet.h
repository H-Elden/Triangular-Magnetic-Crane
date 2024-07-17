/*----- µç´Å¼ÌµçÆ÷ -----*/

#ifndef __MAGNET_H
#define __MAGNET_H

#include "sys.h"
#include "stm32f10x_tim.h"

extern GPIO_TypeDef * DIANCI_GPIO[3];
extern u16 DIANCI_Pin[3];

void Switch_Init(void);

extern void MagnetON(u8 i);
extern void MagnetOFF(u8 i);

#endif	/*----- __MAGNET_H -----*/
