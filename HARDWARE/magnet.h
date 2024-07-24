/*----- 电磁继电器 -----*/

#ifndef __MAGNET_H
#define __MAGNET_H

#include "sys.h"

extern GPIO_TypeDef * DIANCI_GPIO[3];
extern u16 DIANCI_Pin[3];

void Switch_Init(void);

static inline void MagnetON(u8 i)	{GPIO_SetBits(DIANCI_GPIO[i], DIANCI_Pin[i]);}				//开启电磁铁i
static inline void MagnetOFF(u8 i)	{GPIO_ResetBits(DIANCI_GPIO[i], DIANCI_Pin[i]);}		//关闭电磁铁i

#endif	/*----- __MAGNET_H -----*/
