/*----- 电磁继电器 -----*/

#ifndef __MAGNET_H
#define __MAGNET_H

#include "delay.h"
#include "sys.h"
/* 外部变量声明 */
extern GPIO_TypeDef *DIANCI_GPIO[6];
extern u16           DIANCI_Pin[6];
/* 函数声明 */
void               Switch_Init(void);

static inline void MagnetON(u8 i) { GPIO_SetBits(DIANCI_GPIO[i], DIANCI_Pin[i]); }        // 开启电磁铁i
static inline void MagnetOFF(u8 i) {                                                      // 关闭电磁铁i
    GPIO_ResetBits(DIANCI_GPIO[i], DIANCI_Pin[i]);
    GPIO_SetBits(DIANCI_GPIO[i + 3], DIANCI_Pin[i + 3]);
    delay_ms(10);
    GPIO_ResetBits(DIANCI_GPIO[i + 3], DIANCI_Pin[i + 3]);
}

#endif /*----- __MAGNET_H -----*/
