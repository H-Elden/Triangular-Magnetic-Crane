
/*-----按键KEY-----*/

#ifndef __KEY_H
#define __KEY_H

#include "sys.h"

#define KEY_ON 	1
#define KEY_OFF 0

#define KEY0_PIN 									GPIO_Pin_4
#define KEY0_GPIO_PROT 						GPIOE

//初始化按键对应的GPIO引脚
void KEY_GPIO_Init(void);

//判断按键的状态
uint8_t KEY_Scan(void);

#endif	/*  __KEY_H  */
