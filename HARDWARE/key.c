
/*-----KEY的板级支持包-----*/

#include "key.h"

void KEY_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct;

	// 使能GPIOE和GPIOA和GPIOE时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	// 配置引脚为输入模式
	GPIO_InitStruct.GPIO_Pin = KEY0_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;						//上拉输入
	GPIO_Init(KEY0_GPIO_PROT, &GPIO_InitStruct);
}

uint8_t KEY_Scan() {
	//判断按键是否被按下
	if (GPIO_ReadInputDataBit(KEY0_GPIO_PROT, KEY0_PIN) == Bit_RESET) {	//低电平说明按键按下
		//按键被按下
		while (GPIO_ReadInputDataBit(KEY0_GPIO_PROT, KEY0_PIN) == Bit_RESET);
		return KEY_ON;
	} else {
		//按键未按下
		return KEY_OFF;
	}
}
