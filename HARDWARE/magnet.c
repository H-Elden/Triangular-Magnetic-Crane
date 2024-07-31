/*----- 电磁继电器 -----*/
#include "magnet.h"

/* ----- 引脚定义 ----- */
GPIO_TypeDef * DIANCI_GPIO[3] = {GPIOD, GPIOD, GPIOD};
u16 DIANCI_Pin[3] = {GPIO_Pin_3, GPIO_Pin_1, GPIO_Pin_2};

/**
  * @brief  初始化3个电磁继电器
  * @prarm  无
  * @retval 无
  */
void Switch_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;        					//设置输出速率50MHz
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 									//设置推挽输出
	for(u8 i = 0;i < 3;i++)
	{
		GPIO_InitStructure.GPIO_Pin = DIANCI_Pin[i];
		GPIO_Init(DIANCI_GPIO[i], &GPIO_InitStructure);
		DIANCI_GPIO[i] -> BRR = DIANCI_Pin[i];													//设置初始为低电平
	}
}
