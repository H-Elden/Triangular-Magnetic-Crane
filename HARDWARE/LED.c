#include "led.h"
#include "stm32f10x_gpio.h"

// LED硬件初始化函数定义
void LED_Init(void) {

    GPIO_InitTypeDef GPIO_InitStructure;                         // 定义一个引脚初始化的结构体
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);        // 使能 GPIO 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);        // 使能 GPIO 时钟

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;              // 引脚5
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;        // 引脚输入输出模式为推挽输出模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;        // 引脚输出速度为50MHZ
    GPIO_Init(GPIOB, &GPIO_InitStructure);                   // 初始化引脚 PB5(红色LED)
    GPIO_Init(GPIOE, &GPIO_InitStructure);                   // 初始化引脚 PE5(绿色LED)

    GPIO_SetBits(GPIOB, GPIO_Pin_5);        // 初始化设置引脚为高电平(熄灭状态)
    GPIO_SetBits(GPIOE, GPIO_Pin_5);        // 初始化设置引脚为高电平(熄灭状态)
}
