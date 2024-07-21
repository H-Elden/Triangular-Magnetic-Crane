#include "L298N.h"

void Motor_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能GPIOB的时钟

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;         //推挽输出
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15; //PB12 PB13
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIO_ResetBits(GPIOB, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15); //初始化设置引脚低电平
}
/**************************************************************************
函数功能：L298N初始化函数
入口参数：定时器1 3计数上限 定时器1 3预分频系数
返回  值：无
**************************************************************************/
void LL298N_Init(int arr, int psc) {

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;//定时器中断结构体
	TIM_OCInitTypeDef TIM_OCInitStruct;//pwm输出结构体
	GPIO_InitTypeDef GPIO_InitStruct;//引脚初始化结构体

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能GPIOB时钟，GPIOB挂载在APB2时钟下，在STM32中使用IO口前都要使能对应时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //使能通用定时器3时钟



	//L298NPWM输出引脚
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;          //复用输出
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;                //PB1
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);


	TIM_TimeBaseInitStruct.TIM_Period = arr;              //设定计数器自动重装值
	TIM_TimeBaseInitStruct.TIM_Prescaler  = psc;          //设定预分频器
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;//TIM向上计数模式
	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;         //设置时钟分割
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);      //初始化定时器

	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;             //选择PWM1模式
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStruct.TIM_Pulse = 0;                            //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;     //设置输出极性
	TIM_OC4Init(TIM3, &TIM_OCInitStruct);                      //初始化输出比较参数


	TIM_CtrlPWMOutputs(TIM3, ENABLE);	//MOE 主输出使能

	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);  //CH4使能预装载寄存器

	TIM_ARRPreloadConfig(TIM3, ENABLE); //TIM3预装载使能

	TIM_Cmd(TIM3, ENABLE); //使能定时器TIM3

}

void RL298N_Init(int arr, int psc) {

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;//定时器中断结构体
	TIM_OCInitTypeDef TIM_OCInitStruct;//pwm输出结构体
	GPIO_InitTypeDef GPIO_InitStruct;//引脚初始化结构体


	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE); //使能通用定时器8时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //使能GPIOC时钟，GPIOA挂载在APB2时钟下，在STM32中使用IO口前都要使能对应时钟


	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;          //复用输出
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;                //PC9
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStruct);


	TIM_TimeBaseInitStruct.TIM_Period = arr;              //设定计数器自动重装值
	TIM_TimeBaseInitStruct.TIM_Prescaler  = psc;          //设定预分频器
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;//TIM向上计数模式
	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;         //设置时钟分割
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseInitStruct);      //初始化定时器

	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;             //选择PWM1模式
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStruct.TIM_Pulse = 0;                            //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;     //设置输出极性
	TIM_OC4Init(TIM8, &TIM_OCInitStruct);                      //初始化输出比较参数

	TIM_CtrlPWMOutputs(TIM8, ENABLE);	//MOE 主输出使能

	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH4使能预装载寄存器

	TIM_ARRPreloadConfig(TIM8, ENABLE); //TIM8预装载使能

	TIM_Cmd(TIM8, ENABLE); //使能定时器TIM8
}

/**************************************************************************
函数功能：设置TIM1 TIM3通道4PWM值
入口参数：PWM值
返回  值：无
**************************************************************************/
void SetPWM(int moterLeft, int moterRight) {
	if (moterLeft >= 0) { //pwm>=0 (BIN1, BIN2)=(0, 1) 正转 顺时针
		PBout(13) = 0; //BIN1=0
		PBout(12) = 1; //BIN2=1
		TIM3->CCR4 = moterLeft;
		TIM_SetCompare4(TIM3, moterLeft);
	} else if (moterLeft < 0) { //pwm<0 (BIN1, BIN2)=(1, 0) 反转 逆时针
		PBout(13) = 1; //BIN1=1
		PBout(12) = 0; //BIN2=0
		TIM3->CCR4 = -moterLeft;
		TIM_SetCompare4(TIM3, -moterLeft);
	}
	if (moterRight >= 0) { //pwm>=0 (BIN1, BIN2)=(0, 1) 正转 顺时针
		PBout(15) = 0; //BIN1=0
		PBout(14) = 1; //BIN2=1
		TIM8->CCR4 = moterRight;
		TIM_SetCompare4(TIM8, moterRight);
	} else if (moterRight < 0) { //pwm<0 (BIN1, BIN2)=(1, 0) 反转 逆时针
		PBout(15) = 1; //BIN1=1
		PBout(14) = 0; //BIN2=0
		TIM8->CCR4 = -moterRight;
		TIM_SetCompare4(TIM8, -moterRight);
	}
}

