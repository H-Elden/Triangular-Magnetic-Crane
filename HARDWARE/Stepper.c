
/*-----STEPPER 步进电机的板级支持包-----*/

#include "Stepper.h"

/* ----- 引脚定义 ----- */
static GPIO_TypeDef * PUL_GPIO[5] = {GPIOC, GPIOB, GPIOE, GPIOF, GPIOE};
static u16 PUL_Pin[5] = {GPIO_Pin_13, GPIO_Pin_9, GPIO_Pin_1, GPIO_Pin_1, GPIO_Pin_3};
static GPIO_TypeDef * DIR_GPIO[5] = {GPIOF, GPIOE, GPIOE, GPIOF, GPIOE};
static u16 DIR_Pin[5] = {GPIO_Pin_0, GPIO_Pin_0, GPIO_Pin_2, GPIO_Pin_2, GPIO_Pin_6};

/* ----全局变量---- */

/* 关系式：转速 = 9375 / 脉冲周期 */
u16 PulsePeriod[5] = {18, 18, 18, 18, 18};								//脉冲周期，单位	us
u16 RotationRate[5] = {312, 312, 312, 312, 312};					//转速，单位 rpm

u8 StartStepper[5];															//是否开启，默认为0，不开启
u32 usTimer[5];																	//当前脉冲半周期过去了多少微秒
u32 TargetPulse[5];															//目标脉冲数
u32 PulseNum[5];																//已经发送了多少个脉冲

/**
  * @brief  配置步进电机的GPIO 和 通用定时器TIM5
  * @param  无
  * @note   无
  * @retval 无
  */
void Stepper_Init_TIM5(void) {

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// 使能GPIO时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);

	// 配置脉冲引脚
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;									//推挽输出
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;								//高速切换
	for (u8 i = 0; i < 5; i++) {
		GPIO_InitStruct.GPIO_Pin = PUL_Pin[i];
		GPIO_Init(PUL_GPIO[i], &GPIO_InitStruct);
		PUL_GPIO[i] -> BRR = PUL_Pin[i];									//设置默认低电平
	}

	// 配置方向引脚
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;				//低速切换
	for (u8 i = 0; i < 5; i++) {
		GPIO_InitStruct.GPIO_Pin = DIR_Pin[i];
		GPIO_Init(DIR_GPIO[i], &GPIO_InitStruct);
		DIR_GPIO[i] -> BRR = DIR_Pin[i];									//设置默认低电平
	}

	// 配置用到的通用定时器5
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue; 		//定义一个定时中断的结构体
	NVIC_InitTypeDef NVIC_InitStrue; 									//定义一个中断优先级初始化的结构体

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); 					//使能通用定时器5时钟

	TIM_TimeBaseInitStrue.TIM_Prescaler = 0; 											//预分频系数，决定每一个计数的时长
	TIM_TimeBaseInitStrue.TIM_Period = 72 - 1; 										//自动重装载寄存器：定时器从0开始计数，每次计时到1us时触发定时中断服务函数
	TIM_TimeBaseInitStrue.TIM_CounterMode = TIM_CounterMode_Up; 		//计数模式：向上计数
	TIM_TimeBaseInitStrue.TIM_ClockDivision = TIM_CKD_DIV1; 				//一般不使用，默认TIM_CKD_DIV1
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStrue); 			//根据TIM_TimeBaseInitStrue的参数初始化定时器TIM5

	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE); 						//使能TIM5中断，中断模式为更新中断：TIM_IT_Update

	NVIC_InitStrue.NVIC_IRQChannel = TIM5_IRQn; 						//TIM5中断
	NVIC_InitStrue.NVIC_IRQChannelCmd = ENABLE; 						//中断使能
	NVIC_InitStrue.NVIC_IRQChannelPreemptionPriority = 2; //抢占优先级为1级，值越小优先级越高，0级优先级最高
	NVIC_InitStrue.NVIC_IRQChannelSubPriority = 2; 				//响应优先级为0级，值越小优先级越高，0级优先级最高
	NVIC_Init(&NVIC_InitStrue); 													//根据NVIC_InitStrue的参数初始化VIC寄存器，设置TIM5中断

	TIM_Cmd(TIM5, ENABLE); //使能定时器TIM5
}


/**
  * @brief  设置步进电机转速rpm（revolutions per minute每分钟多少圈）
  * @param  i   	电机编号[0,4]
  * @param  rpm   电机转速rpm
  * @note   旋转周期T = 60 * 1,000,000 / n  微秒us
            一个脉冲周期步进电机旋转的角度A = 步进角 / 细分数；
            旋转一圈需要的脉冲个数N = 360 / A；
            一个脉冲周期 = T / N us = 500,000 * STEPANGLE / (3 * SUBDIV * n).
  * @retval 无
  */
void Stepper_SetRPM(u8 i, u16 rpm) {
	if (i > 4)			return;				//输入错误
	RotationRate[i] = rpm;
	PulsePeriod[i] = 500000 * STEPANGLE / (3 * SUBDIV * rpm);
}

/**
  * @brief  设置步进电机脉冲周期（直接决定转速）
  * @param  i   	电机编号[0,4]
  * @param  n   	输出的脉冲周期，单位 us
  * @note   计算同上
  * @retval 无
  */
void Stepper_SetPulsePeriod(u8 i, u16 n) {
	if (i > 4)			return;				//输入错误
	PulsePeriod[i] = n;
	RotationRate[i] = 500000 * STEPANGLE / (3 * SUBDIV * n);
}


/**
  * @brief  步进电机旋转
  * @param  i   		电机编号[0,4]
  * @param  dir   	选择正反转 (CLOCKWISE / ANTICLOCKWISE [1,0])
  * @param  angle  	需要转动的角度 (角度制)
  * @note   无
  * @retval 无
  */
void Stepper_Turn(u8 i, u8 dir, float angle) {
	if (dir == CLOCKWISE)				//顺时针
		CW(i);
	else												//逆时针
		ACW(i);

	u16 delay = 72 * 10;				//软延时，约10us
	while (delay--);							//为保证电机可靠换向，方向信号应先于脉冲信号至少 5 us 建立

	usTimer[i] = 0;							//当前脉冲半周期过去的微秒数置零
	PulseNum[i] = 0;						//已经发送的脉冲数置零
	

	/* 求出总共需要多少个脉冲信号 */
	TargetPulse[i] = angle / STEPANGLE * SUBDIV;		//一个脉冲周期步进电机旋转的角度 = 步进角 / 细分数
	/* 求出总共需要翻转多少次电平 */
	TargetPulse[i] <<= 1;														//一个脉冲要翻转两次电平
	
	StartStepper[i] = 1;				//开启第i个步进电机

}

/**
  * @brief  TIM5的中断服务函数
  * @param  无
  * @note   每1us产生一次中断后，累计当前时间。每半个脉冲周期翻转一次电平，实现用GPIO口模拟占空比50%的PWM输出。
  * @retval 无
  */
void TIM5_IRQHandler() {
	if (TIM_GetITStatus(TIM5, TIM_IT_Update)) {			//当发生中断时状态寄存器(TIMx_SR)的bit0会被硬件置1
		for (u8 i = 0; i < 5; i++) {
			if (StartStepper[i]) {
				usTimer[i]++;									//过去了1us
				if (usTimer[i] == (PulsePeriod[i] >> 1) && PulseNum[i] < TargetPulse[i]) {
					Stepper_Pul_TGL(i);					//翻转脉冲电平
					PulseNum[i]++;
					usTimer[i] = 0;
				}
				if (PulseNum[i] == TargetPulse[i]) {
					StartStepper[i] = 0;				//所有脉冲发送完毕，关闭步进电机
					TargetPulse[i] = 0;					//目标发送脉冲数清零
					PulseNum[i] = 0;						//当前发送脉冲数清零
				}
			}
		}
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update); 			//清除中断标志位，状态寄存器(TIMx_SR)的bit0置0
	}
}

