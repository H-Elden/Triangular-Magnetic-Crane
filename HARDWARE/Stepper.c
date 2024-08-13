#include "stepper.h"

// 全局变量，用于记录定时，单位100ms
volatile uint32_t timer = 0;
uint32_t close_time[6];		//关闭步进的时间点

u8 weight[3];			//抓手是否带负载

void TIM5_Init(void) {
	// 启动时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	// 定时器基本配置
	TIM_TimeBaseStructure.TIM_Period = 1000 - 1; // 自动重装载值，定时器计数到1000时溢出
	TIM_TimeBaseStructure.TIM_Prescaler = 7200 - 1; // 预分频器，定时器时钟为10kHz
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数模式
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	// 定时器5中断配置
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// 使能定时器5更新中断
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);

	// 使能定时器5
	TIM_Cmd(TIM5, ENABLE);
}

void TIM5_IRQHandler(void) {
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
		timer++; // 更新溢出计数，每过去100ms(0.1s)就加1
	}
}


/**
  * @brief    位置模式
  * @param    addr：电机地址
  * @param    dir ：方向        ，0 CCW，其余值为CW
  * @param    vel ：速度(RPM)   ，范围0 - 5000RPM
  * @param    acc ：加速度      ，范围0 - 255，注意：0是直接启动
  * @param    clk ：脉冲数      ，范围0- (2^32 - 1)个
  * @param    raF ：相位/绝对标志，false为相对运动，true为绝对值运动
  * @param    snF ：多机同步标志 ，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Stepper_Turn(uint8_t addr, uint8_t dir, float angle) {
	uint8_t cmd[13] = {0};
	//脉冲数 = 角度 / 步进角（1.8） * 细分数（16）
	uint32_t clk = angle / 1.8 * 16;
	uint16_t vel;
	uint8_t acc;
	if(addr == 1 || addr == 2){			//水平运动
		vel = SVEL_S;
		acc = SACC_S;
	}
	else{														//垂直运动
		vel = SVEL_C;
		acc = SACC_C;	
	}

	// 装载命令
	cmd[0]  =  addr;                      // 地址
	cmd[1]  =  0xFD;                      // 功能码
	cmd[2]  =  dir;                       // 方向
	cmd[3]  =  (uint8_t)(vel >> 8);       // 速度(RPM)高8位字节
	cmd[4]  =  (uint8_t)(vel >> 0);       // 速度(RPM)低8位字节
	cmd[5]  =  acc;                       // 加速度，注意：0是直接启动
	cmd[6]  =  (uint8_t)(clk >> 24);      // 脉冲数(bit24 - bit31)
	cmd[7]  =  (uint8_t)(clk >> 16);      // 脉冲数(bit16 - bit23)
	cmd[8]  =  (uint8_t)(clk >> 8);       // 脉冲数(bit8  - bit15)
	cmd[9]  =  (uint8_t)(clk >> 0);       // 脉冲数(bit0  - bit7 )
	cmd[10] =  false;                     // 相位/绝对标志，false为相对运动，true为绝对值运动
	cmd[11] =  false;                     // 多机同步运动标志，false为不启用，true为启用
	cmd[12] =  0x6B;                      // 校验字节

	// 发送命令
	usart3_SendCmd(cmd, 13);
	u32 delay = 72 * 20;
	while(delay--);											//软延时，20us
	
	//计算关闭电机的时间
	if(addr == 1 || addr == 2){
		if(angle == S1)
			close_time[addr] = timer + TIME_S1;			//关闭电机的时间点 = 当前时间点 + 电机转动时间
		else if(angle == S2)
			close_time[addr] = timer + TIME_S2;
		else if(angle == S2 - S1)
			close_time[addr] = timer + TIME_S2_1;
	}
	else{
		if(angle == C1 || angle == C1 - 5)
			close_time[addr] = weight[addr - 3] ? timer + TIME_C1_W : timer + TIME_C1;
		else if(angle == C2 || angle == C2 - 5)
			close_time[addr] = weight[addr - 3] ? timer + TIME_C2_W : timer + TIME_C2;
		else if(angle == Z0)
			close_time[addr] = weight[addr - 3] ? timer + TIME_Z0_W : timer + TIME_Z0;
	}

}

/**
  * @brief    读取电机状态标志位
  * @param    addr：电机地址
  * @retval   1：正在旋转；0：旋转到位；2：错误
  */
uint8_t Stepper_GetStatus(uint8_t addr) {
	if(timer < close_time[addr] + 2)			//相当于多加200ms 	
		return 1;
	return 0;
}
