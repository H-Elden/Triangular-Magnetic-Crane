#include "stepper.h"

#if TIMER_ENABLE
// 全局变量，用于记录定时，单位100ms
volatile uint32_t timer = 0;
uint32_t close_time[6];		//关闭步进的时间点

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
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
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

#endif	/* TIMER_ENABLE */

/**
  * @brief    打印步进电机驱动板通过串口返回的信息
  * @param    无
  * @retval   无
  */
void Print_RxCmd(void) {
	for (u8 i = 0; i < rxCount; i++)
		printf("%02X ", rxCmd[i]);
	puts("");
}

/**
 * @brief    所有步进电机立即停止
 * @param    无
 * @retval   无
 */
void Stepper_StopNow(void) {
	uint8_t cmd[5] = {0};

	// 装载命令
	cmd[0] =  0x00;                       // 广播，所有步进都执行
	cmd[1] =  0xFE;                       // 功能码
	cmd[2] =  0x98;                       // 辅助码
	cmd[3] =  0x00;                       // 多机同步运动标志，不启用
	cmd[4] =  0x6B;                       // 校验字节

	// 发送命令
	usart3_SendCmd(cmd, 5);
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
	uint16_t vel = SVEL;
	uint8_t acc = SACC;
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

	while (rxFrameFlag == false);
	Print_RxCmd();											  //打印串口信息
	rxFrameFlag = false;									// 清除接收标志
	
#if TIMER_ENABLE
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
		if(angle == C1)
			close_time[addr] = timer + TIME_C1;
		else if(angle == C2)
			close_time[addr] = timer + TIME_C2;
		else if(angle == Z0)
			close_time[addr] = timer + TIME_Z0;
	}
#endif
}

/**
  * @brief    读取电机状态标志位
  * @param    addr：电机地址
  * @retval   1：正在旋转；0：旋转到位；2：错误
  */
uint8_t Stepper_GetStatus(uint8_t addr) {
#if TIMER_MEASURE
	uint8_t cmd[3] = {0};

	// 装载命令
	cmd[0]  =  addr;                      // 地址
	cmd[1]  =  0x3A;                      // 功能码
	cmd[2]  =  0x6B;                      // 校验字节

	// 发送命令
	usart3_SendCmd(cmd, 3);

	while (rxFrameFlag == false);
	rxFrameFlag = false;									// 清除接收标志

	if (rxCmd[1]) {
		if (rxCmd[2] & 0x02) {
			printf("%d Stepper Stop\r\n", addr);
			return 0;				// 电机旋转到位
		}
		return 1;														// 电机旋转未到位
	}
	return 2;														// 错误
	
#else
	return timer < close_time[addr] + 5;			//相当于多加500ms 
#endif
}

#if TIMER_MEASURE
/**
  * @brief    测量步进电机的运行时间
  * @param    无
  * @retval   无
	* @note			使用说明：修改stepper.h的宏定义，1 1
						在主函数while(1)循环前面调用此函数，机器所有抓手摆放
						至初始位置，打开串口等待输出测量结果。
  */
void Stepper_TimerINIT(){
	/* 先向上抬够高 */
	Stepper_Turn(3, UP3, C1);
	Stepper_Turn(4, UP4, C1);
	Stepper_Turn(5, UP0, Z0);
	while(Stepper_GetStatus(3) || Stepper_GetStatus(4) || Stepper_GetStatus(5));
	
	u32 t[6] = {0},temp,i;
	/* TIME_S1 = t[0] */
	for(i = 0;i < 2;i++){
		temp = timer;
		Stepper_Turn(1,WAI1,S1);
		Stepper_Turn(2,WAI2,S1);
		while(Stepper_GetStatus(1) || Stepper_GetStatus(2));			//阻塞等待停止
		t[0] += timer - temp;
		printf("t[0] = %d\r\n",timer - temp);
		temp = timer;
		Stepper_Turn(1,NEI1,S1);
		Stepper_Turn(2,NEI2,S1);
		while(Stepper_GetStatus(1) || Stepper_GetStatus(2));			//阻塞等待停止
		t[0] += timer - temp;
		printf("t[0] = %d\r\n",timer - temp);
	}
	/* TIME_S2 = t[1] */
	for(i = 0;i < 2;i++){
		temp = timer;
		Stepper_Turn(1,WAI1,S2);
		Stepper_Turn(2,WAI2,S2);
		while(Stepper_GetStatus(1) || Stepper_GetStatus(2));			//阻塞等待停止	
		t[1] += timer - temp;
		printf("t[1] = %d\r\n",timer - temp);

		temp = timer;
		Stepper_Turn(1,NEI1,S2);
		Stepper_Turn(2,NEI2,S2);
		while(Stepper_GetStatus(1) || Stepper_GetStatus(2));			//阻塞等待停止	
		t[1] += timer - temp;
		printf("t[1] = %d\r\n",timer - temp);
	}
	/* TIME_S2_1 = t[2] */
	for(i = 0;i < 2;i++){
		temp = timer;
		Stepper_Turn(1,WAI1,S2 - S1);
		Stepper_Turn(2,WAI2,S2 - S1);
		while(Stepper_GetStatus(1) || Stepper_GetStatus(2));			//阻塞等待停止	
		t[2] += timer - temp;
		printf("t[2] = %d\r\n",timer - temp);
		
		temp = timer;
		Stepper_Turn(1,NEI1,S2 - S1);
		Stepper_Turn(2,NEI2,S2 - S1);
		while(Stepper_GetStatus(1) || Stepper_GetStatus(2));			//阻塞等待停止	
		t[2] += timer - temp;
		printf("t[2] = %d\r\n",timer - temp);
	}
	
	/* 回到地面 */
	Stepper_Turn(3, DOWN3, C1);
	Stepper_Turn(4, DOWN4, C1);
	Stepper_Turn(5, DOWN0, Z0);
	while(Stepper_GetStatus(3) || Stepper_GetStatus(4) || Stepper_GetStatus(5));
	
	/* TIME_C1 = t[3] */
	for(i = 0;i < 2;i++){
		temp = timer;
		Stepper_Turn(3,UP3,C1);
		Stepper_Turn(4,UP4,C1);
		while(Stepper_GetStatus(3) || Stepper_GetStatus(4));			//阻塞等待停止	
		t[3] += timer - temp;
		printf("t[3] = %d\r\n",timer - temp);
		
		temp = timer;
		Stepper_Turn(3,DOWN3,C1);
		Stepper_Turn(4,DOWN4,C1);
		while(Stepper_GetStatus(3) || Stepper_GetStatus(4));			//阻塞等待停止	
		t[3] += timer - temp;
		printf("t[3] = %d\r\n",timer - temp);
	}
	/* TIME_C2 = t[4] */
	for(i = 0;i < 2;i++){
		temp = timer;
		Stepper_Turn(3,UP3,C2);
		Stepper_Turn(4,UP4,C2);
		while(Stepper_GetStatus(3) || Stepper_GetStatus(4));			//阻塞等待停止	
		t[4] += timer - temp;
		printf("t[4] = %d\r\n",timer - temp);
		
		temp = timer;
		Stepper_Turn(3,DOWN3,C2);
		Stepper_Turn(4,DOWN4,C2);
		while(Stepper_GetStatus(3) || Stepper_GetStatus(4));			//阻塞等待停止	
		t[4] += timer - temp;
		printf("t[4] = %d\r\n",timer - temp);
	}
	/* TIME_Z0 = t[5] */
	for(i = 0;i < 2;i++){
		temp = timer;
		Stepper_Turn(5,UP0,Z0);
		while(Stepper_GetStatus(5));			//阻塞等待停止	
		t[5] += timer - temp;
		printf("t[5] = %d\r\n",timer - temp);
		
		temp = timer;
		Stepper_Turn(5,DOWN0,Z0);
		while(Stepper_GetStatus(5));			//阻塞等待停止	
		t[5] += timer - temp;
		printf("t[5] = %d\r\n",timer - temp);
	}
	/* 求平均数 */
	for(i = 0;i<6;i++)
		t[i] >>= 2;
	/* 输出测量结果 */
	printf("----- 测量完成！ -----\r\n");
	printf("/* 基于速度SVEL = %d 加速度SACC = %d 的测量结果如下 */\r\n", SVEL, SACC);
	printf("#define TIME_S1\t\t%d\r\n",t[0]);
	printf("#define TIME_S2\t\t%d\r\n",t[1]);
	printf("#define TIME_S2_1\t%d\r\n",t[2]);
	printf("#define TIME_C1\t\t%d\r\n",t[3]);
	printf("#define TIME_C2\t\t%d\r\n",t[4]);
	printf("#define TIME_Z0\t\t%d\r\n",t[5]);
}
#endif	/* TIMER_MEASURE */


