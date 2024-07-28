#include "motorencoder.h"

int LTargetVelocity, RTargetVelocity, LTargetVelocity_f, RTargetVelocity_f; //目标速度
float LTargetCircle, RTargetCircle;						//目标圈数(位置)
int LCurrentPosition, RCurrentPosition, LCurrentPosition_V, RCurrentPosition_V;
int LEncoder, REncoder;												//编码器读数
int LPWM, RPWM; 															//PWM 控制变量
float Velcity_Kp = 1, Velcity_Ki = 0.5, Velcity_Kd; 							//相关速度 PID 参数
float Position_Kp = 0.25, Position_Ki = 0.02, Position_Kd = 1; 		//相关位置 PID 参数
int Fudu_flag = 0;
int Run_flag = 0;
int Velocity_temp;
int Velocity_flag;
int MVEL_flag = 0;
int LStartMinV, RStartMinV; //初始最小速度为目标速度的五分之一
float ZhongZhi; 		//单轴陀螺仪定义走直中值

float Run_Dis;			//正数表示正向行进总距离(A点为0)，负数表示反向行进总距离(H点为0)。单位mm
float Con_Dis;			//继续行走 Con_Dis 毫米的距离停车

MotorState_t MotorState = Stop;		//默认停车

/*

根据测量得到的经验公式：
	设置速度为u，则实际速度
				v = 3.57e-4 * u （单位m/s）
*/


/**
  * @brief  起重机从停车起步，后保持一定速度前进
  * @prarm  dir 0表示正向行进，1表示负向行进
	* @prarm  vel 表示速度
  * @retval 无
  */
void Motor_Run(uint8_t dir, uint16_t vel) {
	puts("Motor_Run");
	Velocity_flag = 1;
	Fudu_flag = 1;
	MotorState = Velocity_Xunji;
	switch (dir) {
		case 0: {
			LTargetVelocity_f = -vel;
			LStartMinV = -vel / 4;
			RTargetVelocity_f = -LTargetVelocity_f;
			RStartMinV = -LStartMinV;
			break;
		}
		case 1: {
			LTargetVelocity_f = vel;
			LStartMinV = vel / 4;
			RTargetVelocity_f = -LTargetVelocity_f;
			RStartMinV = -LStartMinV;
			break;
		}
	}

}

/**
  * @brief  起重机继续行进一定距离后停车
  * @prarm  dis 继续行进的距离，单位mm
  * @retval 无
	* @note		这段距离包含减速的过程，在dis内准确停车
  */
void Con_Stop(float dis) {
	MotorState = VelCir;
	if (LTargetVelocity_f < 0) {
		LTargetCircle = -1.0 * dis / (2 * PI * Radius);
		RTargetCircle = -LTargetCircle;
	} else {
		LTargetCircle = 1.0 * dis / (2 * PI * Radius);
		RTargetCircle = -LTargetCircle;
	}

}

/**
  * @brief  起重机忙走行进一定距离后停车
  * @prarm  dis 继续行进的距离，单位mm
  * @retval 无
	* @note		这段距离包含加减速的过程，在dis内准确停车
  */
void Run(uint8_t dir, float dis, uint16_t vel) {
	printf("Run %.1f mm\r\n", dis);
	Run_flag = 1;
	switch (dir) {
		case 0: {
			Motor_Run(0, vel);
			Con_Stop(dis);
			break;
		}
		case 1: {
			Motor_Run(1, vel);
			Con_Stop(dis);
			break;
		}

	}
}
void Speed_UP(float accel) {

	LTargetVelocity = accel * LCurrentPosition_V / 54000 + LStartMinV;
	RTargetVelocity = - LTargetVelocity;
	if (abs(LTargetVelocity) >= abs(LTargetVelocity_f)) {
        if(Run_flag)
            MVEL_flag = 1;
		LTargetVelocity = LTargetVelocity_f;
		RTargetVelocity = RTargetVelocity_f;
	}
}

void Speed_DOWN(float dccel) {
	if (Velocity_flag) {
		Velocity_temp = (abs(LTargetVelocity) + abs(RTargetVelocity)) / 2;
		Velocity_flag = 0;
		LCurrentPosition_V = 0;
	}
    if(Run_flag && MVEL_flag)
        dccel -= 700;
	if (LTargetVelocity_f > 0) {
		LTargetVelocity = Velocity_temp - fabs(dccel * LCurrentPosition_V / 54000);
	} else {
		LTargetVelocity = -Velocity_temp + fabs(dccel * LCurrentPosition_V / 54000);
	}

	RTargetVelocity = -LTargetVelocity;
	if (abs(LTargetVelocity) < abs(LStartMinV)) {
		LTargetVelocity = LStartMinV;
		RTargetVelocity = RStartMinV;
	}

}
/**************************************************************************
函数功能：TIM6 中断服务函数 定时读取编码器数值并进行速度闭环控制 10ms进入一次
入口参数：无
返回  值：无
**************************************************************************/
void TIM6_IRQHandler() {
	/***闭环速度***/
	if (TIM_GetITStatus(TIM6, TIM_IT_Update) == 1) { //当发生中断时状态寄存器(TIMx_SR)的bit0会被硬件置1
		LEncoder = LRead_Encoder();   		//读取当前编码器读数，即速度
		REncoder = RRead_Encoder();   		//读取当前编码器读数，即速度
		if (Fudu_flag) {
			LCurrentPosition_V += LEncoder;
			if (Run_flag) {
				if (fabs(1.0 * LCurrentPosition / 54000) < fabs(LTargetCircle / 2))
					Speed_UP(Accel);
				else
					Speed_DOWN(Accel - 200);//这个幅度必须和加速幅度一样
			} else {
				Speed_UP(Accel);
				if (MotorState == VelCir)
					Speed_DOWN(Dccel);
			}
		}
		Run_Dis += (1.0 * REncoder / 54000) * 2 * PI * Radius;  //Run_Dis全局变量往后修改
		if (Sensor_open)
			Getdis();
		switch (MotorState) {
			/*----闭环速度----*/
			case Stop: {
				LTargetVelocity = 0;
				RTargetVelocity = 0;

				LTargetVelocity_f = 0;
				RTargetVelocity_f = 0;

				LCurrentPosition = 0;
				RCurrentPosition = 0;

				Run_flag = 0;
				Fudu_flag = 0;
                MVEL_flag = 0;
				LCurrentPosition_V = 0;

				LPWM = LVelocity_FeedbackControl(LTargetVelocity, LEncoder); 		//速度环闭环控制
				RPWM = RVelocity_FeedbackControl(RTargetVelocity, REncoder); 		//速度环闭环控制
				SetPWM(LPWM, RPWM); 				//设置PWM  若现在速度一直达不到目标速度，则pwm数值累加
				//一直加到65536后，pwm会被自动置0  因为l298n的定时器
				break;
			}
			case Velocity_Xunji: {
				Gyro_read();
//					printf("%d    %.3f\r\n", Turn(fAngle[2]), fAngle[2]);
				LTargetVelocity = LTargetVelocity - Turn(fAngle[2]);
				RTargetVelocity = RTargetVelocity - Turn(fAngle[2]);
				LPWM = LVelocity_FeedbackControl(LTargetVelocity, LEncoder); 		//速度环闭环控制
				RPWM = RVelocity_FeedbackControl(RTargetVelocity, REncoder); 		//速度环闭环控制
				SetPWM(LPWM, RPWM); 				//设置PWM  若现在速度一直达不到目标速度，则pwm数值累加
				break;
			}
			/*----速度环 + 位置环----*/
			case VelCir: {
				static int LPWM_P, LPWM_V, RPWM_P, RPWM_V; 				//速度位置串级 PID 控制变量 PWM_P、PWM_V
				//static int TimeCount;

				LCurrentPosition += LEncoder; 	//编码器读数(速度)积分得到位置
				RCurrentPosition += REncoder; 	//编码器读数(速度)积分得到位置

				if (fabs(LTargetCircle * 54000 - LCurrentPosition) < 5400 && LEncoder == 0) { //当编码器读数为0即为停车
					MotorState = Stop;
					puts("stopFlag");
				}

				LPWM_P = LPosition_FeedbackControl(LTargetCircle, LCurrentPosition); //位置闭环控制
				LPWM_P = Velocity_Restrict(LPWM_P, abs(LTargetVelocity)); //限幅位置环输出的 PWM
				LPWM_V = LPWM_P / 2; //位置环输出的 PWM 值按一定比例转换为速度值，接下来使用该速度值进行速度闭环控制。
				//空载时，电源适配器供电情况下，每 76PWM 约等于 1 编码器速度（后期还需要改动）
				LPWM = LVelocity_FeedbackControl(LPWM_V, LEncoder); //速度环闭环控制 相当于位置环的输出为速度环的输入，形成串级 PID

				RPWM_P = RPosition_FeedbackControl(RTargetCircle, RCurrentPosition); //位置闭环控制
				RPWM_P = Velocity_Restrict(RPWM_P, abs(RTargetVelocity)); //限幅位置环输出的 PWM
				RPWM_V = RPWM_P / 2; //位置环输出的 PWM 值按一定比例转换为速度值，接下来使用该速度值进行速度闭环控制。
				//空载时，电源适配器供电情况下，每 76PWM 约等于 1 编码器速度（后期还需要改动）
				RPWM = RVelocity_FeedbackControl(RPWM_V, REncoder); //速度环闭环控制 相当于位置环的输出为速度环的输入，形成串级 PID
				SetPWM(LPWM, RPWM); //设置 PWM

				break;
			}

		}

		TIM_ClearITPendingBit(TIM6, TIM_IT_Update); 			//清除中断标志位，状态寄存器(TIMx_SR)的bit0置0
	}

}

/**************************************************************************
函数功能：编码器初始化函数
入口参数：无
返回  值：无
**************************************************************************/
void MotorEncoder_Init_TIM4(void) { //左电机
	//左电机

	NVIC_InitTypeDef NVIC_InitStruct;
	GPIO_InitTypeDef GPIO_InitStructure; //定义一个引脚初始化的结构体
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;//定义一个定时器初始化的结构体
	TIM_ICInitTypeDef TIM_ICInitStructure; //定义一个定时器编码器模式初始化的结构体

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //使能TIM4时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能CPIOB时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;	//TIM4_CH1、TIM4_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);	//根据GPIO_InitStructure的参数初始化GPIO

	NVIC_InitStruct.NVIC_IRQChannel = TIM4_IRQn;             //TIM4中断
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;             //使能IRQ通道
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;   //抢占优先级 1
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;          //响应优先级3
	NVIC_Init(&NVIC_InitStruct);


	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = 0xffff; //设定计数器自动重装值
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // 预分频器
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //选择时钟分频：不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct的参数初始化定时器TIM4

	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //使用编码器模式3：CH1、CH2同时计数，为四分频
	TIM_ICStructInit(&TIM_ICInitStructure); //把TIM_ICInitStruct 中的每一个参数按缺省值填入
	TIM_ICInitStructure.TIM_ICFilter = 10;  //设置滤波器长度
	TIM_ICInit(TIM4, &TIM_ICInitStructure); //根TIM_ICInitStructure参数初始化定时器TIM4编码器模式

	TIM_ClearFlag(TIM4, TIM_FLAG_Update);//清除TIM的更新标志位
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); //更新中断使能
	TIM_SetCounter(TIM4, 0); //初始化清空编码器数值

	TIM_Cmd(TIM4, ENABLE); //使能定时器4


}
void MotorEncoder_Init_TIM2(void) { //右电机
	NVIC_InitTypeDef NVIC_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	//右电机
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //使能TIM2时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //使能CPIOB时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;	//TIM2_CH1、TIM2_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);	//根据GPIO_InitStructure的参数初始化GPIO

	NVIC_InitStruct.NVIC_IRQChannel = TIM2_IRQn;  //定时器2中断
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;  //使能IRQ通道
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;//抢占优先级1
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;       //响应优先级3
	NVIC_Init(&NVIC_InitStruct);


	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = 0xffff; //设定计数器自动重装值
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // 预分频器
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //选择时钟分频：不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct的参数初始化定时器TIM4

	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //使用编码器模式3：CH1、CH2同时计数，为四分频
	TIM_ICStructInit(&TIM_ICInitStructure); //把TIM_ICInitStruct 中的每一个参数按缺省值填入
	TIM_ICInitStructure.TIM_ICFilter = 10;  //设置滤波器长度
	TIM_ICInit(TIM2, &TIM_ICInitStructure); //根TIM_ICInitStructure参数初始化定时器TIM4编码器模式

	TIM_ClearFlag(TIM2, TIM_FLAG_Update);//清除TIM的更新标志位
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); //更新中断使能
	TIM_SetCounter(TIM2, 0); //初始化清空编码器数值

	TIM_Cmd(TIM2, ENABLE); //使能定时器2
}

/**************************************************************************
函数功能：读取TIM4编码器数值
入口参数：无
返回  值：无
**************************************************************************/
int LRead_Encoder(void) {
	int Encoder_TIM;
	Encoder_TIM = TIM4->CNT; //读取计数
	if (Encoder_TIM > 0xefff)Encoder_TIM = Encoder_TIM - 0xffff; //转化计数值为有方向的值，大于0正转，小于0反转。
	//TIM4->CNT范围为0-0xffff，初值为0。
	TIM4->CNT = 0; //读取完后计数清零
	return Encoder_TIM; //返回值
}

/**************************************************************************
函数功能：TIM4中断服务函数
入口参数：无
返回  值：无
**************************************************************************/
void TIM4_IRQHandler(void) {
	if (TIM4->SR & 0X0001) { //溢出中断
	}
	TIM4->SR &= ~(1 << 0); //清除中断标志位
}

/**************************************************************************
函数功能：读取TIM2编码器数值
入口参数：无
返回  值：无
**************************************************************************/
int RRead_Encoder(void) {
	int Encoder_TIM;
	Encoder_TIM = TIM2->CNT; //读取计数
	if (Encoder_TIM > 0xefff)Encoder_TIM = Encoder_TIM - 0xffff; //转化计数值为有方向的值，大于0正转，小于0反转。
	//TIM4->CNT范围为0-0xffff，初值为0。
	TIM2->CNT = 0; //读取完后计数清零
	return Encoder_TIM; //返回值
}

/**************************************************************************
函数功能：TIM2中断服务函数
入口参数：无
返回  值：无
**************************************************************************/
void TIM2_IRQHandler(void) {
	if (TIM2->SR & 0X0001) { //溢出中断
	}
	TIM2->SR &= ~(1 << 0); //清除中断标志位
}

/**************************************************************************
函数功能：通用定时器6初始化函数，
入口参数：自动重装载值 预分频系数 默认定时时钟为72MHZ时，两者共同决定定时中断时间
返回  值：无
**************************************************************************/
void TIM6_Init() {
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue; //定义一个定时中断的结构体
	NVIC_InitTypeDef NVIC_InitStrue; //定义一个中断优先级初始化的结构体

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); //使能通用定时器2时钟

	TIM_TimeBaseInitStrue.TIM_Period = 7200 - 1; //计数模式为向上计数时，定时器从0开始计数，计数超过到arr时触发定时中断服务函数
	TIM_TimeBaseInitStrue.TIM_Prescaler = 100 - 1; //预分频系数，决定每一个计数的时长
	TIM_TimeBaseInitStrue.TIM_CounterMode = TIM_CounterMode_Up; //计数模式：向上计数
	TIM_TimeBaseInitStrue.TIM_ClockDivision = TIM_CKD_DIV1; //一般不使用，默认TIM_CKD_DIV1
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseInitStrue); //根据TIM_TimeBaseInitStrue的参数初始化定时器TIM2

	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE); //使能TIM2中断，中断模式为更新中断：TIM_IT_Update

	NVIC_InitStrue.NVIC_IRQChannel = TIM6_IRQn; //属于TIM2中断
	NVIC_InitStrue.NVIC_IRQChannelCmd = ENABLE; //中断使能
	NVIC_InitStrue.NVIC_IRQChannelPreemptionPriority = 0; //抢占优先级为1级，值越小优先级越高，0级优先级最高
	NVIC_InitStrue.NVIC_IRQChannelSubPriority = 0; //响应优先级为1级，值越小优先级越高，0级优先级最高【【【TIM6的中断优先级应高于TIM5，否则会出现卡顿】】】
	NVIC_Init(&NVIC_InitStrue); //根据NVIC_InitStrue的参数初始化VIC寄存器，设置TIM2中断

	TIM_Cmd(TIM6, ENABLE); //使能定时器TIM2
}

/**************************************************************************
函数功能：速度闭环PID控制(实际为PI控制)
入口参数：目标速度 当前速度
返回  值：速度控制值
根据增量式离散PID公式
ControlVelocity+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差
e(k-1)代表上一次的偏差  以此类推
ControlVelocity代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
ControlVelocity+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int LVelocity_FeedbackControl(int LTargetVelocity, int LCurrentVelocity) {
	int LBias;  //定义相关变量
	static int LControlVelocity, LLast_bias; //静态变量，函数调用结束后其值依然存在

	LBias = LTargetVelocity - LCurrentVelocity; //求速度偏差
	LControlVelocity += Velcity_Kp * (LBias - LLast_bias) + Velcity_Ki * LBias; //增量式PI控制器
	//Velcity_Kp*(Bias-Last_bias) 作用为限制加速度
	//Velcity_Ki*Bias             速度控制值由Bias不断积分得到 偏差越大加速度越大
	LLast_bias = LBias;
	return LControlVelocity; //返回速度控制值
}

int RVelocity_FeedbackControl(int RTargetVelocity, int RCurrentVelocity) {
	int RBias;  //定义相关变量
	static int RControlVelocity, RLast_bias; //静态变量，函数调用结束后其值依然存在

	RBias = RTargetVelocity - RCurrentVelocity; //求速度偏差
	RControlVelocity += Velcity_Kp * (RBias - RLast_bias) + Velcity_Ki * RBias; //增量式PI控制器
	//Velcity_Kp*(Bias-Last_bias) 作用为限制加速度
	//Velcity_Ki*Bias             速度控制值由Bias不断积分得到 偏差越大加速度越大
	RLast_bias = RBias;
	return RControlVelocity; //返回速度控制值
}

/**************************************************************************
函数功能：位置式 PID 控制器
入口参数：目标圈数(位置) 当前位置
返回 值：速度控制值
根据位置式离散 PID 公式
pwm=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]
e(k)代表本次偏差
e(k-1)代表上一次的偏差
∑e(k)代表 e(k)以及之前的偏差的累积和;其中 k 为 1,2,,k;
pwm 代表输出
**************************************************************************/
//注意此处添加了电机误差系数 54
int LPosition_FeedbackControl(float Circle, int CurrentPosition) {
	float TargetPosition, Bias, ControlVelocity; //定义相关变量
	static float Last_bias, Integral_Bias; //静态变量，函数调用结束后其值依然存在
	TargetPosition = Circle * 54000; //目标位置=目标圈数*54000
	//10ms 读取一次编码器(即 100HZ)，电机减速比为 27，gmr编码器精度 500，AB 双相组合得到 4 倍频，
	//则转 1 圈编码器读数为 27*500*4=54000，电机转速=Encoder*100/54000r/s 使用定时器 2
	//54 是误差系数，电机本身存在误差，可根据实际情况调整该系数以提高控制精度
	Bias = TargetPosition - CurrentPosition; //求位置偏差
	Integral_Bias += Bias;
	if (Integral_Bias > 970) Integral_Bias = 970; //积分限幅 防止到达目标位置后过冲
	if (Integral_Bias < -970) Integral_Bias = -970; //积分限幅 防止到达目标位置后过冲
	ControlVelocity = Position_Kp * Bias + Position_Ki * Integral_Bias + Position_Kd * (Bias - Last_bias); //增量式 PI 控制器
	//Position_Kp*Bias 偏差越大速度越大
	//Position_Ki*Integral_Bias 减小稳态误差
	//Position_Kd*(Bias-Last_bias) 限制速度
	Last_bias = Bias;
	return ControlVelocity; //返回速度控制值

}

int RPosition_FeedbackControl(float Circle, int CurrentPosition) {
	float TargetPosition, Bias, ControlVelocity; //定义相关变量
	static float Last_bias, Integral_Bias; //静态变量，函数调用结束后其值依然存在
	TargetPosition = Circle * 54000; //目标位置=目标圈数*54000
	//10ms 读取一次编码器(即 100HZ)，电机减速比为 27，gmr编码器精度 500，AB 双相组合得到 4 倍频，
	//则转 1 圈编码器读数为 27*500*4=54000，电机转速=Encoder*100/54000r/s 使用定时器 2
	//54 是误差系数，电机本身存在误差，可根据实际情况调整该系数以提高控制精度
	Bias = TargetPosition - CurrentPosition; //求位置偏差
	Integral_Bias += Bias;
	if (Integral_Bias > 970) Integral_Bias = 970; //积分限幅 防止到达目标位置后过冲
	if (Integral_Bias < -970) Integral_Bias = -970; //积分限幅 防止到达目标位置后过冲
	ControlVelocity = Position_Kp * Bias + Position_Ki * Integral_Bias + Position_Kd * (Bias - Last_bias); //增量式 PI 控制器
	//Position_Kp*Bias 偏差越大速度越大
	//Position_Ki*Integral_Bias 减小稳态误差
	//Position_Kd*(Bias-Last_bias) 限制速度
	Last_bias = Bias;
	return ControlVelocity; //返回速度控制值

}

/**************************************************************************
函数功能：速度(PWM)限幅
入口参数：PWM_P:位置环输出的PWM值 TargetVelocity:目标速度(速度限制值)
返回  值：无
**************************************************************************/
int Velocity_Restrict(int PWM_P, int TargetVelocity) {
	if (PWM_P > +TargetVelocity * 2) PWM_P = +TargetVelocity * 2;
	else if (PWM_P < -TargetVelocity * 2) PWM_P = -TargetVelocity * 2;
	else PWM_P = PWM_P;
	return PWM_P;
}

/**************************************************************************
函数功能：转向控制  巡线
入口参数：CCD提取的中线 Z轴陀螺仪
返回  值：转向控制PWM
作    者：平衡小车之家
**************************************************************************/
int Turn(float YAW) { //转向控制
	float Bias;
	int Kp = 500, Kd = 2000;
	static float Turn, Last_Bias;
	Bias = YAW - ZhongZhi;
//    Integral_Bias+=Bias;
//    if(Integral_Bias> 970) Integral_Bias= 970; //积分限幅 防止到达目标位置后过冲
//    if(Integral_Bias<-970) Integral_Bias=-970; //积分限幅 防止到达目标位置后过冲
	Turn = Kp * Bias + Kd * (Bias - Last_Bias);
	Last_Bias = Bias;
	if (Turn > 100) Turn = 100;
	if (Turn < -100) Turn = -100;
	return Turn;
}
