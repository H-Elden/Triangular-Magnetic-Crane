/*----- 整机运行过程 -----*/

#include "Process.h"

/**
  * @brief  主程序初始化
  * @prarm  无
  * @retval 无
  */
void Init() {
#if TIMER_ENABLE
	TIM5_Init();								//全局定时器，用于判断步进是否到位	使用定时器5
#endif
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //中断优先级分组2
	delay_init(); 							//延迟函数初始化
	LED_Init(); 								//LED灯初始化
	KEY_GPIO_Init();						//按键KEY初始化
	Usart1_Init(115200); 				//串口初始化
	Usart2Init(9600);						//串口2初始化接传感器端
	Usart3_Init();							//串口3初始化，接步进电机驱动板
	puts("-----INIT-----");


	WitInit(WIT_PROTOCOL_NORMAL, 0x50);				//初始化标准协议，设置设备地址
	WitSerialWriteRegister(SensorUartSend);		//注册写回调函数    串口1接收数据调用 SensorUartSend函数
	WitRegisterCallBack(SensorDataUpdata);		//注册获取传感器数据回调函数   串口2接收数据调用SensorDataUpdata函数
	WitDelayMsRegister(Delayms);							//注册延时回调函数
	printf("\r\n********************** wit-motion normal example  ************************\r\n");//打印输出
	AutoScanSensor();//自动搜索传感器，如果线没有插对或者使用的串口工具不对会搜索不到传感器
	WitSetOutputRate(RRATE_10HZ);//回传速率设置为10赫兹
	WitSetUartBaud(WIT_BAUD_115200);//波特率设置为115200
	delay_ms(1000);//延迟等待初始化完成
	Gyro_read();
	ZhongZhi = fAngle[2];
	delay_ms(10);
	Gyro_read();
	ZhongZhi = fAngle[2];
	printf("ZhongZhi = %.3f\r\n", ZhongZhi);

	PointDis_Init();						//点位距离初始化
	TIM6_Init(); 								//10ms 读取一次编码器(即100Hz)，使用定时器6
	TIM7_Init();								//用于超声波计数 7
	MotorEncoder_Init_TIM2(); 	//编码器初始化 使用定时器 2
	MotorEncoder_Init_TIM4(); 	//编码器初始化 使用定时器 4
	Motor_Init();								//电机初始化
	LL298N_Init(7199, 0); 			//电机驱动外设初始化 使用定时器 3
	RL298N_Init(7199, 0); 			//电机驱动外设初始化 使用定时器 8
	HCSR04_Init();							//初始化超声波
	Switch_Init();							//初始化电磁继电器

	uint8_t cmd[3] = {0x05, 0x3A, 0x6B};    // 读取电机状态的命令
	usart3_SendCmd(cmd, 3);                 // 向步进驱动板发送命令
	LED_RED = 0;                            // 红灯亮起，检测是否电机上电
	while (rxFrameFlag == false);           // 如果没有上电会一直循环等待，红灯常亮
	rxFrameFlag = false;                    // 清除接收标志
	LED_RED = 1;                            // 红灯熄灭，说明步进上电了
}

/**
  * @brief  点位距离初始化
  * @prarm  无
  * @retval 无
  */
float PointDis[5][3];
void PointDis_Init() {
	PointDis[0][0] = 0;             			//B线
	PointDis[1][0] = 480;									//C线
	PointDis[2][0] = 670 + 375;						//E线
	PointDis[3][0] = 670 + 1192.5 - 100;	//H线。木桩子提前测
	PointDis[4][0] = -3100;								//I线

	for (u8 i = 0; i < 3; i++) {
		PointDis[i][1] = PointDis[i][0] + 30;					//截止线
		PointDis[i][2] = PointDis[i][0] + 100;				//认为无砝码
	}
}

int way;				//默认way = 0，在F抓中间砝码

/**
  * @brief  B线，检测是否有砝码0
  * @prarm  无
  * @retval 无
  */
void BLine() {
	puts("");
	puts("----- B Line -----");
	SetGoaldis(0, 14, 30);
	isStore = 'B';					//存储砝码信息
	isStop = 0;
	LED_RED = 1;						//关红灯
	SensorON(0);
	while (Sensor_open && Run_Dis < PointDis[0][2]);
	Sensor_open = 0;				//关闭所有传感器
	dist[0] = 0;
	puts("B OFF 0");
}

/**
  * @brief  决定way的值。way为1时，抓取C和Cy线砝码
  * @prarm  无
  * @retval 无
  */
void CLine() {
	puts("");
	puts("----- C Line -----");
	SetGoaldis(1, 25, 45);
	SetGoaldis(2, 25, 45);
	isStore = 'C';								//存储砝码信息
	isStop = obj[0] ? 0 : 1;			//如果在B测到0，不停车；如果在B没测到0，C线测到就停车
	Con_Dis = 825;
	LED_GREEN = 1;								//开左传感器关绿灯
	LED_RED = 1;									//开右传感器关红灯
	puts("C ON 1 2");
	SensorON(1);
	SensorON(2);
	while (Run_Dis < PointDis[1][2]);
//	delay_ms(100);
	Sensor_open = 0;							//关闭所有传感器
	dist[1] = dist[2] = 0;
	LED_GREEN = 1;								//关灯重置
	LED_RED = 1;									//关灯重置
	puts("C OFF All");
	if (way) {
		puts("-----WAY == 1-----");
		//先向外移动到S1
		if (obj[1])		Stepper_Turn(1, WAI1, S1);
		if (obj[2])		Stepper_Turn(2, WAI2, S1);
		while (MotorState != Stop);				//阻塞等待 车子停稳
		Catch('C', obj[1], 1, obj[2]);		//中线一定会抓
		if(obj[1] && obj[2])							
			delay_ms(500);									//防止撞倒木桩
		if (obj[1] && obj[2]) {						//直接去D
			Run(0, 375, MVEL);
			while(Run_Dis < 1012.5);				//阻塞等待走到Cy时开启超声波，到D关
			E1_Check();
		} else {													//先去Cy再去D
			Run(0, 187.5, MVEL);
			while (MotorState != Stop);			//阻塞等待 车子停稳
			Catch('c', !obj[1], 0, !obj[2]);//中线一定不抓
			Run(0, 187.5, MVEL);						//从Cy线走就开启超声波，到D关
			E1_Check();
		}
		while (MotorState != Stop);				//阻塞等待 车子停稳
		DPoint();
		Motor_Run(0, MVEL);
	} else
		puts("-----WAY == 0-----");
}

/**
  * @brief  将中间砝码放在木桩上
  * @prarm  无
  * @retval 无
  */
void DPoint() {
	puts("");
	puts("----- D Point -----");
	if(way) {
		Sensor_open = 0;			//关闭所有传感器
		LED_GREEN = 1;				//关灯重置
		LED_RED = 1;					//关灯重置
		puts("E1 OFF All");
		dist[1] = dist[2] = 0;
	}
	delay_ms(500);
	MagnetOFF(0);						//关闭电磁铁0
	delay_ms(50);
#if TIMER_ENABLE
	weight[5 - 3] = 0;							//步进5设置为不带负载
#endif
	Stepper_Turn(5, UP0, C1);				//步进0向上提举到一定高度
	while (Stepper_GetStatus(5));		//等待步进电机向上移动完毕
}

/**
  * @brief  way为1时，检测E线砝码
  * @prarm  无
  * @retval 无
  */
void E1_Check() {
	puts("");
	puts("----- E1_Check -----");
	SetGoaldis(1, 0, 6);
	SetGoaldis(2, 0, 6);
	isStore = 'E';				//存储砝码信息
	isStop = 0;						//不停车
	Con_Dis = 0;
	LED_GREEN = 1;				//开左传感器关绿灯
	LED_RED = 1;					//开右传感器关红灯
	SensorON(1);
	SensorON(2);
	puts("E1 ON 1 2");
}

/**
  * @brief  way为0时，识别并抓取E、F线砝码
  * @prarm  无
  * @retval 无
  */
void ELine0() {
	puts("");
	puts("----- E Line -----");
	SetGoaldis(1, 0, 6);
	SetGoaldis(2, 0, 6);
	SetGoaldis(0, 14, 30);
	isStore = 'E';				//存储砝码信息
	isStop = 1;						//测到物品就停车
	Con_Dis = 1387.5;
	LED_GREEN = 1;				//开左传感器关绿灯
	LED_RED = 1;					//开右传感器关红灯
	SensorON(1);
	SensorON(2);
	puts("E0 ON 1 2");
	//阻塞等待直到 已经停车 或者 已经走过E线
	while (Run_Dis < PointDis[2][2]);
//	delay_ms(100);
	if (!obj[3] && !obj[4])
		Con_Stop(1575 - Run_Dis);
	Sensor_open = 0;	//关闭所有传感器
	LED_GREEN = 1;		//关灯重置
	LED_RED = 1;			//关灯重置
	puts("E0 OFF 1 2");
	dist[1] = dist[2] = 0;
	isStore = 'F';				//存储砝码信息
	puts("E0 ON 0");
//	SetGoaldis(1, 25, 45);
//	SetGoaldis(2, 25, 45);
//	SensorON(1);
//	SensorON(2);
	SensorON(0);
	

	if (obj[3] || obj[4]) {
		while (MotorState != Stop);									//阻塞等待 车子停稳
		Sensor_open = 0;														//停车就关掉超声波
		dist[1] = dist[2] = dist[0] = 0;
		LED_RED = 1;
		puts("E0 OFF 0");
//		if(!obj[6] && !obj[3])		obj[3] = 1;				//矫正误测
//		if(!obj[7] && !obj[4])		obj[4] = 1;				//矫正误测
		
		
		if(!obj[3])	Stepper_Turn(1,WAI1,S1);
		if(!obj[4])	Stepper_Turn(2,WAI2,S1);
		Catch('E', obj[3], 0, obj[4]);							//E抓取
		delay_ms(500);
		if (obj[3] && obj[4]) {
			Motor_Run(0, MVEL);
		} else {
			Run(0, 187.5, MVEL);											//去F抓
			while (MotorState != Stop);								//阻塞等待 车子停稳
			Catch('F', !obj[3], obj[5], !obj[4]); 		//F抓取
			//delay_ms(500);														//等步进抬升到一定高度，避免撞倒木桩
			Motor_Run(0, MVEL);
		}
	} else {
		Stepper_Turn(1, WAI1, S1);
		Stepper_Turn(2, WAI2, S1);
		while (Run_Dis <= 1387.5 - 100);						//防止机子太快误测到G点砝码
		Sensor_open = 0;														//如果没有测到也没有停车，会在这里关闭超声波
		dist[1] = dist[2] = dist[0] = 0;
		puts("E0 OFF 0");
		while (MotorState != Stop){putchar('-');}									//阻塞等待 车子停稳
		Catch('F', !obj[3], obj[5], !obj[4]);				//F抓取
		//delay_ms(800);														//等步进抬升到一定高度，避免撞倒木桩
		Motor_Run(0, MVEL);
	}
}

/**
  * @brief  I线的识别和放置
  * @prarm  无
  * @retval 无
  */
void HLine() {
	puts("");
	puts("----- H Line -----");
	/*
	LED_GREEN = 1;		//开左传感器关绿灯
	puts("H ON 1");
	isStore = 0;			//不需要存储信息
	isStop = 1;
	Con_Dis = 2205 + 10;
	SetGoaldis(1, 45, 55);
	SensorON(1);
	while (MotorState == Velocity_Xunji);
	delay_ms(100);
	Sensor_open = 0;
	LED_GREEN = 1;		//关灯重置
	puts("H Close 1");
	*/
	while (MotorState != Stop);	//阻塞等待 车子停稳
	puts("H STOP");
	Run_Dis = 0;			//置零，回程距离为负

	Place_Side();			//放置两侧的砝码

	Back();						//回程

}

/**
  * @brief  两种情况回程的行进和抓取
  * @prarm  无
  * @retval 无
  */
void Back() {
	puts("");
	puts("----- Back -----");
	if (way) {
		obj[3] ? Stepper_Turn(1, NEI1, S2) : Stepper_Turn(1, NEI1, S2 - S1);			//步进1向内到达正确位置
		obj[4] ? Stepper_Turn(2, NEI2, S2) : Stepper_Turn(2, NEI2, S2 - S1);			//步进2向内到达正确位置
		(obj[3] && obj[4]) ? Run(1, 817.5, MVEL) : Run(1, 630, MVEL);							//两个都有去E，否则去F
		delay_ms(500);										//避免抓手下降挂到砝码
		Stepper_Turn(3, DOWN3, C2 - 5);		//步进3向下预先放到抓取高度
		Stepper_Turn(4, DOWN4, C2 - 5);		//步进4向下预先放到抓取高度
		while (MotorState != Stop);
		if (!obj[3] || !obj[4]) {					//先去F抓
			Catch('F', !obj[3], 0, !obj[4]);
			if (!obj[3] && !obj[4])					//F抓了两个，直接去I
				Motor_Run(1, MVEL);
			else {													//还要去E抓
				Run(1, 187.5, MVEL);
				while (MotorState != Stop);		//阻塞等待 车子停稳
			}
		}
		Catch('E', obj[3], 0, obj[4]);
		Motor_Run(1, MVEL);
	} else {
		obj[1] ? Stepper_Turn(1, NEI1, S2 - S1) : Stepper_Turn(1, NEI1, S2);					//步进1向内到达正确位置
		obj[2] ? Stepper_Turn(2, NEI2, S2 - S1) : Stepper_Turn(2, NEI2, S2);					//步进2向内到达正确位置
		if (!obj[5])
			Run(1, 255, MVEL);							//去G抓
		else if (obj[3] && obj[4])
			Run(1, 630, MVEL);							//去F抓
		else
			Run(1, 1005, MVEL);							//去D
		delay_ms(500);										//避免抓手下降挂到砝码
		Stepper_Turn(3, DOWN3, C2 - 5);		//步进3向下预先放到抓取高度
		Stepper_Turn(4, DOWN4, C2 - 5);		//步进4向下预先放到抓取高度
		while (MotorState != Stop){				//阻塞等待 车子停稳
			printf("-");
		}
		if (!obj[5]) {
			Catch('G', 0, 1, 0);						//在G抓取
			Run(1, 750, MVEL);							//去D
			while (MotorState != Stop);			//阻塞等待 车子停稳
		} else if (obj[3] && obj[4]) {
			Catch('F', 0, 1, 0);						//在F抓取
			delay_ms(500);									//防止撞倒木桩
			Run(1, 375, MVEL);							//去D
			while (MotorState != Stop);			//阻塞等待 车子停稳
		}
		puts("before D");
		DPoint();													//在D放置
		puts("after D");
		if (!obj[1] || !obj[2]) {					//先去Cy抓
			Run(1, 187.5, MVEL);
			while (MotorState != Stop);			//阻塞等待 车子停稳
			Catch('c', !obj[1], 0, !obj[2]);//在Cy抓取
			if (!obj[1] && !obj[2])					//Cy抓了两个，直接去I
				Motor_Run(1, MVEL);
			else {													//还要去C抓
				Run(1, 187.5, MVEL);
				while (MotorState != Stop);		//阻塞等待 车子停稳
				Catch('C', obj[1], 0, obj[2]);//在C抓
				Motor_Run(1, MVEL);
			}
		} else {
			Run(1, 375, MVEL);
			while (MotorState != Stop);				//阻塞等待 车子停稳
			Catch('C', obj[1], 0, obj[2]);		//在C抓
			Motor_Run(1, MVEL);
		}
	}
}


/**
  * @brief  I线的识别和放置
  * @prarm  无
  * @retval 无
  */
void ILine() {
	puts("");
	puts("----- I Line -----");
	/*
	puts("I ON 3");
	SetGoaldis(3, 40, 60);
	SensorON(3);
	isStore = 0;									//不存储砝码信息
	isStop = 1;
	Con_Dis = -3510 - 10;
	while (MotorState == Velocity_Xunji);
	delay_ms(200);
	Sensor_open = 0;
	LED_GREEN = 1;								//关灯重置
	dist[3] = 0;
	puts("I Close 3");
	*/
	while (MotorState != Stop);		//阻塞等待 车子停稳
	puts("I STOP");
	Place_Side();
}


/**
  * @brief  抓取砝码
  * @prarm  Line	当前位置
  * @prarm  odd		奇数侧是否抓取
  * @prarm  mid		中间是否抓取
  * @prarm  even	偶数侧是否抓取
  * @retval 无
  */
void Catch(char Line, u8 odd, u8 mid, u8 even) {
	if (!(odd || mid || even))		return;					//三个都不抓
	printf("\r\n----- %c%d Catch -----\r\n", Line, way);
	float angle_wai;
	switch (Line) {
		case 'E':
		case 'c':
			angle_wai = S2;
			break;
		case 'F':
		case 'C':
			angle_wai = S2 - S1;
			break;
		default:
			angle_wai = 0;
			break;
	}

	if (odd)	while (Stepper_GetStatus(1) || Stepper_GetStatus(3));
	if (even)	while (Stepper_GetStatus(2) || Stepper_GetStatus(4));
	if (mid)	while (Stepper_GetStatus(5));

	if (odd)	Stepper_Turn(3, DOWN3, C1);				//步进3向下抓取
	if (even)	Stepper_Turn(4, DOWN4, C1);				//步进4向下抓取
	if (mid)	Stepper_Turn(5, DOWN0, Z0);				//步进5向下抓取

	if (odd)	while (Stepper_GetStatus(3));			//等待步进3向下移动完毕
	if (even)	while (Stepper_GetStatus(4));			//等待步进4向下移动完毕
	if (mid)	while (Stepper_GetStatus(5));			//等待步进5向下移动完毕

	delay_ms(50);
	if (odd)	MagnetON(1);											//打开电磁铁1
	if (even)	MagnetON(2);											//打开电磁铁2
	if (mid)	MagnetON(0);											//打开电磁铁0
	delay_ms(50);

#if TIMER_ENABLE
	if (odd)	weight[3 - 3] = 1;										//步进3设置为带负载
	if (even)	weight[4 - 3] = 1;										//步进4设置为带负载
	if (mid)	weight[5 - 3] = 1;										//步进5设置为带负载
#endif

	if (odd)	Stepper_Turn(3, UP3, C2);					//先上升
	if (even)	Stepper_Turn(4, UP4, C2);					//先上升
	if (mid)	Stepper_Turn(5, UP0, Z0);					//先上升
	delay_ms(100);															//防止拖地
	if (odd)	Stepper_Turn(1, WAI1, angle_wai);	//后向外
	if (even)	Stepper_Turn(2, WAI2, angle_wai);	//后向外
}

void Place_Side() {
	puts("Place");
	//等待步进电机移动完毕
	while (Stepper_GetStatus(1) || Stepper_GetStatus(2) || Stepper_GetStatus(3) || Stepper_GetStatus(4));
#if TIMER_ENABLE
	weight[0] = weight[1] = 0;			//3、4号电机均设置为不带负载
#endif
	delay_ms(50);
	MagnetOFF(1);										//关闭电磁铁1
	MagnetOFF(2);										//关闭电磁铁2
	delay_ms(50);

	Stepper_Turn(3, UP3, C1 - 5);		//步进3向上提举，脱离砝码
	Stepper_Turn(4, UP4, C1 - 5);		//步进4向上提举，脱离砝码
	while (Stepper_GetStatus(3) || Stepper_GetStatus(4));		//等待步进电机向上移动完毕
}
