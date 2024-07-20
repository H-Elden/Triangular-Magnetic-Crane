/*----- 整机运行过程 -----*/

#include "Process.h"

/**
  * @brief  主程序初始化
  * @prarm  无
  * @retval 无
  */
void Init() {
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //中断优先级分组2
	delay_init(); 							//延迟函数初始化
	LED_Init(); 								//LED灯初始化
	KEY_GPIO_Init();						//按键KEY初始化
	Usart1_Init(115200); 					//串口初始化
	Usart2Init(9600);//串口2初始化接传感器端
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
    //WitStartIYAWCali();  陀螺仪z轴置零函数（偶尔用一次）
	delay_ms(1000);//延迟等待初始化完成
	Gyro_read();
	ZhongZhi = fAngle[2];
	delay_ms(10);
	Gyro_read();
	ZhongZhi = fAngle[2];
	printf("ZhongZhi = %.3f\r\n", ZhongZhi);
    n_Fudu = 1;
	
    way = 0;
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
    usart3_SendCmd(cmd, 3);                                // 向步进驱动板发送命令
    LED_RED = 0;                                                    // 红灯亮起，检测是否电机上电
    while (rxFrameFlag == false);                    // 如果没有上电会一直循环等待，红灯常亮
    rxFrameFlag = false;                                    // 清除接收标志
    LED_RED = 1;                                                    // 红灯熄灭，说明步进上电了
}

/**
  * @brief  点位距离初始化
  * @prarm  无
  * @retval 无
  */
float PointDis[5][3];
void PointDis_Init() {
    PointDis[0][0] = 0;             //B线
	PointDis[1][0] = 375;															//C线
	PointDis[2][0] = 670 + 375;						//E线
	PointDis[3][0] = 670 + 1192.5 - 20;		//H线。木桩子提前测
	PointDis[4][0] = -3000;														//I线

	for (u8 i = 0; i < 3; i++) {
		PointDis[i][1] = PointDis[i][0] + 30;					//截止线
		PointDis[i][2] = PointDis[i][0] + 100;				//认为无砝码
	}
}

int way;
void BLine() {
    puts("");
	puts("----- B Line -----");
    SetGoaldis(0, 14, 23);
    isStore = 'B';					//存储砝码信息
    isStop = 0;
    SensorON(0);
	while (Sensor_open && Run_Dis < PointDis[0][2]);
    Sensor_open = 0;				//关闭所有传感器
    dist[0] = 0;
}

void CLine() {
	puts("");
	puts("----- C Line -----");
	SetGoaldis(1, 25, 45);
	SetGoaldis(2, 25, 45);
	isStore = 'C';					//存储砝码信息
	isStop = 1;
    Con_Dis = 825;
	LED_GREEN = 1;		//开左传感器关绿灯
	LED_RED = 1;			//开右传感器关红灯
	puts("C ON 1 2");
	SensorON(1);
	SensorON(2);
	while (Sensor_open && Run_Dis < PointDis[1][2]);
    if(!obj[1] && !obj[2])
        isStop = 0;
	Sensor_open = 0;				//关闭所有传感器
	dist[1] = dist[2] = 0;
	delay_ms(100);
	LED_GREEN = 1;		//关灯重置
	LED_RED = 1;			//关灯重置
	puts("C OFF All");
    if(isStop != 0)
    {
        puts("-----WAY == 1-----");
        	while (MotorState != Stop);	//阻塞等待 车子停稳
            obj[1] ? /*Stepper_Turn(3, DOWN3, C1)*/ 1: Stepper_Turn(1, WAI1, S1);			//步进3向下抓取  或  步进1向外
            obj[2] ? Stepper_Turn(4, DOWN4, C1) : Stepper_Turn(2, WAI2, S1);			//步进4向下抓取  或  步进2向外
            Stepper_Turn(5, DOWN0, Z0);
            
            while (/*Stepper_GetStatus(3)*/ Stepper_GetStatus(4) || Stepper_GetStatus(5));	//等待步进电机向下移动完毕
            delay_ms(50);
            if (obj[1])		MagnetON(1);								//开启电磁铁1
            if (obj[2])		MagnetON(2);								//开启电磁铁2
            MagnetON(0);//开启电磁铁0
            
            delay_ms(50);
//            if (obj[1])	Stepper_Turn(3, UP3, C2);				//步进3向上提取
            if (obj[2])	Stepper_Turn(4, UP4, C2);				//步进4向上提取
            Stepper_Turn(5, UP0, Z0);//步进0向上
            delay_ms(50);
            if (obj[1])	Stepper_Turn(1, WAI1, S2);   		//步进1向外
            if (obj[2])	Stepper_Turn(2, WAI2, S2);   //步进2向外
            
            if (obj[1] && obj[2]) {
                Run(0, 375, MVEL);
                DPoint2();
            }
            else
            {
                Run(0, 187.5, MVEL);
                Cyline();
            }
    }
}

void Cyline() {
    puts("");
	puts("----- Cy Line -----");
    
    while (MotorState != Stop);	//阻塞等待 车子停稳
    
    if (!obj[1])	while (Stepper_GetStatus(1));			//等待步进电机1横向移动完毕
	if (!obj[2])	while (Stepper_GetStatus(2));			//等待步进电机2横向移动完毕
    
//    if (!obj[1])	Stepper_Turn(3, DOWN3, C1);			//步进3向下抓取
	if (!obj[2])	Stepper_Turn(4, DOWN4, C1);			//步进4向下抓取
    
//    if (!obj[1])	while (Stepper_GetStatus(3));			//等待步进电机3向下移动完毕
	if (!obj[2])	while (Stepper_GetStatus(4));			//等待步进电机4向下移动完毕
    
    delay_ms(50);
	if (!obj[1])		MagnetON(1);								//开启电磁铁1
	if (!obj[2])		MagnetON(2);								//开启电磁铁2
	delay_ms(50);
    
//    if (!obj[1])	Stepper_Turn(3, UP3, C2);				//步进3向上提取
    if (!obj[2])	Stepper_Turn(4, UP4, C2);				//步进4向上提取
    delay_ms(50);
    if (!obj[1])	Stepper_Turn(1, WAI1, S2 - S1);   		//步进1向外
    if (!obj[2])	Stepper_Turn(2, WAI2, S2 - S1);   //步进2向外


	Run(0, 187.5, MVEL);
    DPoint2();
}

void ELine1() {
    puts("-----WAY == 0-----");
	puts("");
	puts("----- E1 Line -----");
	SetGoaldis(1, 0, 6);
	SetGoaldis(2, 0, 6);
//    SetGoaldis(0, 0, 22);
	isStore = 'E';				//存储砝码信息
	isStop = 1;						//测到物品就停车
	Con_Dis = 1387.5;
	LED_GREEN = 1;		//开左传感器关绿灯
	LED_RED = 1;			//开右传感器关红灯
	SensorON(1);
	SensorON(2);
//    SensorON(0);
	puts("E1 ON 1 2");
	//阻塞等待直到 已经停车 或者 已经走过E线
	while (MotorState == Velocity_Xunji && Run_Dis < PointDis[2][2]);
    Sensor_open = 0;	//关闭所有传感器
    SetGoaldis(0, 14, 23);
    isStop = 0;
    SensorON(0);
    if(!obj[3] && !obj[4])
        Con_Stop(1575 - Run_Dis);
	delay_ms(50);
	LED_GREEN = 1;		//关灯重置
	LED_RED = 1;			//关灯重置
	puts("E1 OFF All");
	dist[1] = dist[2] = 0;
	if (obj[3] || obj[4]) {
		while (MotorState != Stop);	//阻塞等待 车子停稳
		obj[3] ? /*Stepper_Turn(3, DOWN3, C1)*/1 : Stepper_Turn(1, WAI1, S1);			//步进3向下抓取  或  步进1向外
		obj[4] ? Stepper_Turn(4, DOWN4, C1) : Stepper_Turn(2, WAI2, S1);			//步进4向下抓取  或  步进2向外

		while (Stepper_GetStatus(4));	//等待步进电机向下移动完毕
		delay_ms(50);
		if (obj[3])		MagnetON(1);								//开启电磁铁1
		if (obj[4])		MagnetON(2);								//开启电磁铁2
		delay_ms(50);

//		if (obj[3])	Stepper_Turn(3, UP3, C2);				//步进3向上提取
		if (obj[4])	Stepper_Turn(4, UP4, C2);				//步进4向上提取
		delay_ms(50);
		if (obj[3])	Stepper_Turn(1, WAI1, S2);   		//步进1向外
		if (obj[4])	Stepper_Turn(2, WAI2, S2);   //步进2向外

		if (obj[3] && obj[4]) {
			Motor_Run(0, MVEL);
		} else {
			Run(0, 200, MVEL);											//正向行进200mm
			FLine1();
		}
	} else {
		Stepper_Turn(1, WAI1, S1);
		Stepper_Turn(2, WAI2, S1);
		FLine1();
	}
}

void ELine21() {
    puts("");
	puts("----- E21 Line -----");
    SetGoaldis(1, 0, 6);
	SetGoaldis(2, 0, 6);
    isStore = 'E';				//存储砝码信息
	isStop = 0;						//不停车
	Con_Dis = 0;
	LED_GREEN = 1;		//开左传感器关绿灯
	LED_RED = 1;			//开右传感器关红灯
	SensorON(1);
	SensorON(2);
	puts("E21 ON 1 2");
    while(Run_Dis < PointDis[2][2]);
    delay_ms(100);
	Sensor_open = 0;	//关闭所有传感器
	LED_GREEN = 1;		//关灯重置
	LED_RED = 1;			//关灯重置
	puts("E21 OFF All");
	dist[1] = dist[2] = 0;
}

void ELine22() {
    puts("");
	puts("----- E22 Line -----");
    Stepper_Turn(1, NEI1, S2);
    Stepper_Turn(2, NEI2, S2);
    delay_ms(50);
//    Stepper_Turn(3, DOWN3, C2);
    Stepper_Turn(4, DOWN4, C2);
    while ( Stepper_GetStatus(4));				//等待步进电机移动完毕
	while (Stepper_GetStatus(1) || Stepper_GetStatus(2));				
    
    while(MotorState != Stop);
    
//    Stepper_Turn(3, DOWN3, C1);
    Stepper_Turn(4, DOWN4, C1);
    while ( Stepper_GetStatus(4));				//等待步进电机移动完毕
    delay_ms(50);
    MagnetON(1);
    MagnetON(2);
    delay_ms(50);
    
    Stepper_Turn(1, WAI1, S2);
    Stepper_Turn(2, WAI2, S2);
//    Stepper_Turn(3, UP3, C2);
    Stepper_Turn(4, UP4, C2);
    Motor_Run(1, MVEL);
    
}
void ELine221() {
    puts("");
	puts("----- E221 Line -----");
    while(MotorState != Stop);
//    if(obj[3])     Stepper_Turn(3, DOWN3, C1);
    if(obj[4])     Stepper_Turn(4, DOWN4, C1);
    while ( Stepper_GetStatus(4));				//等待步进电机移动完毕
    if(obj[3])      MagnetON(1);
    if(obj[4])      MagnetON(2);
    delay_ms(50);
    if(obj[3]){
//        Stepper_Turn(3, UP3, C2);
        Stepper_Turn(1, WAI1, S2);
    }
    if(obj[4]){
        Stepper_Turn(4, UP4, C2);
        Stepper_Turn(2, WAI2, S2);
    }
    Motor_Run(1, MVEL);
}

void FLine1() {
	puts("");
	puts("----- F1 Line -----");
    while(Run_Dis <= 1378.5);
    Sensor_open = 0;//关闭0号超声波
    dist[0] = 0;
	while (MotorState != Stop);	//阻塞等待 车子停稳

    if (!obj[3])	while (Stepper_GetStatus(1));			//等待步进电机1横向移动完毕
	if (!obj[4])	while (Stepper_GetStatus(2));			//等待步进电机2横向移动完毕
    
//    if (!obj[3])	Stepper_Turn(3, DOWN3, C1);			//步进3向下抓取
	if (!obj[4])	Stepper_Turn(4, DOWN4, C1);			//步进4向下抓取
    if ( obj[5])    Stepper_Turn(5, DOWN0, Z0);         //步进0向下抓取

//    if (!obj[3])	while (Stepper_GetStatus(3));			//等待步进电机3向下移动完毕
	if (!obj[4])	while (Stepper_GetStatus(4));			//等待步进电机4向下移动完毕
    if ( obj[5])	while (Stepper_GetStatus(5));			//等待步进电机0向下移动完毕
    
	delay_ms(50);
	if (!obj[3])		MagnetON(1);								//开启电磁铁1
	if (!obj[4])		MagnetON(2);								//开启电磁铁2
    if ( obj[5])		MagnetON(0);								//开启电磁铁2
	delay_ms(50);

//    if (!obj[3])	Stepper_Turn(3, UP3, C2);				//步进3向上提取
    if (!obj[4])	Stepper_Turn(4, UP4, C2);				//步进4向上提取
    if ( obj[5])    Stepper_Turn(5, UP0, Z0);         //步进0向上提取
    delay_ms(50);
    if (!obj[3])	Stepper_Turn(1, WAI1, S2 - S1);   		//步进1向外
    if (!obj[4])	Stepper_Turn(2, WAI2, S2 - S1);   //步进2向外


	Motor_Run(0, MVEL);
}

void FLine2() {
    puts("");
	puts("----- F2 Line -----");
    obj[3] ? Stepper_Turn(1, NEI1, S2) : Stepper_Turn(1, NEI1, S2 - S1);			//步进向内移动
	obj[4] ? Stepper_Turn(2, NEI2, S2) : Stepper_Turn(2, NEI2, S2 - S1);			
    delay_ms(50);
//    Stepper_Turn(3, DOWN3, C2);
    Stepper_Turn(4, DOWN4, C2);
    
    while ( Stepper_GetStatus(4));				//等待步进电机移动完毕
	while (Stepper_GetStatus(1) || Stepper_GetStatus(2));				
    
    while(MotorState != Stop);
    
//    if(!obj[3])
//            Stepper_Turn(3, DOWN3, C1);
    if(!obj[4])
            Stepper_Turn(4, DOWN4, C1);
    while (Stepper_GetStatus(4));				//等待步进电机移动完毕
    if(!obj[3])
        MagnetON(1);
    if(!obj[4])
        MagnetON(2);
    delay_ms(50);
    
    if(!obj[3]){
//            Stepper_Turn(3, UP3, C2);
            Stepper_Turn(1, WAI1, S2 - S1);
    }
    if(!obj[4]){
            Stepper_Turn(4, UP4, C2);
            Stepper_Turn(2, WAI2, S2 - S1);
    }
    if(!obj[3] && !obj[4])
        Motor_Run(1, MVEL);
    else{
        Run(1, 187.5, MVEL);
        ELine221();
    }
}

void HLine1() {
    while(Run_Dis <= 1378.5);
    Sensor_open = 0;//关闭0号超声波
    dist[0] = 0;
	puts("");
	puts("----- H1 Line -----");
	LED_GREEN = 1;		//开左传感器关绿灯
	puts("H1 ON 1");
	isStore = 0;//不需要存储信息
	isStop = 1;
	Con_Dis = 2205;
	SetGoaldis(1, 45, 55);
	SensorON(1);
	while (MotorState == Velocity_Xunji);
	delay_ms(200);
	Sensor_open = 0;
	LED_GREEN = 1;		//关灯重置
	LED_RED = 1;			//关灯重置
	puts("H1 Close All");
	while (MotorState != Stop);	//阻塞等待 车子停稳
	Run_Dis = 0;
	while ( Stepper_GetStatus(4));				//等待步进电机移动完毕
	while (Stepper_GetStatus(1) || Stepper_GetStatus(2));				

	delay_ms(50);
	MagnetOFF(1);								//关闭电磁铁1
	MagnetOFF(2);								//关闭电磁铁2
	delay_ms(50);

//	Stepper_Turn(3, UP3, C1);			//步进3向上提举，脱离砝码
	Stepper_Turn(4, UP4, C1);			//步进4向上提举，脱离砝码
	while ( Stepper_GetStatus(4));		//等待步进电机向上移动完毕
	
    obj[1] ? Stepper_Turn(1,NEI1,S2 - S1) : Stepper_Turn(1,NEI1,S2);					//步进1向内到达正确位置
	obj[2] ? Stepper_Turn(2,NEI2,S2 - S1) : Stepper_Turn(2,NEI2,S2);					//步进2向内到达正确位置
    delay_ms(50);
//	Stepper_Turn(3,DOWN3,C2);				//步进3向下预先放到抓取高度
	Stepper_Turn(4,DOWN4,C2);				//步进4向下预先放到抓取高度
    
    if (obj[5])
    {
		Run(1,1005, MVEL);
        DPoint1();
    }
	else
    {
		Run(1, 255, MVEL);
        GLine1();
    }
}

void HLine2() {
    puts("");
	puts("----- H2 Line -----");
	LED_GREEN = 1;		//开左传感器关绿灯
	puts("H2 ON 1");
	isStore = 0;//不需要存储信息
	isStop = 1;
	Con_Dis = 2205;
	SetGoaldis(1, 45, 55);
	SensorON(1);
	while (MotorState == Velocity_Xunji);
	delay_ms(200);
	Sensor_open = 0;
	LED_GREEN = 1;		//关灯重置
	LED_RED = 1;			//关灯重置
	puts("H2 Close All");
	while (MotorState != Stop);	//阻塞等待 车子停稳
	Run_Dis = 0;
	while (Stepper_GetStatus(4));				//等待步进电机移动完毕
	while (Stepper_GetStatus(1) || Stepper_GetStatus(2));				

	delay_ms(50);
	MagnetOFF(1);								//关闭电磁铁1
	MagnetOFF(2);								//关闭电磁铁2
	delay_ms(50);

//	Stepper_Turn(3, UP3, C1);			//步进3向上提举，脱离砝码
	Stepper_Turn(4, UP4, C1);			//步进4向上提举，脱离砝码
	while (Stepper_GetStatus(4));		//等待步进电机向上移动完毕
    
    if(!obj[3] || !obj[4])
    {
        Run(1, 630, MVEL);
        FLine2();
    }
    else
    {
        Run(1, 817.5, MVEL);
        ELine22();
    }
}

void GLine1() {
	puts("");
	puts("----- G1 Line -----");
    while (MotorState != Stop);
	Stepper_Turn(5,DOWN0,Z0);					//步进0向下抓取
	while(Stepper_GetStatus(5));					//等待步进电机向下移动完毕
	delay_ms(50);
	MagnetON(0);										//开启电磁铁0
	delay_ms(50);

	Stepper_Turn(5,UP0,Z0);					//步进0向上提举到一定高度
    delay_ms(50);
	Run(1, 750, MVEL);
    DPoint1();
}

void DPoint1() {
	puts("");
	puts("----- D1 Point -----");
    while (MotorState != Stop);
	delay_ms(50);
	MagnetOFF(0);								//关闭电磁铁0
	delay_ms(50);
	Stepper_Turn(5,UP0,C1);					//步进0向上提举到一定高度
	while(Stepper_GetStatus(5));					//等待步进电机向上移动完毕

	if (!obj[1] || !obj[2]) {
		Run(1, 188, MVEL);
		while (MotorState != Stop);
		CLine1();
	} else {
		Run(1, 375, MVEL);
		while (MotorState != Stop);
		BLine1();
	}
}

void DPoint2() {
    puts("");
	puts("----- D2 Point -----");
    while (MotorState != Stop);	//阻塞等待 车子停稳
	delay_ms(50);
	MagnetOFF(0);								//关闭电磁铁0
	delay_ms(50);
	Stepper_Turn(5,UP0,C1);					//步进0向上提举到一定高度
	while(Stepper_GetStatus(5));					//等待步进电机向上移动完毕
    Motor_Run(0,MVEL);
}

void CLine1() {
	puts("");
	puts("----- C1 Line -----");

//    if(!obj[1]) Stepper_Turn(3, DOWN3, C1);			//步进3向下抓取 
    if(!obj[2]) Stepper_Turn(4, DOWN4, C1);			//步进4向下抓取 

    while (Stepper_GetStatus(4));	//等待步进电机向下移动完毕
    delay_ms(50);
    if (!obj[1])		MagnetON(1);								//开启电磁铁1
    if (!obj[2])		MagnetON(2);								//开启电磁铁2
    delay_ms(50);

//    if (!obj[1])	Stepper_Turn(3, UP3, C2);				//步进3向上提取
    if (!obj[2])	Stepper_Turn(4, UP4, C2);				//步进4向上提取
    delay_ms(50);
    if (!obj[1])	Stepper_Turn(1, WAI1, S2);   		//步进1向外
    if (!obj[2])	Stepper_Turn(2, WAI2, S2);   //步进2向外

	if (!obj[1] && !obj[2])
		Motor_Run(1, MVEL);
	else {
		Run(1, 187.5, MVEL);
		while (MotorState != Stop);
		BLine1();
	}
}

void BLine1() {
	puts("");
	puts("----- B1 Line -----");

	
//	if(obj[1])	Stepper_Turn(3,DOWN3,C1);				//步进3向下抓取
	if(obj[2])	Stepper_Turn(4,DOWN4,C1);				//步进4向下抓取

	while(Stepper_GetStatus(4));		//等待步进电机向下移动完毕
	delay_ms(50);
	if(obj[1])		MagnetON(1);								//开启电磁铁1
	if(obj[2])		MagnetON(2);								//开启电磁铁2
	delay_ms(50);

//    if (obj[1])	Stepper_Turn(3, UP3, C2);				//步进3向上提取
    if (obj[2])	Stepper_Turn(4, UP4, C2);				//步进4向上提取
    delay_ms(50);
    if (obj[1])	Stepper_Turn(1, WAI1, S2-S1);   		//步进1向外
    if (obj[2])	Stepper_Turn(2, WAI2, S2-S1);   //步进2向外
    
	Motor_Run(1, MVEL);
}

void ILine() {
	puts("");
	puts("----- I Line -----");
	puts("I ON 3");
	SetGoaldis(3, 40, 60);
	SensorON(3);
	isStore = 0;									//不存储砝码信息
	isStop = 1;
    Con_Dis = -3510;
	while (MotorState == Velocity_Xunji);
	delay_ms(200);
	Sensor_open = 0;
	LED_GREEN = 1;		//关灯重置
	LED_RED = 1;			//关灯重置
	puts("I Close All");

	while (MotorState != Stop);	//阻塞等待 车子停稳
    while(Stepper_GetStatus(1)||Stepper_GetStatus(2));		//等待步进电机移动完毕
    while(Stepper_GetStatus(4));		
    
	delay_ms(50);
	MagnetOFF(1);								//关闭电磁铁1
	MagnetOFF(2);								//关闭电磁铁2
	delay_ms(50);
//	Stepper_Turn(3,UP3,C1);			//步进3向上提举，脱离砝码
	Stepper_Turn(4,UP4,C1);			//步进4向上提举，脱离砝码

	while(Stepper_GetStatus(4));				//等待步进电机向上移动完毕
}
