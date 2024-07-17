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
	uart_init(115200); 					//串口初始化
	Usart2Init(9600);//串口2初始化接传感器端
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
	Stepper_Init_TIM5();				//初始化步进电机
	Switch_Init();							//初始化电磁继电器

}

/**
  * @brief  点位距离初始化
  * @prarm  无
  * @retval 无
  */
float PointDis[5][3];
void PointDis_Init() {
    PointDis[0][0] = 107.5;             //B线
	PointDis[1][0] = 107.5 + 375;															//C线
	PointDis[2][0] = 670 + 375;						//E线
	PointDis[3][0] = 670 + 1192.5 - 20;		//H线。木桩子提前测
	PointDis[4][0] = -3000;														//I线

	for (u8 i = 0; i < 2; i++) {
		PointDis[i][1] = PointDis[i][0] + 30;					//截止线
		PointDis[i][2] = PointDis[i][0] + 100;				//认为无砝码
	}
}

int way;
void BLine() {
    puts("");
	puts("----- B Line -----");
    SetGoaldis(0, 0, 24);
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
	SetGoaldis(1, 30, 40);
	SetGoaldis(2, 30, 40);
	isStore = 'C';					//存储砝码信息
	isStop = 1;
    Con_Dis = 825;
	LED_GREEN = 1;		//开左传感器关绿灯
	LED_RED = 1;			//开右传感器关红灯
	puts("C1 ON 1 2");
	SensorON(1);
	SensorON(2);
	while (Sensor_open && Run_Dis < PointDis[1][2]);
	Sensor_open = 0;				//关闭所有传感器
	dist[1] = dist[2] = 0;
	delay_ms(100);
	LED_GREEN = 1;		//关灯重置
	LED_RED = 1;			//关灯重置
	puts("C1 OFF All");
    if(isStop == 1)
    {
    if(obj[1] || obj[2])
        {
        	while (MotorState != Stop);	//阻塞等待 车子停稳
            puts("Stepper");
            obj[1] ? Stepper_Turn(3, DOWN3, C1) : Stepper_Turn(1, WAI1, S1);			//步进3向下抓取  或  步进1向外
            obj[2] ? Stepper_Turn(4, DOWN4, C1) : Stepper_Turn(2, WAI2, S1);			//步进4向下抓取  或  步进2向外
            Stepper_Turn(0, DOWN0, Z0);
            
            while (StartStepper[3] || StartStepper[4] || StartStepper[0]);	//等待步进电机向下移动完毕
            delay_ms(50);
            if (obj[1])		MagnetON(1);								//开启电磁铁1
            if (obj[2])		MagnetON(2);								//开启电磁铁2
            MagnetON(0);
            
            delay_ms(50);
            if (obj[1])	Stepper_Turn(3, UP3, C2);				//步进3向上提取
            if (obj[2])	Stepper_Turn(4, UP4, C2);				//步进4向上提取
            delay_ms(50);
            if (obj[1])	Stepper_Turn(1, WAI1, S2);   		//步进1向外
            if (obj[2])	Stepper_Turn(2, WAI2, S2);   //步进2向外
            Stepper_Turn(0, UP0, Z0);
            
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
}

void Cyline() {
    
    while (MotorState != Stop);	//阻塞等待 车子停稳
    
    if (!obj[1])	while (StartStepper[1]);			//等待步进电机1横向移动完毕
	if (!obj[2])	while (StartStepper[2]);			//等待步进电机2横向移动完毕
    
    if (!obj[1])	Stepper_Turn(3, DOWN3, C1);			//步进3向下抓取
	if (!obj[2])	Stepper_Turn(4, DOWN4, C1);			//步进4向下抓取
    
    if (!obj[1])	while (StartStepper[3]);			//等待步进电机3向下移动完毕
	if (!obj[2])	while (StartStepper[4]);			//等待步进电机4向下移动完毕
    
    delay_ms(50);
	if (!obj[1])		MagnetON(1);								//开启电磁铁1
	if (!obj[2])		MagnetON(2);								//开启电磁铁2
	delay_ms(50);
    
    if (!obj[1])	Stepper_Turn(3, UP3, C2);				//步进3向上提取
    if (!obj[1])	Stepper_Turn(4, UP4, C2);				//步进4向上提取
    delay_ms(50);
    if (!obj[1])	Stepper_Turn(1, WAI1, S2 - S1);   		//步进1向外
    if (!obj[1])	Stepper_Turn(2, WAI2, S2 - S1);   //步进2向外


	Run(0, 187.5, MVEL);
    DPoint2();
}

void ELine1() {
	puts("");
	puts("----- E Line -----");
	SetGoaldis(1, 2, 4);
	SetGoaldis(2, 2, 4);			//测量修正：7.4-16:36
    SetGoaldis(0, 0, 21);
	isStore = 'E';				//存储砝码信息
	isStop = 1;						//测到物品就停车
	Con_Dis = 1387.5;
	LED_GREEN = 1;		//开左传感器关绿灯
	LED_RED = 1;			//开右传感器关红灯
	SensorON(1);
	SensorON(2);
    SensorON(0);
	puts("E ON 1 2");
	//阻塞等待直到 已经停车 或者 已经走过E线
	while (MotorState == Velocity_Xunji && Run_Dis < PointDis[2][2]);
	delay_ms(100);
	Sensor_open = 0;	//关闭所有传感器
	LED_GREEN = 1;		//关灯重置
	LED_RED = 1;			//关灯重置
	puts("E OFF All");
	dist[0] = dist[1] = dist[2] = 0;
	if (obj[3] || obj[4]) {
		while (MotorState != Stop);	//阻塞等待 车子停稳
		puts("Stepper");
		obj[3] ? Stepper_Turn(3, DOWN3, C1) : Stepper_Turn(1, WAI1, S1);			//步进3向下抓取  或  步进1向外
		obj[4] ? Stepper_Turn(4, DOWN4, C1) : Stepper_Turn(2, WAI2, S1);			//步进4向下抓取  或  步进2向外

		while (StartStepper[3] || StartStepper[4]);	//等待步进电机向下移动完毕
		delay_ms(50);
		if (obj[3])		MagnetON(1);								//开启电磁铁1
		if (obj[4])		MagnetON(2);								//开启电磁铁2
		delay_ms(50);

		if (obj[3])	Stepper_Turn(3, UP3, C2);				//步进3向上提取
		if (obj[4])	Stepper_Turn(4, UP4, C2);				//步进4向上提取
		delay_ms(1000);
		if (obj[3])	Stepper_Turn(1, WAI1, S2);   		//步进1向外
		if (obj[4])	Stepper_Turn(2, WAI2, S2);   //步进2向外

		if (obj[3] && obj[4]) {
			Motor_Run(0, MVEL);
			puts("E Motor_Run");
		} else {
			Run(0, 200, MVEL);											//正向行进200mm
			FLine1();
		}
	} else {
		Stepper_Turn(1, WAI1, S1);
		Stepper_Turn(2, WAI2, S1);
        Run(0, 200, MVEL);
		FLine1();
	}
}

void ELine21() {
    SetGoaldis(1, 2, 4);
	SetGoaldis(2, 2, 4);			//测量修正：7.4-16:36
    isStore = 'E';				//存储砝码信息
	isStop = 0;						//测到物品就停车
	Con_Dis = 0;
	LED_GREEN = 1;		//开左传感器关绿灯
	LED_RED = 1;			//开右传感器关红灯
	SensorON(1);
	SensorON(2);
	puts("E2 ON 1 2");
    while(Run_Dis < PointDis[2][2]);
    delay_ms(100);
	Sensor_open = 0;	//关闭所有传感器
	LED_GREEN = 1;		//关灯重置
	LED_RED = 1;			//关灯重置
	puts("E2 OFF All");
	dist[1] = dist[2] = 0;
}

void ELine22() {
    
    Stepper_Turn(1, NEI1, S2);
    Stepper_Turn(2, NEI2, S2);
    delay_ms(50);
    Stepper_Turn(3, DOWN3, C2);
    Stepper_Turn(4, DOWN4, C2);
    while (StartStepper[3] || StartStepper[4]);				//等待步进电机横向移动完毕
	while (StartStepper[1] || StartStepper[2]);				//等待步进电机横向移动完毕
    
    while(MotorState != Stop);
    
    Stepper_Turn(3, DOWN3, C1);
    Stepper_Turn(4, DOWN4, C1);
    while (StartStepper[3] || StartStepper[4]);				//等待步进电机横向移动完毕
    delay_ms(50);
    MagnetON(1);
    MagnetON(2);
    delay_ms(50);
    
    Stepper_Turn(1, WAI1, S2);
    Stepper_Turn(2, WAI2, S2);
    Stepper_Turn(3, UP3, C2);
    Stepper_Turn(4, UP4, C2);
    Motor_Run(1, MVEL);
    
}
void ELine221() {
    while(MotorState != Stop);
    if(obj[3])     Stepper_Turn(3, DOWN3, C1);
    if(obj[4])     Stepper_Turn(4, DOWN4, C1);
    while (StartStepper[3] || StartStepper[4]);				//等待步进电机横向移动完毕
    if(obj[3])      MagnetON(1);
    if(obj[4])      MagnetON(2);
    delay_ms(50);
    if(obj[3]){
        Stepper_Turn(3, UP3, C2);
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
	puts("----- F Line -----");
	while (MotorState != Stop);	//阻塞等待 车子停稳

    if (!obj[3])	while (StartStepper[1]);			//等待步进电机1横向移动完毕
	if (!obj[4])	while (StartStepper[2]);			//等待步进电机2横向移动完毕
    
    if (!obj[3])	Stepper_Turn(3, DOWN3, C1);			//步进3向下抓取
	if (!obj[4])	Stepper_Turn(4, DOWN4, C1);			//步进4向下抓取
    if ( obj[5])    Stepper_Turn(0, DOWN0, Z0);         //步进0向下抓取

    if (!obj[3])	while (StartStepper[3]);			//等待步进电机3向下移动完毕
	if (!obj[4])	while (StartStepper[4]);			//等待步进电机4向下移动完毕
    if ( obj[5])	while (StartStepper[0]);			//等待步进电机0向下移动完毕
    
	delay_ms(50);
	if (!obj[3])		MagnetON(1);								//开启电磁铁1
	if (!obj[4])		MagnetON(2);								//开启电磁铁2
    if ( obj[5])		MagnetON(0);								//开启电磁铁2
	delay_ms(50);

    if (!obj[3])	Stepper_Turn(3, UP3, C2);				//步进3向上提取
    if (!obj[4])	Stepper_Turn(4, UP4, C2);				//步进4向上提取
    if ( obj[5])    Stepper_Turn(0, UP0, Z0);         //步进0向上提取
    delay_ms(1000);
    if (!obj[3])	Stepper_Turn(1, WAI1, S2 - S1);   		//步进1向外
    if (!obj[4])	Stepper_Turn(2, WAI2, S2 - S1);   //步进2向外
    delay_ms(1000);


	Motor_Run(0, MVEL);
	puts("F Motor_Run");
}

void FLine2() {
    
    obj[3] ? Stepper_Turn(1, NEI1, S2) : Stepper_Turn(1, NEI1, S2 - S1);			//步进3向下抓取  或  步进1向外
	obj[4] ? Stepper_Turn(2, NEI2, S2) : Stepper_Turn(2, NEI2, S2 - S1);			//步进4向下抓取  或  步进2向外
    delay_ms(50);
    Stepper_Turn(3, DOWN3, C2);
    Stepper_Turn(4, DOWN4, C2);
    
    while (StartStepper[3] || StartStepper[4]);				//等待步进电机横向移动完毕
	while (StartStepper[1] || StartStepper[2]);				//等待步进电机横向移动完毕
    
    while(MotorState != Stop);
    
    if(!obj[3])
            Stepper_Turn(3, DOWN3, C1);
    if(!obj[4])
            Stepper_Turn(4, DOWN4, C1);
    while (StartStepper[3] || StartStepper[4]);				//等待步进电机横向移动完毕
    if(!obj[3])
        MagnetON(1);
    if(!obj[4])
        MagnetON(2);
    delay_ms(50);
    
    if(!obj[3]){
            Stepper_Turn(3, UP3, C2);
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
	puts("");
	puts("----- H Line -----");
	LED_GREEN = 1;		//开左传感器关绿灯
	puts("H ON 1");
	isStore = 'F';
	isStop = 1;
	Con_Dis = 2205;
	SetGoaldis(1, 45, 55);
	SensorON(1);
	while (MotorState == Velocity_Xunji);
	delay_ms(200);
	Sensor_open = 0;
	LED_GREEN = 1;		//关灯重置
	LED_RED = 1;			//关灯重置
	puts("H Close All");
	while (MotorState != Stop);	//阻塞等待 车子停稳
	Run_Dis = 0;
	while (StartStepper[3] || StartStepper[4]);				//等待步进电机横向移动完毕
	while (StartStepper[1] || StartStepper[2]);				//等待步进电机横向移动完毕

	delay_ms(50);
	MagnetOFF(1);								//关闭电磁铁1
	MagnetOFF(2);								//关闭电磁铁2
	delay_ms(50);

	Stepper_Turn(3, UP3, C1);			//步进3向上提举，脱离砝码
	Stepper_Turn(4, UP4, C1);			//步进4向上提举，脱离砝码
	while (StartStepper[3] || StartStepper[4]);		//等待步进电机向上移动完毕
	
    /////
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
	puts("H Motor_Run");
	delay_ms(500);
	Stepper_Turn(3,DOWN3,C2);				//步进3向下预先放到抓取高度
	Stepper_Turn(4,DOWN4,C2);				//步进4向下预先放到抓取高度
	obj[1] ? Stepper_Turn(1,NEI1,S2 - S1) : Stepper_Turn(1,NEI1,S2);					//步进1向内到达正确位置
	obj[2] ? Stepper_Turn(2,NEI2,S2 - S1) : Stepper_Turn(2,NEI2,S2);					//步进2向内到达正确位置
}

void HLine2() {
    
    puts("");
	puts("----- H2 Line -----");
	LED_GREEN = 1;		//开左传感器关绿灯
	puts("H ON 1");
	isStore = 'F';
	isStop = 1;
	Con_Dis = 2205;
	SetGoaldis(1, 45, 55);
	SensorON(1);
	while (MotorState == Velocity_Xunji);
	delay_ms(200);
	Sensor_open = 0;
	LED_GREEN = 1;		//关灯重置
	LED_RED = 1;			//关灯重置
	puts("H Close All");
	while (MotorState != Stop);	//阻塞等待 车子停稳
	Run_Dis = 0;
	while (StartStepper[3] || StartStepper[4]);				//等待步进电机横向移动完毕
	while (StartStepper[1] || StartStepper[2]);				//等待步进电机横向移动完毕

	delay_ms(50);
	MagnetOFF(1);								//关闭电磁铁1
	MagnetOFF(2);								//关闭电磁铁2
	delay_ms(50);

	Stepper_Turn(3, UP3, C1);			//步进3向上提举，脱离砝码
	Stepper_Turn(4, UP4, C1);			//步进4向上提举，脱离砝码
	while (StartStepper[3] || StartStepper[4]);		//等待步进电机向上移动完毕
    
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
	puts("----- G Line -----");
	/////
    puts("Stepper");
    while (MotorState != Stop);
	Stepper_Turn(0,DOWN0,Z0);					//步进0向下抓取
	while(StartStepper[0]);					//等待步进电机向下移动完毕
	delay_ms(50);
	MagnetON(0);										//开启电磁铁0
	delay_ms(50);

	Stepper_Turn(0,UP0,Z0);					//步进0向上提举到一定高度
	//while(StartStepper[0]);					//等待步进电机向上移动完毕
    delay_ms(50);
	Run(1, 750, MVEL);
	puts("G Motor_Run");
    DPoint1();
}

void DPoint1() {
	puts("");
	puts("----- D Point -----");
    while (MotorState != Stop);
	delay_ms(50);
	MagnetOFF(0);								//关闭电磁铁0
	delay_ms(50);
    puts("Stepper");
	Stepper_Turn(0,UP0,C1);					//步进0向上提举到一定高度
	while(StartStepper[0]);					//等待步进电机向上移动完毕

	puts("D Motor_Run");
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
    
    while (MotorState != Stop);	//阻塞等待 车子停稳
	delay_ms(50);
	MagnetOFF(0);								//关闭电磁铁0
	delay_ms(50);
    puts("Stepper");
	Stepper_Turn(0,UP0,C1);					//步进0向上提举到一定高度
	while(StartStepper[0]);					//等待步进电机向上移动完毕
    Motor_Run(0,MVEL);
}

void CLine1() {
	puts("");
	puts("----- C Line -----");

    puts("Stepper");
    if(!obj[1]) Stepper_Turn(3, DOWN3, C1);			//步进3向下抓取 
    if(!obj[2]) Stepper_Turn(4, DOWN4, C1);			//步进4向下抓取 

    while (StartStepper[3] || StartStepper[4]);	//等待步进电机向下移动完毕
    delay_ms(50);
    if (!obj[1])		MagnetON(1);								//开启电磁铁1
    if (!obj[2])		MagnetON(2);								//开启电磁铁2
    delay_ms(50);

    if (!obj[1])	Stepper_Turn(3, UP3, C2);				//步进3向上提取
    if (!obj[2])	Stepper_Turn(4, UP4, C2);				//步进4向上提取
    delay_ms(50);
    if (!obj[1])	Stepper_Turn(1, WAI1, S2);   		//步进1向外
    if (!obj[2])	Stepper_Turn(2, WAI2, S2);   //步进2向外

	puts("C Motor_Run");
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
	puts("----- B Line -----");

	
	if(obj[1])	Stepper_Turn(3,DOWN3,C1);				//步进3向下抓取
	if(obj[2])	Stepper_Turn(4,DOWN4,C1);				//步进4向下抓取

	while(StartStepper[3] || StartStepper[4]);		//等待步进电机向下移动完毕
	delay_ms(50);
	if(obj[1])		MagnetON(1);								//开启电磁铁1
	if(obj[2])		MagnetON(2);								//开启电磁铁2
	delay_ms(50);

    if (obj[1])	Stepper_Turn(3, UP3, C2);				//步进3向上提取
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
    while(StartStepper[1]||StartStepper[2]);		//等待步进电机移动完毕
    while(StartStepper[3]||StartStepper[4]);		
	puts("Stepper");
	delay_ms(50);
	MagnetOFF(1);								//关闭电磁铁1
	MagnetOFF(2);								//关闭电磁铁2
	delay_ms(50);
	Stepper_Turn(3,UP3,C1);			//步进3向上提举，脱离砝码
	Stepper_Turn(4,UP4,C1);			//步进4向上提举，脱离砝码

	while(StartStepper[3]||StartStepper[4]);		//等待步进电机向上移动完毕
}
