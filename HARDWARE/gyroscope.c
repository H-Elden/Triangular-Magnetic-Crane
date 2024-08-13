#include "gyroscope.h"

/*
@ link : http://wit-motion.cn

@ Function:
1. Power on automatic detection sensor
2. Read acceleration, angular velocity, angle and magnetic field data
3. Set switching baud rate parameters

USB-TTL                   STM32Core              		JY901s
VCC          -----           VCC        ----        	 VCC
TX           -----           RX1  (GPIOA_10)   
RX           -----           TX1  (GPIOA_9)
GND          -----           GND    ----       			 GND
                             RX2  (GPIOA_3)  ----        TX
							 TX2  (GPIOA_2)  ----        RX
------------------------------------
*/


float fAcc[3], fGyro[3], fAngle[3]; //定义float数组为下面计算输出做准备
int i;

static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;
const uint32_t c_uiBaud[10] = {0, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};

void Gyro_read(void)
{
		if(s_cDataUpdate)//在注册获取传感器数据回调函数会对变量进行赋值
		{
			for(i = 0; i < 3; i++)//for循环待i不符合条件时跳出for循环
			{
				fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;//算法公式
				fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;//算法公式
				fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;//算法公式
			}
			if(s_cDataUpdate & ACC_UPDATE)//判断不为0就执行下面的语句
			{
//				printf("acc:%.3f %.3f %.3f\r\n", fAcc[0], fAcc[1], fAcc[2]);//打印对应的数据出来
				s_cDataUpdate &= ~ACC_UPDATE;//s_cDataUpdate和~ACC_UPDATE与运算后赋值给s_cDataUpdate
			}
			if(s_cDataUpdate & GYRO_UPDATE)//以下几行代码同上
			{
//				printf("gyro:%.3f %.3f %.3f\r\n", fGyro[0], fGyro[1], fGyro[2]);
				s_cDataUpdate &= ~GYRO_UPDATE;
			}
			if(s_cDataUpdate & ANGLE_UPDATE)
			{
//				printf("angle:%.3f\r\n",fAngle[2]);
				s_cDataUpdate &= ~ANGLE_UPDATE;
			}
			if(s_cDataUpdate & MAG_UPDATE)
			{
				//printf("mag:%d %d %d\r\n", sReg[HX], sReg[HY], sReg[HZ]);
				s_cDataUpdate &= ~MAG_UPDATE;
			}
		}
}

void SensorUartSend(uint8_t *p_data, uint32_t uiSize)//传感器串口发送
{
	Uart2Send(p_data, uiSize);//串口2发送
}

void Delayms(uint16_t ucMs)//延时程序
{
	delay_ms(ucMs);
}

void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)//传感器数据升级
{
	int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)//判断uiReg的数据是什么来进行选择对应的操作
        {
//            case AX:
//            case AY:
            case AZ:
				s_cDataUpdate |= ACC_UPDATE;//s_cDataUpdate变量和ACC_UPDATE变量或运算后的结果再把结果赋值给s_cDataUpdate，如s_cDataUpdate=s_cDataUpdate|和ACC_UPDATE
            break;
//            case GX:
//            case GY:
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
            break;
//            case HX:
//            case HY:
            case HZ:
				s_cDataUpdate |= MAG_UPDATE;
            break;
//            case Roll:
//            case Pitch:
            case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
            break;
            default:
				s_cDataUpdate |= READ_UPDATE;
			break;
        }
		uiReg++;
    }
}

void AutoScanSensor(void)			//串口波特率检测
{
	int i, iRetry;
	
	for(i = 1; i < 10; i++)			//for循环待i不符合条件会跳出循环
	{
		Usart2Init(c_uiBaud[i]);//串口2波特率从小到大查询
		iRetry = 2;
		do//do-while()循环语句先执行再判断
		{
			s_cDataUpdate = 0;
			WitReadReg(AX, 3);		//给函数进行导入形参
			delay_ms(100);				//延时100ms
			if(s_cDataUpdate != 0)//判断s_cDataUpdate不等于0如果是不等于0就执行括号里面的内容
			{
				//printf("%d baud find sensor\r\n\r\n", c_uiBaud[i]);//打印找到传感器和打印对应的波特率
				//ShowHelp();				//执行函数打印内容
				return ;
			}
			iRetry--;//变量自减
		}while(iRetry);//while不为0就一直执行循环的内容
	}
	//printf("can not find sensor\r\n");//如果上面没有找到传感器就会执行下面这两句，如果找到就不会执行这两句
	//printf("please check your connection\r\n");//
}

