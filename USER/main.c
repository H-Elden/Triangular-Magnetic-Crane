#include "Process.h"

/**

  * @brief  主程序初始化
  * @prarm  无
  * @retval int
  */
int main()
{
	u8 only[5] = {0};
    u8 only_way = 0;
	Init();
	LED_GREEN = 0;									//绿灯亮起，表示初始化完成，开始待机
	while(KEY_Scan() != KEY_ON);		//阻塞等待按下按钮 KEY0
	LED_GREEN = 1;									//绿灯熄灭，结束待机，开始运行程序
    
	puts("-----BEGIN-----");
//    SensorON(1);
//    SensorON(2);
//    SensorON(3);
//	Stepper_Turn(3,UP3,C1);
	Stepper_Turn(4,UP4,C1);
	Stepper_Turn(5,UP0,Z0); 
    
	delay_ms(500);
	Motor_Run(0, MVEL);							//以800的速度正向行进
	while(1)
	{
//        printf("dis %d = %.3f   dis %d = %.3f\r\n", 1, dist[1],2,dist[2]);
		//B线：识别
		if(only[0] == 0 && Run_Dis >= PointDis[0][0] && Run_Dis <= PointDis[0][1]){
			BLine();
			only[0] = 1;
		}
		//C线：识别 + 抓取或不抓取
		else if(only[1] == 0 && Run_Dis >= PointDis[1][0] && Run_Dis <= PointDis[1][1]){
			CLine(); 
			only[1] = 1;
		}
		//E线：吸取
		else if(way == 0 && only[2] == 0 && Run_Dis >= PointDis[2][0]){
			ELine1();
			only[2] = 1;
		}
        else if(way == 0 && Run_Dis >= 1387.5 && only_way == 0){
			SensorOFF(0);
            dist[0] = 0;
            puts("sensor 0 close");
            only_way = 1;
		}
        else if(way == 1 && only[2] == 0 && Run_Dis >= PointDis[2][0]){
			ELine21();
			only[2] = 1;
		}//H线：放置
        else if(way == 0 && only[3] == 0 && Run_Dis >= PointDis[3][0]){
			HLine1();
			only[3] = 1;
		}
        else if(way == 1 && only[3] == 0 && Run_Dis >= PointDis[3][0]){
			HLine2();
			only[3] = 1;
		}
		//I线：放置
		else if(only[4] == 0 && Run_Dis <= PointDis[4][0]){
			ILine();
			only[4] = 1;
			puts("-----END-----");
		}
		delay_ms(10);
	}
}

