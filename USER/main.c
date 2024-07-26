#include "Process.h"

/**
  * @brief  主程序初始化
  * @prarm  无
  * @retval int
  */
int main() {
	u8 only[5] = {0};
	Init();
	LED_GREEN = 0;									//绿灯亮起，表示初始化完成，开始待机
	while (KEY_Scan() != KEY_ON);		//阻塞等待按下按钮 KEY0
	LED_GREEN = 1;									//绿灯熄灭，结束待机，开始运行程序
	puts("-----BEGIN-----");
	u32 Start = timer;							//记下程序开始的时间
	Stepper_Turn(3, UP3, C1);
	Stepper_Turn(4, UP4, C1);
	Stepper_Turn(5, UP0, Z0);

	delay_ms(500);
	Motor_Run(0, MVEL);							//以MVEL的速度正向行进

	while (1) {
		
		//B线：识别
		if (only[0] == 0 && Run_Dis >= PointDis[0][0] && Run_Dis <= PointDis[0][1]) {
			only[0] = 1;
			BLine();
		}
		//C线：识别 + 抓取或不抓取
		else if (only[1] == 0 && Run_Dis >= PointDis[1][0] && Run_Dis <= PointDis[1][1]) {
			only[1] = 1;
			CLine();
		}
		//E线：吸取
		else if (only[2] == 0 && way == 0 && Run_Dis >= PointDis[2][0] && Run_Dis <= PointDis[2][1]) {
			only[2] = 1;
			ELine0();
		}
		//H线：放置
		else if (only[3] == 0 && Run_Dis >= PointDis[3][0]) {
			only[3] = 1;
			HLine();
		}
		//I线：放置
		else if (only[4] == 0 && Run_Dis <= PointDis[4][0]) {
			only[4] = 1;
			ILine();
			puts("-----END-----");
			printf("总用时：\r\n\t%.1fs\r\n",(timer - Start)/10.0);
		}
		delay_ms(10);
	}
}

