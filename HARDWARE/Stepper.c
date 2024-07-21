#include "stepper.h"

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
}

/**
  * @brief    读取电机状态标志位
  * @param    addr：电机地址
  * @retval   1：正在旋转；0：旋转到位；2：错误
  */
uint8_t Stepper_GetStatus(uint8_t addr) {
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
}

