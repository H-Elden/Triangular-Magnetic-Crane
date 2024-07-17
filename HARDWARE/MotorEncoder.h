#ifndef __MOTORENCODER_H
#define __MOTORENCODER_H
#include "sys.h"
#include "L298N.h"
#include "HC-SR04.h"
#include "gyroscope.h"

#define PI 3.1415926
#define Radius 32.3//单位毫米
#define MVEL 800			//行进电机速度

typedef enum {
	Stop,								//停止
	Velocity_Xunji,                 //mpu6050或ccd走直PID
	VelCir,							//串级PID，速度环+位置环
} MotorState_t;

extern int LEncoder, REncoder; //当前速度、当前位置
extern int LTargetVelocity, RTargetVelocity, LCurrentPosition, RCurrentPosition, LCurrentPosition_V, RCurrentPosition_V, Encoder, LPWM, RPWM; //目标速度、目标圈数、编码器读数、PWM 控制变量
extern float LTargetCircle, RTargetCircle;
extern float Velcity_Kp, Velcity_Ki, Velcity_Kd; //相关速度 PID 参数
extern float Position_Kp, Position_Ki, Position_Kd; //相关位置 PID 参数
extern int LStartMinV, RStartMinV; //初始最小速度为目标速度的十分之一
extern MotorState_t MotorState;
extern float ZhongZhi;
extern int n_Fudu;

void MotorEncoder_Init_TIM2(void);
void MotorEncoder_Init_TIM4(void);
void TIM6_Init(void);

int LRead_Encoder(void);
int RRead_Encoder(void);
int LPosition_FeedbackControl(float Cirle, int CurrentVelocity);
int RPosition_FeedbackControl(float Cirle, int CurrentVelocity);
int LVelocity_FeedbackControl(int TargetVelocity, int CurrentVelocity);
int RVelocity_FeedbackControl(int TargetVelocity, int CurrentVelocity);
int Velocity_Restrict(int PWM_P, int TargetVelocity);
int Turn(float YAW);
float Myabs(float a);

//要增加的接口
extern float Run_Dis;			//正数表示正向行进总距离(A点为0)，负数表示反向行进总距离(H点为0)。单位mm
extern float Con_Dis;

/**
  * @brief  起重机从停车起步，后保持一定速度前进
  * @prarm  dir 0表示正向行进，1表示负向行进
	* @prarm  vel 表示速度
  * @retval 无
  */
void Motor_Run(uint8_t dir, uint16_t vel);

/**
  * @brief  起重机继续行进一定距离后停车
  * @prarm  dis 继续行进的距离，单位mm
  * @retval 无
	* @note		这段距离包含减速的过程，在dis内准确停车
  */
void Con_Stop(float dis);//此函数使用后需要将使用Clear（）函数

/**
  * @brief  起重机忙走行进一定距离后停车
  * @prarm  dis 继续行进的距离，单位mm
  * @retval 无
	* @note		这段距离包含加减速的过程，在dis内准确停车
  */
void Run(uint8_t dir, float dis, uint16_t vel); //此函数使用后需要将使用Clear（）函数

void Clear(void);//起重机完成一个位置环指令后，即起重机通过位置环停稳后，应使用此函数将位置环积分清0

#endif
