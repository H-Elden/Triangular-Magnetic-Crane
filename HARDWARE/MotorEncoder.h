#ifndef __MOTORENCODER_H
#define __MOTORENCODER_H
#include "sys.h"
#include "L298N.h"
#include "HC-SR04.h"
#include "gyroscope.h"

#define PI 3.1415926
#define Radius 32.3					//单位毫米
#define MVEL 2000						//行进电机速度
#define Accel 1500.0
#define Dccel 800.0

typedef enum {
	Stop,								//停止
	Velocity_Xunji,     //使用陀螺仪，确保走直
	VelCir,							//串级PID，速度环+位置环
} MotorState_t;

extern int LEncoder, REncoder; //当前速度、当前位置
extern int LTargetVelocity, RTargetVelocity, LCurrentPosition, RCurrentPosition, Encoder, LPWM, RPWM; //目标速度、目标圈数、编码器读数、PWM 控制变量
extern float LTargetCircle, RTargetCircle;
extern float Velcity_Kp, Velcity_Ki, Velcity_Kd; //相关速度 PID 参数
extern float Position_Kp, Position_Ki, Position_Kd; //相关位置 PID 参数
extern MotorState_t MotorState;
extern float ZhongZhi;

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

//要增加的接口
extern float Run_Dis;			//正数表示正向行进总距离(A点为0)，负数表示反向行进总距离(H点为0)。单位mm
extern float Con_Dis;

void Motor_Run(uint8_t dir, uint16_t vel);
void Con_Stop(float dis);
void Run(uint8_t dir, float dis, uint16_t vel);

#endif
