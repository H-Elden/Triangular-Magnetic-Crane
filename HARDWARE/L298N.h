#ifndef __L298N_H 
#define __L298N_H 
#include "sys.h"

//紫线外 蓝线内              场地正向为负
//左高低引脚为pb13 pb12
//左信号引脚为pb1
//左编码器b相pb6 a相pb7

//绿线外 黄线内              场地正向为正
//右高低引脚为pb15 pb14
//右信号引脚为pC9
//右编码器b相pa0 a相pa1

void Motor_Init(void);
void LL298N_Init(int arr, int psc);
void RL298N_Init(int arr, int psc);
void SetPWM(int moterLeft,int moterRight);

#endif //定义完毕，或者引用过头文件到达这一步
