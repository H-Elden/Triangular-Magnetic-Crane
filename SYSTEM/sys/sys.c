#include "sys.h"

//THUMB指令不支持汇编内联
//采用如下方法实现执行汇编指令WFI  
void WFI_SET(void)
{
	__ASM volatile("wfi");		  
}
//关闭所有中断
void INTX_DISABLE(void)
{		  
	__ASM volatile("cpsid i");
}
//开启所有中断
void INTX_ENABLE(void)
{
	__ASM volatile("cpsie i");		  
}
//设置栈顶地址
//addr:栈顶地址
__asm void MSR_MSP(u32 addr) 
{
    MSR MSP, r0 			//set Main Stack value
    BX r14
}

//系统时钟初始化函数
//pll:选择的倍频数，从2开始，最大值为16		 
//void Stm32_Clock_Init(u8 PLL)
//{
//	unsigned char temp=0;   
//	MYRCC_DeInit();		  //复位并配置向量表
// 	RCC->CR|=0x00010000;  //外部高速时钟使能HSEON
//	while(!(RCC->CR>>17));//等待外部时钟就绪
//	RCC->CFGR=0X00000400; //APB1=DIV2;APB2=DIV1;AHB=DIV1;
//	PLL-=2;//抵消2个单位
//	RCC->CFGR|=PLL<<18;   //设置PLL值 2~16
//	RCC->CFGR|=1<<16;	  //PLLSRC ON 
//	FLASH->ACR|=0x32;	  //FLASH 2个延时周期

//	RCC->CR|=0x01000000;  //PLLON
//	while(!(RCC->CR>>25));//等待PLL锁定
//	RCC->CFGR|=0x00000002;//PLL作为系统时钟	 
//	while(temp!=0x02)     //等待PLL作为系统时钟设置成功
//	{   
//		temp=RCC->CFGR>>2;
//		temp&=0x03;
//	}    
//}	
void JTAG_Set(u8 mode)
{
	u32 temp;
	temp=mode;
	temp<<=25;
	RCC->APB2ENR|=1<<0;     //开启辅助时钟	   
	AFIO->MAPR&=0XF8FFFFFF; //清除MAPR的[26:24]
	AFIO->MAPR|=temp;       //设置jtag模式
} 
