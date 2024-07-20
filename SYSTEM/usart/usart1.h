#ifndef __USART_H
#define __USART_H
#include "stdio.h"
#include "sys.h"
#include "gyroscope.h"

#define USART_REC_LEN  			200  	//定义最大接收字节数 200

extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符
extern u16 USART_RX_STA;         		//接收状态标记
//如果想串口中断接收，请不要注释以下宏定义
void Usart1_Init(u32 bound);
void usart1_send(u8 data);
void Usart1_SendByte( USART_TypeDef * pUSARTx, uint8_t ch);
#endif


