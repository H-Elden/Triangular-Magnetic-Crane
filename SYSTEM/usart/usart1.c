#include "sys.h"
#include "usart1.h"

//加入以下代码,支持printf函数,而不需要选择use MicroLIB
#pragma import(__use_no_semihosting)
//标准库需要的支持函数
struct __FILE {
	int handle;
};

FILE __stdout;
//定义_sys_exit()以避免使用半主机模式
void _sys_exit(int x) {
	x = x;
}

//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA = 0;     //接收状态标记

void Usart1_Init(u32 bound) {
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟

	//USART1_TX   GPIOA.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9

	//USART1_RX	  GPIOA.10初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10

	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0 ; //抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

	//USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

	USART_Init(USART1, &USART_InitStructure); //初始化串口1
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
	USART_Cmd(USART1, ENABLE);                    //使能串口1

}

void usart1_send(u8 data) {
	USART1->DR = data;
	while ((USART1->SR & 0x40) == 0);
}
/***************** 发送一个字节 **********************/
void Usart1_SendByte( USART_TypeDef * pUSARTx, uint8_t ch) {
	/* 发送一个字节数据到USART */
	USART_SendData(pUSARTx, ch);
	/* 等待发送数据寄存器为空 */
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);

}
int fputc(int ch, FILE *file) {
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	USART_SendData(USART1, ch);
	return ch;
}

void USART1_IRQHandler(void) { //USART1(串口1)中断
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) { //判断获取IT状态是否为0
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);//把UART1串口1中的数据清零
	}
	USART_ClearITPendingBit(USART2, USART_IT_ORE); //清零
}

