#include "usart3.h"

__IO bool rxFrameFlag = false;
__IO uint8_t rxCmd[FIFO_SIZE] = {0};
__IO uint8_t rxCount = 0;

/**
	* @brief   初始化USART3
	* @param   无
	* @retval  无
	*/
void Usart3_Init(void) {
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_Init(&NVIC_InitStructure);

	// 使能GPIOB、AFIO外设时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	// 使能USART3外设时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	/* 初始化USART3引脚 */
	// PB10 - USART3_TX
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;				/* 复用推挽输出 */
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	// PB11 - USART3_RX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;					/* 浮空输入 */
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* 初始化USART3 */
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART3, &USART_InitStructure);

	/* 清除USART3中断 */
	USART3->SR;
	USART3->DR;
	USART_ClearITPendingBit(USART3, USART_IT_RXNE);

	/* 使能USART3中断 */
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);

	/* 使能USART3 */
	USART_Cmd(USART3, ENABLE);
}


/**
	* @brief   USART3中断函数
	* @param   无
	* @retval  无
	*/
void USART3_IRQHandler(void) {
	__IO uint16_t i = 0;

	/* 串口接收中断 */
	if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
		// 未完成一帧数据接收，数据进入缓冲队列
		fifo_enQueue((uint8_t)USART3->DR);

		// 清除串口接收中断
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
	}
	/* 串口空闲中断 */
	else if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET) {
		// 先读SR再读DR，清除IDLE中断
		USART3->SR;
		USART3->DR;

		// 提取一帧数据命令
		rxCount = fifo_queueLength();
		for (i = 0; i < rxCount; i++) {
			rxCmd[i] = fifo_deQueue();
		}

		// 一帧数据接收完成，置位帧标志位
		rxFrameFlag = true;
	}
}

/**
	* @brief   USART发送多个字节
	* @param   无
	* @retval  无
	*/
void usart3_SendCmd(__IO uint8_t *cmd, uint8_t len) {
	__IO uint8_t i = 0;

	for (i = 0; i < len; i++) {
		usart3_SendByte(cmd[i]);
	}
}

/**
	* @brief   USART发送一个字节
	* @param   无
	* @retval  无
	*/
void usart3_SendByte(uint8_t data) {
	__IO uint16_t t0 = 0;

	USART3->DR = data;

	while (!(USART3->SR & USART_FLAG_TXE)) {
		++t0;
		if (t0 > 8000)	{
			return;
		}
	}
}


