#ifndef __USART3_H
#define __USART3_H

#include "sys.h"
#include "fifo.h"

extern __IO bool rxFrameFlag;
extern __IO uint8_t rxCmd[FIFO_SIZE];
extern __IO uint8_t rxCount;

void Usart3_Init(void);

void usart3_SendCmd(__IO uint8_t *cmd, uint8_t len);
void usart3_SendByte(uint8_t data);

#endif
