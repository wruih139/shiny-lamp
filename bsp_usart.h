#ifndef __BSP_USART_H
#define __BSP_USART_H

#include "AllHeader.h"

#define DEBUG_USARTx Serial

void USART1_init(unsigned long baudrate);
void USART2_init(unsigned long baudrate);

void USART1_Send_U8(uint8_t ch);
void USART1_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length);

void USART2_Send_U8(uint8_t ch);
void USART2_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length);

#endif
