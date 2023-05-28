/*
 * UART.h
 *
 *  Created on: May 28, 2023
 *      Author: David
 */

#ifndef INC_UART_H_
#define INC_UART_H_
#include "stm32f411xe.h"
#include <stdint.h>

void UART2_Init(void);
void UART2_Recieve(uint8_t*pData,uint16_t Size);
void UART2_DMA_Transmit(uint8_t*pData,uint16_t Size);

#endif
