/*
 * i2c.h
 *
 *  Created on: May 21, 2023
 *      Author: David
 */

#ifndef INC_I2C_H_
#define INC_I2C_H_

#include "stm32f411xe.h"
#include "main.h"
#include <stdint.h>

void I2C1_Init(void);
void I2C1_Start(void);
void I2C_Master_Transmit(uint8_t address,uint8_t *pData,uint16_t Size);
void I2C_Master_Recieve(uint8_t address,uint8_t *pData,uint16_t Size);
void I2C_Address(uint8_t address);
void I2C_Stop(void);
void I2C_Mem_Read(uint8_t address,uint16_t Address,uint16_t Mem_Size,uint8_t *pData,uint16_t Size);




#endif /* I2C_H_ */

