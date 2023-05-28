/*
 * i2c.c
 *
 *  Created on: May 21, 2023
 *      Author:  Malefetsane Lenka
 *      I2C in Blocking mode,
 *      Target device; STM32F411RE
 *
 */
#include "i2c.h"

/**
 * @brief Initialises I2C1 on STM32F411RE, on PB8 and PB9
 *        With Clock speed of 16MHz, 10KHz I2C Speed
 */
void I2C1_Init(void){
	//Enable Clock for GPIOB
	RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

	// Alternate Function Mode for PB8 and PB9
	GPIOB -> MODER |= (0xA <<16); // 10, AF

	//PULL UP

	//GPIOB -> PUPDR |= (0x55<<16);

	// OUTPUT OPEN DRAIN MODE FOR PB8  and PB9
	GPIOB -> OTYPER |= (0x0003<<8); //Set bit and 9 high

	//Set the Pins to HIGH Speed Mode

	GPIOB -> OSPEEDR |= (0xA<<16); //11 high Speed Mode

	//I2C1 is Connected to AF4
	//We will need Alternate Function High Register for PB8 and PB9
	GPIOB -> AFR[1] |= 0x0044;  //0b0100 , AF4

	//ENABLE CLOCK FOR I2C1
	RCC -> APB1ENR |= RCC_APB1ENR_I2C1EN;

	// Reset I2C
	I2C1 -> CR1 |= I2C_CR1_SWRST;
	// Unreset it
	I2C1 -> CR1 &= ~(I2C_CR1_SWRST);
	// Peripheral Clock Frequency
	I2C1 -> CR2 |= 0x10; //	16MHz

    //  Configure the clock control registers
	// T_high = CCR * T_PCLK1
	// T_PCLK1 = 1/8MHz=125ns
	// Standard Mode; 100kHZ
	// 50% duty cycle; T_high = T/2
	// T=1/100kHz = 100us
	// T_high = 5us
	// CCR = 5us/125ns = 40 => 0x21h
	I2C1 -> CCR |= 0x320;


	// Configure the rise time register
	// TRISE = tr/T_PCLK+1 = 1000ns/125ns + 1 = 9
	I2C1 -> TRISE = 0x11;

	//Enable the Peripheral
	//I2C1 -> CR1 |= I2C_CR1_NOSTRETCH;
	I2C1 -> CR1 |= I2C_CR1_PE;

}

/**
 * @brief Generates Start Condition
 *        
 */
void I2C1_Start(void){
	volatile uint16_t temp;
	I2C1 -> CR1 |= I2C_CR1_START; //Set Start Condition
	while(!(I2C1->SR1 & I2C_SR1_SB));  //Wait Until  Start Bit is Generated

	temp = I2C1 -> SR1;  // Read Status Register to Clear SB BIT


}
void I2C_Address(uint8_t address){
	volatile uint16_t temp;
	I2C1-> CR1 |= I2C_CR1_ACK;  // Aknowledge
	I2C1-> DR = address ;   //Either from Read/Write function its already left shifted
	while (!(I2C1->SR1 & I2C_SR1_ADDR));  //Successful Address sent
	temp =  I2C1->SR1 | I2C1-> SR2;


}
void I2C_Master_Transmit(uint8_t address,uint8_t *pData,uint16_t Size){
	uint16_t rem = Size;
	I2C1_Start();  //Start Condition

	I2C_Address(address<<1); //Write Condition
	while (rem){
		while (!(I2C1->SR1 & I2C_SR1_TXE)); //Wait for Data Register to be empty
		
		I2C1 -> DR = pData[Size-rem];       //Write to Data Rgister
		
		rem--;



	}
	//while (!(I2C1->SR1 & I2C_SR1_BTF)); //wait until BTF is set
	I2C_Stop();




}
void I2C_Master_Recieve(uint8_t address,uint8_t *pData,uint16_t Size){
	volatile uint16_t temp;
	I2C1_Start();  //Start Condition

	uint16_t rem = Size;
	if (Size==1){
		I2C1-> DR = (address<<1 | 0x01); //Sent address with 1 to indicate r


		while (!(I2C1->SR1 & I2C_SR1_ADDR));  //Successful Address sent

		I2C1 -> CR1 &= ~(I2C_CR1_ACK);  //Sent NACK

		temp = I2C1-> SR1; //Read SR1 to clear ADDR bit
		temp = I2C1-> SR2;  //Read SR2 to clear ADDR bit

		I2C_Stop();

		while (!(I2C1->SR1 & I2C_SR1_RXNE)); //Wait until Recive buffer is not empty

		pData[Size-rem] = I2C1->DR; // Load data to recieving array


	}
	else {
		I2C_Address(address <<1 | 0x01);

		while (rem > 3){
			HAL_Delay(100);
			while (!(I2C1->SR1 & I2C_SR1_RXNE)); //wait until RXNE is set
			
			pData[Size-rem] = I2C1->DR; // Load data to recieving array
			I2C1-> CR1 |= I2C_CR1_ACK;  // Aknowledge
			rem--;


		}
		while (!(I2C1->SR1 & I2C_SR1_BTF)); //wait until BTF is set
		I2C1 -> CR1 &= ~(I2C_CR1_ACK);  // Sent a NACK

		pData[Size-rem] = I2C1->DR; // Load data to recieving array
		rem--;
		while (!(I2C1->SR1 & I2C_SR1_BTF)); //wait until BTF is set
		I2C_Stop();

		pData[Size-rem] = I2C1->DR; // Load data to recieving array

		rem--;
		// Read Last Byte

		pData[Size-rem] = I2C1->DR; // Load data to recieving array


	}

}
void I2C_Stop(void){
	I2C1 -> CR1 |= I2C_CR1_STOP;


}
/**
  * @brief  Reads from specified Memory Address for I2C Device using Repeated Start
  *         Memory size atmost 2 bytes but can be extended easily to 4 bytes
  * @param  address : 7bit address of I2C Device
  * @param  Address:  Memory address to read from on a device
  * @param  Mem_Size: Size of memory address
  * @param  pData:     pointer to Data buffer
  * @param  Size:    Number of bytes to read from Device
  * @retval None
  */
void I2C_Mem_Read(uint8_t address,uint16_t Address,uint16_t Mem_Size,uint8_t *pData,uint16_t Size){
	if (Mem_Size==1){
		uint8_t temp = (uint8_t) Address;
		pData[0] = temp;
		I2C1_Start(); 
		I2C_Address(address<<1); //Write Condition
		while (!(I2C1->SR1 & I2C_SR1_TXE)); //wait until RXNE is set
		I2C1 -> DR = pData[0];
		
	}
	else{
			pData[0] = (uint8_t) (Address >>8);
			pData[1] = (uint8_t) (Address&0xFF);
			uint16_t rem = Mem_Size;
			I2C1_Start();
			I2C_Address(address<<1); //Write Condition
			while (rem){
				while (!(I2C1->SR1 & I2C_SR1_TXE)); 
				I2C1 -> DR = pData[Size-rem];
				rem--;
			}

	}
	I2C_Master_Recieve(address, pData, Size);

}








