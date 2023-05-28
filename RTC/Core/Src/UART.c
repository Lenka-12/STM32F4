#include "UART.h"


void UART2_Init(void){
    RCC ->AHB1ENR|= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_DMA1EN; //Enable clock for GPIOA and DMA1 
    RCC -> APB1ENR |= RCC_APB1ENR_USART2EN; //Enable Clock for USART2

    GPIOA -> MODER &=~(GPIO_MODER_MODER2 |GPIO_MODER_MODER3);  //Reset PA3 and PA2
    GPIOA -> MODER |=  GPIO_MODER_MODER2_1|GPIO_MODER_MODER3_1;  //AF Mode PA3 and PA2
    GPIOA -> OTYPER &= ~(GPIO_OTYPER_OT2 | GPIO_OTYPER_OT3); //Output Push Pull
    GPIOA -> OSPEEDR |= GPIO_OSPEEDER_OSPEEDR2_0 | GPIO_OSPEEDER_OSPEEDR3_0; //Medium Speed
    GPIOA ->AFR[0]   |= (0x0077 << 8);  //AFRL , AF7 for USART2


    USART2 -> CR1 |= USART_CR1_UE; //Enable USART2
    USART2 -> CR1 &= ~(USART_CR1_M); //8 data bits, 1 Start Bit
    USART2 -> CR2 &= ~(USART_CR2_STOP_1 | USART_CR2_STOP_0);  //1 Stop Bit
    USART2 -> CR3 |= USART_CR3_DMAT;  //Enable DMA Transfer
    USART2 -> BRR = 0x0683;  // 9600 Baud Rate, 16MHz clock speed, Over sampling by 16


}
void UART2_DMA_Transmit(uint8_t*pData,uint16_t Size){
    /**
     * 1. Write the USART_DR register address in the DMA control register to configure it as the
     *destination of the transfer. The data will be moved to this address from memory after
     *each TXE event.
     * 2. Write the memory address in the DMA control register to configure it as the source of
     *the transfer. The data will be loaded into the USART_DR register from this memory
     *area after each TXE event.
     *3. Configure the total number of bytes to be transferred to the DMA control register.
     *4. Configure the channel priority in the DMA register
     *5. Configure DMA interrupt generation after half/ full transfer as required by the
     *application.
     *6. Clear the TC bit in the SR register by writing 0 to it.
     *7. Activate the channel in the DMA register
     */
    DMA1_Stream6 -> PAR = (uint32_t) &(USART2 -> DR);  /*1*/
    DMA1_Stream6 ->  M0AR = (uint32_t)pData; /*2*/
    DMA1_Stream6 -> NDTR = Size; /*3*/
    /**
     * Circular Mode, Channel 4 Selection, Memory to Peripheral
     * Memory Increment
    */
    DMA1_Stream6  -> CR |= (0x04<<25) | DMA_SxCR_MINC| DMA_SxCR_DIR_0|DMA_SxCR_TCIE;
    USART2 -> CR1 |= USART_CR1_TE;

    DMA1_Stream6 -> CR |= DMA_SxCR_EN; //Enable DMA
    while(!(DMA1->HISR & DMA_HISR_TCIF6));
    DMA1-> HIFCR |= DMA_HIFCR_CTCIF6;
     DMA1_Stream6 -> CR &= ~DMA_SxCR_EN; //Enable DMA

}
