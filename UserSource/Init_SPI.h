#ifndef Init_SPI_H
#define Init_SPI_H

#include "stm32f0xx.h" 

//#define CS_ON    GPIO_ResetBits(GPIOA, GPIO_Pin_15) //GPIOA->BSRR |= GPIO_BSRR_BR4;// //
//#define CS_OFF    GPIO_SetBits(GPIOA, GPIO_Pin_15)//GPIOA->BSRR |= GPIO_BSRR_BS4;//  
#define CS_HIGH     GPIOB -> BSRR = GPIO_BSRR_BS_1
#define CS_LOW      GPIOB -> BSRR = GPIO_BSRR_BR_1


void AS5050_Init();
uint16_t SPI_SendByteAS (uint16_t byte);

uint16_t spiReadData();
static uint8_t spiCalcEvenParity(short value);

//void SPI1_SendByte(uint8_t byte);
void AT45DB161_Init();
uint8_t SPI_SendByteDB (uint8_t byte);
uint8_t AT45DB161_Read_Status();
void AT45DB161_Read_Data(uint16_t page, uint16_t addr, uint32_t length, uint8_t *out);
void AT45DB161_PageProgram(uint16_t page, uint8_t *data, uint16_t length);

uint8_t SPI1_SendByte(uint8_t byte);
void  SPI1_WR_String(const char *);
uint8_t SPI1_ReadByte(uint8_t byte);
void SPI_ini(void);

#endif

