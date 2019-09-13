#include "Init_SPI.h"
#include "stm32f0xx.h"
#include "stm32f0xx_gpio.h"

    
#define SPI1_DR_8bit          (*(__IO uint8_t *)((uint32_t)&(SPI1->DR)))
//Регистр SPI_DR c 8-битным доступом 
#define SPI_DR8     *(uint8_t *)0x4001300C
//#define PARITY_BIT 0x0001
//typedef uint8_t AS_RWTYPE;
//#define AS_READ   false
//#define AS_WRITE  true
//   



#define SPI_CMD_READ 0x8000 /*!< flag indicating read attempt when using SPI interface */
#define SPI_REG_DATA 0x7ffe /*!< data register when using SPI */
#define SPI_REG_AGC 0x7ff0 /*!< agc register when using SPI */
#define SPI_REG_CLRERR 0x6700 /*!< clear error register when using SPI */

static uint8_t spiCalcEvenParity(short value)
{
 uint8_t cnt = 0;
 uint8_t i;
 for (i = 0; i < 16; i++)
 {
 if (value & 0x1)
 {
 cnt++;
 }
 value >>= 1;
 }
 return cnt & 0x1;
} 

uint16_t spiReadData()
{
 uint16_t dat; // 16-bit data buffer for SPI communication
 short angle, agcreg;
 uint8_t agc;
 short value;
 uint8_t alarmHi, alarmLo;
 /* Send READ AGC command. Received data is thrown away: this data comes from the precedent
command (unknown)*/
 dat = SPI_CMD_READ | SPI_REG_AGC;
 dat |= spiCalcEvenParity(dat);
// spiTransfer((uint8_t*)&dat, sizeof(uint16_t));
   SPI_SendByteAS(dat);
 /* Send READ ANGLE command. Received data is the AGC value, from the precedent command */
 dat = SPI_CMD_READ | SPI_REG_DATA;
 dat |= spiCalcEvenParity(dat);
 //spiTransfer((uint8_t*)&dat, sizeof(uint16_t));
 agcreg = SPI_SendByteAS(dat);;
 /* Send NOP command. Received data is the ANGLE value, from the precedent command */
 dat = 0x0000; // NOP command.
 //spiTransfer((u8*)&dat, sizeof(u16));
   
 angle = (SPI_SendByteAS(0x3FFF)) >> 2;
 
 if (((dat >> 1) & 0x1) || ((agcreg >> 1) & 0x1))
 {
 /* error flag set - need to reset it */
 dat = SPI_CMD_READ | SPI_REG_CLRERR;
 dat |= spiCalcEvenParity(dat);
 //spiTransfer((u8*)&dat, sizeof(u16));
    SPI_SendByteAS(dat);
 }
 else
 {
 agc = (agcreg >> 2) & 0x3f; // AGC value (0..63)
 value = (dat >> 2) & 0x3fff; // Angle value (0..4095 for AS5055)
 angle = (value * 360) / 4095; // Angle value in degree (0..359.9°)
 alarmLo = (dat >> 14) & 0x1;
 alarmHi = (dat >> 15) & 0x1;
 }
 return angle;
} 




//// Проверка чётности
//static uint8_t CheckEvenParity(uint16_t uWord)
//{
//  uint8_t uOnes = 0;
//  uint8_t uMask = 1;
//  uint8_t i;
//   
//  for (i = 0; i < 16; i++)
//  {
//    if (uWord & (uMask << i))
//      uOnes++;
//  }    
// 
//  return (0 == (uOnes % 2));
//}
// 
//// количество бит "1" должно быть чётным
//static void SetEvenParity(uint16_t *uWord)
//{
////  if (NULL == uWord)
////    return;
//   
//  if (!CheckEvenParity(*uWord))
//  {
//    *uWord |= PARITY_BIT;
//  }  
//}

//static uint16_t MakeCommand(uint16_t uAddress, AS_RWTYPE bWrite)
//{
//  uint16_t uResult;  
//   
//  uResult = uAddress << 1;
// 
////  if (bWrite)
////    uResult &= ~WRITE_BIT;
////  else
////    uResult |=  WRITE_BIT;
////   
////  SetEvenParity(&uResult);
//   
//  return uResult;
//}



void AS5050_Init(){
//Включаем тактирование SPI1, GPIOA, GPIOB
    RCC -> APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC -> AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;
//Настраиваем  PA5, PA6, PA7 как AF
    GPIOA -> MODER |= GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;
//High Speed
    GPIOA -> OSPEEDR |= GPIO_OSPEEDER_OSPEEDR5 | GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7;
//Настраиваем  PB1 как GP OUTPUT
    GPIOB -> MODER |= GPIO_MODER_MODER1_0;
//High Speed
    GPIOB -> OSPEEDR |= GPIO_OSPEEDER_OSPEEDR1;
//CS = 1
    CS_HIGH;
/*
SPI_CR1_SSM – программное управление выводом NSS (внутри микросхемы)
SPI_CR1_SSI – активный уровень на NSS
SPI_CR1_MSTR – режим SPI – мастер
SPI_BR = 0 – частота SPI = fclk / 2 = 24МГц
*/
    SPI1 -> CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR;
   SPI1->CR1 |= 
			 SPI_CR1_BR_0 |     //011: fPCLK/16
			 SPI_CR1_BR_1 ;
/*
SPI_CR2_DS = 15 – режим 16 бит
SPI_CR2_FRXTH – событие RXNE будет генерироваться, когда FIFO будет заполнено на ? - то есть когда придет 1 байт
*/
    SPI1 -> CR2 = SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2 | SPI_CR2_DS_3 | SPI_CR2_FRXTH;
//SPI_CR1_SPE - включаем SPI
    SPI1 -> CR1 |= SPI_CR1_SPE;
}

uint16_t SPI_SendByteAS (uint16_t byte){
//Ждем пока передающий буфер не пуст
    while(!(SPI1-> SR & SPI_SR_TXE));
//Передаем байт
    SPI1->DR = byte;
//Ждем пока приемный буфер пуст
    while(!(SPI1-> SR & SPI_SR_RXNE));
//Возвращаем принятое значение
    return SPI1->DR;
}

//Для памяти
void AT45DB161_Init(){
//Включаем тактирование SPI1, GPIOA, GPIOB
    RCC -> APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC -> AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;
//Настраиваем  PA5, PA6, PA7 как AF
    GPIOA -> MODER |= GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;
//High Speed
    GPIOA -> OSPEEDR |= GPIO_OSPEEDER_OSPEEDR5 | GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7;
//Настраиваем  PB1 как GP OUTPUT
    GPIOB -> MODER |= GPIO_MODER_MODER1_0;
//High Speed
    GPIOB -> OSPEEDR |= GPIO_OSPEEDER_OSPEEDR1;
//CS = 1
    CS_HIGH;
/*
SPI_CR1_SSM – программное управление выводом NSS (внутри микросхемы)
SPI_CR1_SSI – активный уровень на NSS
SPI_CR1_MSTR – режим SPI – мастер
SPI_BR = 0 – частота SPI = fclk / 2 = 24МГц
*/
    SPI1 -> CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR;
/*
SPI_CR2_DS = 7 – режим 8 бит
SPI_CR2_FRXTH – событие RXNE будет генерироваться, когда FIFO будет заполнено на ? - то есть когда придет 1 байт
*/
    SPI1 -> CR2 = SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2 | SPI_CR2_FRXTH;
//SPI_CR1_SPE - включаем SPI
    SPI1 -> CR1 |= SPI_CR1_SPE;
}

uint8_t SPI_SendByteDB (uint8_t byte){
//Ждем пока передающий буфер не пуст
    while(!(SPI1-> SR & SPI_SR_TXE));
//Передаем байт
    SPI_DR8 = byte;
//Ждем пока приемный буфер пуст
    while(!(SPI1-> SR & SPI_SR_RXNE));
//Возвращаем принятое значение
    return SPI_DR8;
}

uint8_t AT45DB161_Read_Status(){
    uint8_t temp;
//CS = 0 
    CS_LOW;
//Отправляем код команды
    SPI_SendByteDB(0xD7);
//Читаем байт статуса
    temp = SPI_SendByteDB(0x00);
//Ждем пока завершится работа SPI
    while(SPI1 -> SR & SPI_SR_BSY);
//CS = 1
    CS_HIGH;
//Возвращаем байт статуса
    return temp;
}

/* uint16_t page – номер читаемой страницы (0 – 4095)
   uint16_t addr – адрес байта внутри страницы (0 – 511)
   uint32_t length – количество читаемых байт
   uint8_t *out – указатель, куда ложить прочитанные байты
*/
void AT45DB161_Read_Data(uint16_t page, uint16_t addr, uint32_t length, uint8_t *out){
    uint32_t i;
    uint8_t temp;
//Ожидаем готовности микросхемы
do {
        temp = AT45DB161_Read_Status();
    } while (!(temp & 0x80));
//Исходя из размера страницы формируем адрес
    if (temp & 0x01){   //512
        i = ((page << 9) | (addr & 0x1FF));
    } else {            //528
        i = ((page << 10) | (addr & 0x3FF)) ;
    }
//CS = 0 
    CS_LOW;
//Отправляем код команды 
    SPI_SendByteDB(0x0B);
//Отправляем адрес 
    SPI_SendByteDB(i >> 16);
    SPI_SendByteDB(i >> 8);
    SPI_SendByteDB(i);
//Отправляем произвольный байт
    SPI_SendByteDB(0x00);
//Читаем последующие байты в out[]
    for (i = 0; i < length; i++){
        out[i] = SPI_SendByteDB(0xFF);
    }
//Ждем завершения работы SPI
    while(SPI1 -> SR & SPI_SR_BSY);
//CS = 1 
    CS_HIGH;
}

void AT45DB161_PageProgram(uint16_t page, uint8_t *data, uint16_t length){
    uint16_t i;
    uint8_t temp;
//Читаем регистр статуса, чтобы определить размер страницы
    temp = AT45DB161_Read_Status();
    CS_LOW;
//Пишем данные в буфер 1 с 0 адреса
    SPI_SendByteDB(0x84);
    SPI_SendByteDB(0x00);
    SPI_SendByteDB(0x00);
    SPI_SendByteDB(0x00);
    for (i = 0; i < length; i++){
        SPI_SendByteDB(data[i]);
    }
    while(SPI1 -> SR & SPI_SR_BSY);
    CS_HIGH;
    CS_LOW;
//Посылаем команду 83h
    SPI_SendByteDB(0x83);
//Исходя из размера страницы формируем адрес
    if (temp & 0x01){   //512
        SPI_SendByteDB((uint8_t)(page >> 7));
        SPI_SendByteDB((uint8_t)((page & 0x7F) << 1));
        SPI_SendByteDB(0x00);
    } else {            //528
        SPI_SendByteDB((uint8_t)(page >> 6));
        SPI_SendByteDB((uint8_t)((page & 0x3F) << 2));
        SPI_SendByteDB(0x00);
    }
    while(SPI1 -> SR & SPI_SR_BSY);
    CS_HIGH;
//Ожидаем готовности микросхемы
    while (!(AT45DB161_Read_Status() & 0x80));
}

//Для дисплея
uint8_t SPI1_SendByte(uint8_t data)
{
   uint32_t i; //Объявляем переменную для цикла задержки
// CS_ON;     
   SPI1_DR_8bit = data;                      
        while ((SPI1->SR & SPI_SR_BSY)); 
//   CS_OFF;
//   while (!(SPI1->SR & SPI_SR_RXNE));    
//        for(i=0; i<300; i++);  //Задержка
        return (SPI1_DR_8bit );	
   
}
//==============================================================================
void  SPI1_WR_String(const char *s)
{ int i = 0;
 while (s [i] != 0) {
	SPI1_SendByte (s[i++]);
	}
}
//==============================================================================
uint8_t SPI1_ReadByte(uint8_t byte)
{
//  while (!(SPI1->SR & SPI_SR_TXE));
  SPI1->DR=byte;
  while (!(SPI1->SR & SPI_SR_RXNE));
  return (SPI1->DR);
}

void SPI_ini(void)
{
//AF0
        //PF7 DC
        // PA15 CS
 RCC->AHBENR |=RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOAEN;															 
 GPIOB->MODER &=~ GPIO_MODER_MODER3 | // MOSI
                  GPIO_MODER_MODER5 |// SCK
                  GPIO_MODER_MODER4 ; // DC
 GPIOA->MODER &=~ GPIO_MODER_MODER15; // NSS
// GPIOF->MODER &=~ GPIO_MODER_MODER7;
 GPIOB->MODER |=  GPIO_MODER_MODER5_1| // MODER = 10 Alternate Function
                  GPIO_MODER_MODER3_1 |
                  GPIO_MODER_MODER4_0 ; //DC out
 GPIOA->MODER |=  GPIO_MODER_MODER15_0; //   //GPIO_MODER_MODER15_0 ; // out
 GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR15; // high cpeed
 GPIOA->PUPDR |= GPIO_PUPDR_PUPDR15_0 ; // PUPDR2 = 01  PULLUP   PUPDR2 = 10  PULLDN 
// GPIOF->MODER |=  GPIO_MODER_MODER7_0 ;  //DC
 RCC->APB2ENR |=  RCC_APB2ENR_SPI1EN;
// 000: fPCLK/2
//001: fPCLK/4
//010: fPCLK/8
//011: fPCLK/16
//100: fPCLK/32
//101: fPCLK/64
//110: fPCLK/128
//111: fPCLK/256               
 SPI1->CR1 |= 
			 SPI_CR1_BR_0 |     //011: fPCLK/16
			 SPI_CR1_BR_1 | 
//          SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE | 
//			 SPI_CR1_BR_2 | 
			 SPI_CR1_MSTR |
			 SPI_CR1_SSI 	|				 
			 SPI_CR1_SSM 
         
//         | SPI_CR1_CPOL
//         | SPI_CR1_CPHA
						 ;	
	SPI1->CR2  = 0x700;   //  8 bit
//	SPI1->CR2 |= SPI_CR2_FRXTH;
	SPI1->CR1 |= SPI_CR1_SPE;	

}

