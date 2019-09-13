#include "Init_I2C.h"
#include "stm32f0xx.h"

#define EEPROM_OWN_ADDRESS (0x50)

void Delay_ms(uint32_t ms)
{
        volatile uint32_t nCount;
        RCC_ClocksTypeDef RCC_Clocks;
				RCC_GetClocksFreq (&RCC_Clocks);

        nCount=(RCC_Clocks.HCLK_Frequency/10000)*ms;
        for (; nCount!=0; nCount--);
}

void Delay_us(uint32_t us)
{
        volatile uint32_t nCount;
        RCC_ClocksTypeDef RCC_Clocks;
				RCC_GetClocksFreq (&RCC_Clocks);

        nCount=(RCC_Clocks.HCLK_Frequency/10000000)*us;
        for (; nCount!=0; nCount--);
}


void eeprom_pagewrite(uint8_t address, uint8_t *buf,uint8_t nbytes)
{
//   uint8_t Count=0;	// Счётчик успешно принятых байт
   I2C1->CR2 &= ~I2C_CR2_AUTOEND;
   I2C1->CR2 &= ~I2C_CR2_RD_WRN;			// Режим передачи
	I2C1->CR2 &= ~I2C_CR2_NBYTES;				// Очистить размер данных
	I2C1->CR2 |= ((nbytes+1)<<16);	// Установить размер данных
	I2C1->CR2 &= ~I2C_CR2_SADD;	// Очистить адрес ведомого устройства
	I2C1->CR2 |= (EEPROM_OWN_ADDRESS<<1);			// Установить адрес ведомого устройства
	I2C1->CR2 |= I2C_CR2_START;					// Выдать старт на шину
	while ((I2C1->ISR & I2C_ISR_BUSY)==0) {};	// Ожидать выдачу старта
   // Сейчас либо I2C запросит первый байт для отправки,
	// Либо взлетит NACK-флаг, говорящий о том, что микросхема не отвечает.
	// Если взлетит NACK-флаг, отправку прекращаем.
	while ((((I2C1->ISR & I2C_ISR_TXIS)==0) && ((I2C1->ISR & I2C_ISR_NACKF)==0)) && (I2C1->ISR & I2C_ISR_BUSY)) {};
	if (I2C1->ISR & I2C_ISR_TXIS) I2C1->TXDR=address;	// Отправляю адрес регистра
   // Отправляем байты до тех пор, пока не взлетит TC-флаг.
	// Если взлетит NACK-флаг, отправку прекращаем.
	while ((((I2C1->ISR & I2C_ISR_TC)==0) && ((I2C1->ISR & I2C_ISR_NACKF)==0)) && (I2C1->ISR & I2C_ISR_BUSY))
	{
		if (I2C1->ISR & I2C_ISR_TXIS) I2C1->TXDR=*(buf++);	// Отправляю данныеCount++
	}
   

//    //стоп
   I2C1->CR2 |= I2C_CR2_STOP;				// Выдать стоп на шину
	while (I2C1->ISR & I2C_ISR_BUSY) {};		// Ожидать выдачу стопа
	// Очищаю флаги - необходимо для дальнейшей работы шины
	I2C1->ICR |= I2C_ICR_STOPCF;		// STOP флаг
	I2C1->ICR |= I2C_ICR_NACKCF;		// NACK флаг
	// Если есть ошибки на шине - очищаю флаги
	if (I2C1->ISR & (I2C_ISR_ARLO | I2C_ISR_BERR))
	{
		I2C1->ICR |= I2C_ICR_ARLOCF;
		I2C1->ICR |= I2C_ICR_BERRCF;
	}
}

void eeprom_pageread(uint8_t address,uint8_t *buf,uint8_t nbytes)
{
//   uint8_t Count=0;	// Счётчик успешно принятых байт
   I2C1->CR2 &= ~I2C_CR2_AUTOEND;  //авто-СТОП отключаем
   I2C1->CR2 &= ~I2C_CR2_RD_WRN;			// Режим передачи
	I2C1->CR2 &= ~I2C_CR2_NBYTES;				// Очистить размер данных
	I2C1->CR2 |= ((1)<<16);	// Установить размер данных
	I2C1->CR2 &= ~I2C_CR2_SADD;	// Очистить адрес ведомого устройства
	I2C1->CR2 |= (EEPROM_OWN_ADDRESS<<1);			// Установить адрес ведомого устройства
	I2C1->CR2 |= I2C_CR2_START;					// Выдать старт на шину
	while ((I2C1->ISR & I2C_ISR_BUSY)==0) {};	// Ожидать выдачу старта
      // Сейчас либо I2C запросит первый байт для отправки,
	// Либо взлетит NACK-флаг, говорящий о том, что микросхема не отвечает.
	// Если взлетит NACK-флаг, отправку прекращаем.
	while ((((I2C1->ISR & I2C_ISR_TC)==0) && ((I2C1->ISR & I2C_ISR_NACKF)==0)) && (I2C1->ISR & I2C_ISR_BUSY))
	{
		if (I2C1->ISR & I2C_ISR_TXIS) I2C1->TXDR = address;	// Отправляю адрес регистра
	}
   // Повторный старт
   I2C1->CR2 |= I2C_CR2_RD_WRN;	// Режим приёма
	I2C1->CR2 &= ~I2C_CR2_NBYTES;				// Очистить размер данных
	I2C1->CR2 |= ((nbytes)<<16);	// Установить размер данных
	I2C1->CR2 &= ~I2C_CR2_SADD;	// Очистить адрес ведомого устройства
	I2C1->CR2 |= (EEPROM_OWN_ADDRESS<<1);			// Установить адрес ведомого устройства
	I2C1->CR2 |= I2C_CR2_START;					// Выдать старт на шину
	while ((I2C1->ISR & I2C_ISR_BUSY)==0) {};	// Ожидать выдачу старта
      // Принимаем байты до тех пор, пока не взлетит TC-флаг.
	// Если взлетит NACK-флаг, приём прекращаем.
	while ((((I2C1->ISR & I2C_ISR_TC)==0) && ((I2C1->ISR & I2C_ISR_NACKF)==0)) && (I2C1->ISR & I2C_ISR_BUSY))
	{
		if (I2C1->ISR & I2C_ISR_RXNE) *(buf++) = I2C1->RXDR;	// Принимаю данныеCount++
	}
//   //стоп
   I2C1->CR2 |= I2C_CR2_STOP;				// Выдать стоп на шину
	while (I2C1->ISR & I2C_ISR_BUSY) {};		// Ожидать выдачу стопа
	// Очищаю флаги - необходимо для дальнейшей работы шины
	I2C1->ICR |= I2C_ICR_STOPCF;		// STOP флаг
	I2C1->ICR |= I2C_ICR_NACKCF;		// NACK флаг
	// Если есть ошибки на шине - очищаю флаги
	if (I2C1->ISR & (I2C_ISR_ARLO | I2C_ISR_BERR))
	{
		I2C1->ICR |= I2C_ICR_ARLOCF;
		I2C1->ICR |= I2C_ICR_BERRCF;
	}
   

}

void EEPROM_float_write(uint8_t addr, float val) // запись в ЕЕПРОМ
{ 

//  uint8_t *x = (uint8_t *)&val;

//  for(uint8_t i = 0; i < 4; i++) EEPROM.write(i+addr, x[i]);
	eeprom_pagewrite(addr, (uint8_t *)&val,4);Delay_ms(10);

}
 

float EEPROM_float_read(uint8_t addr) // чтение из ЕЕПРОМ
{   

  float x;
//uint8_t *y = (uint8_t *)&x;
//  for(uint8_t i = 0; i < 4; i++) x[i] = EEPROM.read(i+addr);
	eeprom_pageread(addr, (uint8_t *)&x,4);
  

  return x;

}

void EEPROM_16bit_write(uint8_t addr, uint16_t val) // запись в ЕЕПРОМ
{ 
//uint8_t *x = (uint8_t *)&val;

//  for(uint8_t i = 0; i < 4; i++) EEPROM.write(i+addr, x[i]);
	eeprom_pagewrite(addr, (uint8_t *)&val,2);Delay_ms(10);

}

uint16_t EEPROM_16bit_read(uint8_t addr) // чтение из ЕЕПРОМ
{   
	uint16_t x;
uint8_t *y = (uint8_t *)&x;
//  for(uint8_t i = 0; i < 4; i++) x[i] = EEPROM.read(i+addr);
	eeprom_pageread(addr, y,2);
  

  return x;

}

void EEPROM_8bit_write(uint8_t addr, uint8_t val) // запись в ЕЕПРОМ
{ 
//uint8_t *x = (uint8_t *)&val;

//  for(uint8_t i = 0; i < 4; i++) EEPROM.write(i+addr, x[i]);
	eeprom_pagewrite(addr, (uint8_t *)&val,1);Delay_ms(10);

}

uint8_t EEPROM_8bit_read(uint8_t addr) // чтение из ЕЕПРОМ
{   
	uint16_t x;
uint8_t *y = (uint8_t *)&x;
//  for(uint8_t i = 0; i < 4; i++) x[i] = EEPROM.read(i+addr);
	eeprom_pageread(addr, y,1);
  

  return x;

}




