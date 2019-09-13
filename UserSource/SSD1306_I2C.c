#include "SSD1306_I2C.h"
#include "Init_I2C.h"
#include "stm32f0xx.h"
//#include "string.h"

    
#define SSD1306_I2C_ADDR1        0x3C
#define SSD1306_I2C_ADDR2        0x3D 
#define SSD1306_I2C_ADDR3        0x3C
#define SSD1306_I2C_ADDR4        0x3D 
#define SSD1306_WIDTH            128
#define SSD1306_HEIGHT           64
#define I2C_TRANSMITTER_MODE     0
#define I2C_RECEIVER_MODE        1
#define I2C_ACK_ENABLE           1
#define I2C_ACK_DISABLE          0
//#define ssd1306_I2C_TIMEOUT		 1000
//#define SSD1306_I2C              I2C1


static uint8_t SSD1306_Buffer_1[SSD1306_WIDTH * SSD1306_HEIGHT / 8];
static uint8_t SSD1306_Buffer_2[SSD1306_WIDTH * SSD1306_HEIGHT / 8];
//static uint8_t SSD1306_Buffer_3[SSD1306_WIDTH * SSD1306_HEIGHT / 8];
//static uint8_t SSD1306_Buffer_4[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

uint16_t I2C_Timout = 0;
uint8_t Timeout = 0;

uint16_t I2C_Timout2 = 0;
uint8_t Timeout2 = 0;

 
 
void SSD1306_Fill_1(SSD1306_COLOR_t color) {
	/* Set memory */
	memset(SSD1306_Buffer_1, (color == SSD1306_COLOR_BLACK) ? 0x00 : 0xFF, sizeof(SSD1306_Buffer_1));
}

void SSD1306_Fill_2(SSD1306_COLOR_t color) {
	/* Set memory */
	memset(SSD1306_Buffer_1, (color == SSD1306_COLOR_BLACK) ? 0x00 : 0xFF, sizeof(SSD1306_Buffer_1));
}
 
void SSD1306_Fill_3(SSD1306_COLOR_t color) {
	/* Set memory */
	memset(SSD1306_Buffer_2, (color == SSD1306_COLOR_BLACK) ? 0x00 : 0xFF, sizeof(SSD1306_Buffer_2));
}

void SSD1306_Fill_4(SSD1306_COLOR_t color) {
	/* Set memory */
	memset(SSD1306_Buffer_2, (color == SSD1306_COLOR_BLACK) ? 0x00 : 0xFF, sizeof(SSD1306_Buffer_2));
}
 
void ssd1306_send_1(uint8_t control_byte, uint8_t data)
{
//   I2C1->CR2 =  I2C_CR2_AUTOEND | (2<<16) | (SSD1306_I2C_ADDR1<<1); 
//   /* Check Tx empty */
//   //while (!(I2C1->ISR & I2C_ISR_TXE) );
//   while ( (!(I2C1->ISR & I2C_ISR_TXE) ) && (Timeout == 0) )
//   {
//      I2C_Timout++;
//      if (I2C_Timout == ssd1306_I2C_TIMEOUT) {I2C_Timout = 0;Timeout = 1; }
//   };
//   Timeout = 0;
//   I2C_Timout = 0;

//   I2C1->TXDR = control_byte; 
//   I2C1->CR2 |= I2C_CR2_START; 
//   //таймаут
//   while ( (!(I2C1->ISR & I2C_ISR_TXE) ) && (Timeout == 0) )
//   {
//      I2C_Timout++;
//      if (I2C_Timout == ssd1306_I2C_TIMEOUT) {I2C_Timout = 0;Timeout = 1; }
//   };
//   Timeout = 0;
//   I2C_Timout = 0;
//   I2C1->TXDR = data; 
   
//   uint8_t Count=0;	// Счётчик успешно принятых байт
   I2C1->CR2 &= ~I2C_CR2_AUTOEND;
   I2C1->CR2 &= ~I2C_CR2_RD_WRN;			// Режим передачи
	I2C1->CR2 &= ~I2C_CR2_NBYTES;				// Очистить размер данных
	I2C1->CR2 |= (2<<16);	// Установить размер данных
	I2C1->CR2 &= ~I2C_CR2_SADD;	// Очистить адрес ведомого устройства
	I2C1->CR2 |= (SSD1306_I2C_ADDR1<<1);			// Установить адрес ведомого устройства
	I2C1->CR2 |= I2C_CR2_START;					// Выдать старт на шину
	while ((I2C1->ISR & I2C_ISR_BUSY)==0) {};	// Ожидать выдачу старта
   // Сейчас либо I2C запросит первый байт для отправки,
	// Либо взлетит NACK-флаг, говорящий о том, что микросхема не отвечает.
	// Если взлетит NACK-флаг, отправку прекращаем.
	while ((((I2C1->ISR & I2C_ISR_TXIS)==0) && ((I2C1->ISR & I2C_ISR_NACKF)==0)) && (I2C1->ISR & I2C_ISR_BUSY)) {};
	if (I2C1->ISR & I2C_ISR_TXIS) I2C1->TXDR = control_byte;	// Отправляю адрес регистра
   // Отправляем байты до тех пор, пока не взлетит TC-флаг.
	// Если взлетит NACK-флаг, отправку прекращаем.
	while ((((I2C1->ISR & I2C_ISR_TC)==0) && ((I2C1->ISR & I2C_ISR_NACKF)==0)) && (I2C1->ISR & I2C_ISR_BUSY))
	{
		if (I2C1->ISR & I2C_ISR_TXIS) I2C1->TXDR=data;	// Отправляю данныеCount++
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

void ssd1306_send_2(uint8_t control_byte, uint8_t data)
{

//   uint8_t Count=0;	// Счётчик успешно принятых байт
   I2C1->CR2 &= ~I2C_CR2_AUTOEND;
   I2C1->CR2 &= ~I2C_CR2_RD_WRN;			// Режим передачи
	I2C1->CR2 &= ~I2C_CR2_NBYTES;				// Очистить размер данных
	I2C1->CR2 |= (2<<16);	// Установить размер данных
	I2C1->CR2 &= ~I2C_CR2_SADD;	// Очистить адрес ведомого устройства
	I2C1->CR2 |= (SSD1306_I2C_ADDR2<<1);			// Установить адрес ведомого устройства
	I2C1->CR2 |= I2C_CR2_START;					// Выдать старт на шину
	while ((I2C1->ISR & I2C_ISR_BUSY)==0) {};	// Ожидать выдачу старта
   // Сейчас либо I2C запросит первый байт для отправки,
	// Либо взлетит NACK-флаг, говорящий о том, что микросхема не отвечает.
	// Если взлетит NACK-флаг, отправку прекращаем.
	while ((((I2C1->ISR & I2C_ISR_TXIS)==0) && ((I2C1->ISR & I2C_ISR_NACKF)==0)) && (I2C1->ISR & I2C_ISR_BUSY)) {};
	if (I2C1->ISR & I2C_ISR_TXIS) I2C1->TXDR = control_byte;	// Отправляю адрес регистра
   // Отправляем байты до тех пор, пока не взлетит TC-флаг.
	// Если взлетит NACK-флаг, отправку прекращаем.
	while ((((I2C1->ISR & I2C_ISR_TC)==0) && ((I2C1->ISR & I2C_ISR_NACKF)==0)) && (I2C1->ISR & I2C_ISR_BUSY))
	{
		if (I2C1->ISR & I2C_ISR_TXIS) I2C1->TXDR=data;	// Отправляю данныеCount++
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

void ssd1306_send_3(uint8_t control_byte, uint8_t data)
{

//   uint8_t Count=0;	// Счётчик успешно принятых байт
   I2C2->CR2 &= ~I2C_CR2_AUTOEND;
   I2C2->CR2 &= ~I2C_CR2_RD_WRN;			// Режим передачи
	I2C2->CR2 &= ~I2C_CR2_NBYTES;				// Очистить размер данных
	I2C2->CR2 |= (2<<16);	// Установить размер данных
	I2C2->CR2 &= ~I2C_CR2_SADD;	// Очистить адрес ведомого устройства
	I2C2->CR2 |= (SSD1306_I2C_ADDR3<<1);			// Установить адрес ведомого устройства
	I2C2->CR2 |= I2C_CR2_START;					// Выдать старт на шину
	while ((I2C2->ISR & I2C_ISR_BUSY)==0) {};	// Ожидать выдачу старта
   // Сейчас либо I2C запросит первый байт для отправки,
	// Либо взлетит NACK-флаг, говорящий о том, что микросхема не отвечает.
	// Если взлетит NACK-флаг, отправку прекращаем.
	while ((((I2C2->ISR & I2C_ISR_TXIS)==0) && ((I2C2->ISR & I2C_ISR_NACKF)==0)) && (I2C2->ISR & I2C_ISR_BUSY)) {};
	if (I2C2->ISR & I2C_ISR_TXIS) I2C2->TXDR = control_byte;	// Отправляю адрес регистра
   // Отправляем байты до тех пор, пока не взлетит TC-флаг.
	// Если взлетит NACK-флаг, отправку прекращаем.
	while ((((I2C2->ISR & I2C_ISR_TC)==0) && ((I2C2->ISR & I2C_ISR_NACKF)==0)) && (I2C2->ISR & I2C_ISR_BUSY))
	{
		if (I2C2->ISR & I2C_ISR_TXIS) I2C2->TXDR=data;	// Отправляю данныеCount++
	}
   

//    //стоп
   I2C2->CR2 |= I2C_CR2_STOP;				// Выдать стоп на шину
	while (I2C2->ISR & I2C_ISR_BUSY) {};		// Ожидать выдачу стопа
	// Очищаю флаги - необходимо для дальнейшей работы шины
	I2C2->ICR |= I2C_ICR_STOPCF;		// STOP флаг
	I2C2->ICR |= I2C_ICR_NACKCF;		// NACK флаг
	// Если есть ошибки на шине - очищаю флаги
	if (I2C2->ISR & (I2C_ISR_ARLO | I2C_ISR_BERR))
	{
		I2C2->ICR |= I2C_ICR_ARLOCF;
		I2C2->ICR |= I2C_ICR_BERRCF;
	}

}

void ssd1306_send_4(uint8_t control_byte, uint8_t data)
{

//   uint8_t Count=0;	// Счётчик успешно принятых байт
   I2C2->CR2 &= ~I2C_CR2_AUTOEND;
   I2C2->CR2 &= ~I2C_CR2_RD_WRN;			// Режим передачи
	I2C2->CR2 &= ~I2C_CR2_NBYTES;				// Очистить размер данных
	I2C2->CR2 |= (2<<16);	// Установить размер данных
	I2C2->CR2 &= ~I2C_CR2_SADD;	// Очистить адрес ведомого устройства
	I2C2->CR2 |= (SSD1306_I2C_ADDR4<<1);			// Установить адрес ведомого устройства
	I2C2->CR2 |= I2C_CR2_START;					// Выдать старт на шину
	while ((I2C2->ISR & I2C_ISR_BUSY)==0) {};	// Ожидать выдачу старта
   // Сейчас либо I2C запросит первый байт для отправки,
	// Либо взлетит NACK-флаг, говорящий о том, что микросхема не отвечает.
	// Если взлетит NACK-флаг, отправку прекращаем.
	while ((((I2C2->ISR & I2C_ISR_TXIS)==0) && ((I2C2->ISR & I2C_ISR_NACKF)==0)) && (I2C2->ISR & I2C_ISR_BUSY)) {};
	if (I2C2->ISR & I2C_ISR_TXIS) I2C2->TXDR = control_byte;	// Отправляю адрес регистра
   // Отправляем байты до тех пор, пока не взлетит TC-флаг.
	// Если взлетит NACK-флаг, отправку прекращаем.
	while ((((I2C2->ISR & I2C_ISR_TC)==0) && ((I2C2->ISR & I2C_ISR_NACKF)==0)) && (I2C2->ISR & I2C_ISR_BUSY))
	{
		if (I2C2->ISR & I2C_ISR_TXIS) I2C2->TXDR=data;	// Отправляю данныеCount++
	}
   

//    //стоп
   I2C2->CR2 |= I2C_CR2_STOP;				// Выдать стоп на шину
	while (I2C2->ISR & I2C_ISR_BUSY) {};		// Ожидать выдачу стопа
	// Очищаю флаги - необходимо для дальнейшей работы шины
	I2C2->ICR |= I2C_ICR_STOPCF;		// STOP флаг
	I2C2->ICR |= I2C_ICR_NACKCF;		// NACK флаг
	// Если есть ошибки на шине - очищаю флаги
	if (I2C2->ISR & (I2C_ISR_ARLO | I2C_ISR_BERR))
	{
		I2C2->ICR |= I2C_ICR_ARLOCF;
		I2C2->ICR |= I2C_ICR_BERRCF;
	}

}
		
void ssd1306_Multisend_1(uint8_t control_byte, uint8_t* data, uint16_t count)
{
//   uint8_t i;
//   I2C1->CR2 = I2C_CR2_AUTOEND | ((count+1)<<16) | (SSD1306_I2C_ADDR1<<1); // | I2C_CR2_AUTOEND пока count не поставил +1 передача в экран не прописывалась, но I2C шла передача

//   /* Check Tx empty */
//   //while (!(I2C1->ISR & I2C_ISR_TXE) );
//   while ( (!(I2C1->ISR & I2C_ISR_TXE) ) && (Timeout == 0) )
//   {
//      I2C_Timout++;
//      if (I2C_Timout == ssd1306_I2C_TIMEOUT) {I2C_Timout = 0;Timeout = 1; }
//   };
//   Timeout = 0;
//   I2C_Timout = 0;

//   I2C1->TXDR = control_byte; 
//   I2C1->CR2 |= I2C_CR2_START; 
//   //	while (!(I2C1->ISR & I2C_ISR_TXIS) );	
//   while ( (!(I2C1->ISR & I2C_ISR_TXIS) ) && (Timeout == 0) )
//   {
//      I2C_Timout++;
//      if (I2C_Timout == ssd1306_I2C_TIMEOUT) {I2C_Timout = 0;Timeout = 1; }
//   };
//   Timeout = 0;
//   I2C_Timout = 0;


//   for (i = 0; i < count; i++) 
//   {
//      I2C1->TXDR = data[i];
//      //while (!(I2C1->ISR & I2C_ISR_TXE) );
//      while ( (!(I2C1->ISR & I2C_ISR_TXE) ) && (Timeout == 0) )
//      {
//         I2C_Timout++;
//         if (I2C_Timout == ssd1306_I2C_TIMEOUT) {I2C_Timout = 0;Timeout = 1; }
//      };
//      Timeout = 0;
//      I2C_Timout = 0;
//   }
//   I2C1->CR2 |= I2C_CR2_STOP;
//////////////   
//   uint8_t Count=0;	// Счётчик успешно принятых байт
   I2C1->CR2 &= ~I2C_CR2_AUTOEND;
   I2C1->CR2 &= ~I2C_CR2_RD_WRN;			// Режим передачи
	I2C1->CR2 &= ~I2C_CR2_NBYTES;				// Очистить размер данных
	I2C1->CR2 |= ((count+1)<<16);	// Установить размер данных
	I2C1->CR2 &= ~I2C_CR2_SADD;	// Очистить адрес ведомого устройства
	I2C1->CR2 |= (SSD1306_I2C_ADDR1<<1);			// Установить адрес ведомого устройства
	I2C1->CR2 |= I2C_CR2_START;					// Выдать старт на шину
	while ((I2C1->ISR & I2C_ISR_BUSY)==0) {};	// Ожидать выдачу старта
   // Сейчас либо I2C запросит первый байт для отправки,
	// Либо взлетит NACK-флаг, говорящий о том, что микросхема не отвечает.
	// Если взлетит NACK-флаг, отправку прекращаем.
	while ((((I2C1->ISR & I2C_ISR_TXIS)==0) && ((I2C1->ISR & I2C_ISR_NACKF)==0)) && (I2C1->ISR & I2C_ISR_BUSY)) {};
	if (I2C1->ISR & I2C_ISR_TXIS) I2C1->TXDR = control_byte;	// Отправляю адрес регистра
   // Отправляем байты до тех пор, пока не взлетит TC-флаг.
	// Если взлетит NACK-флаг, отправку прекращаем.
	while ((((I2C1->ISR & I2C_ISR_TC)==0) && ((I2C1->ISR & I2C_ISR_NACKF)==0)) && (I2C1->ISR & I2C_ISR_BUSY))
	{
		if (I2C1->ISR & I2C_ISR_TXIS) I2C1->TXDR=*(data++);	// Отправляю данныеCount++
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
      
void 	ssd1306_Multisend_2(uint8_t control_byte, uint8_t* data, uint16_t count)
{
//   uint8_t Count=0;	// Счётчик успешно принятых байт
   I2C1->CR2 &= ~I2C_CR2_AUTOEND;
   I2C1->CR2 &= ~I2C_CR2_RD_WRN;			// Режим передачи
	I2C1->CR2 &= ~I2C_CR2_NBYTES;				// Очистить размер данных
	I2C1->CR2 |= ((count+1)<<16);	// Установить размер данных
	I2C1->CR2 &= ~I2C_CR2_SADD;	// Очистить адрес ведомого устройства
	I2C1->CR2 |= (SSD1306_I2C_ADDR2<<1);			// Установить адрес ведомого устройства
	I2C1->CR2 |= I2C_CR2_START;					// Выдать старт на шину
	while ((I2C1->ISR & I2C_ISR_BUSY)==0) {};	// Ожидать выдачу старта
   // Сейчас либо I2C запросит первый байт для отправки,
	// Либо взлетит NACK-флаг, говорящий о том, что микросхема не отвечает.
	// Если взлетит NACK-флаг, отправку прекращаем.
	while ((((I2C1->ISR & I2C_ISR_TXIS)==0) && ((I2C1->ISR & I2C_ISR_NACKF)==0)) && (I2C1->ISR & I2C_ISR_BUSY)) {};
	if (I2C1->ISR & I2C_ISR_TXIS) I2C1->TXDR = control_byte;	// Отправляю адрес регистра
   // Отправляем байты до тех пор, пока не взлетит TC-флаг.
	// Если взлетит NACK-флаг, отправку прекращаем.
	while ((((I2C1->ISR & I2C_ISR_TC)==0) && ((I2C1->ISR & I2C_ISR_NACKF)==0)) && (I2C1->ISR & I2C_ISR_BUSY))
	{
		if (I2C1->ISR & I2C_ISR_TXIS) I2C1->TXDR=*(data++);	// Отправляю данныеCount++
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

void 	ssd1306_Multisend_3(uint8_t control_byte, uint8_t* data, uint16_t count)
{
//   uint8_t Count=0;	// Счётчик успешно принятых байт
   I2C2->CR2 &= ~I2C_CR2_AUTOEND;
   I2C2->CR2 &= ~I2C_CR2_RD_WRN;			// Режим передачи
	I2C2->CR2 &= ~I2C_CR2_NBYTES;				// Очистить размер данных
	I2C2->CR2 |= ((count+1)<<16);	// Установить размер данных
	I2C2->CR2 &= ~I2C_CR2_SADD;	// Очистить адрес ведомого устройства
	I2C2->CR2 |= (SSD1306_I2C_ADDR3<<1);			// Установить адрес ведомого устройства
	I2C2->CR2 |= I2C_CR2_START;					// Выдать старт на шину
	while ((I2C2->ISR & I2C_ISR_BUSY)==0) {};	// Ожидать выдачу старта
   // Сейчас либо I2C запросит первый байт для отправки,
	// Либо взлетит NACK-флаг, говорящий о том, что микросхема не отвечает.
	// Если взлетит NACK-флаг, отправку прекращаем.
	while ((((I2C2->ISR & I2C_ISR_TXIS)==0) && ((I2C2->ISR & I2C_ISR_NACKF)==0)) && (I2C2->ISR & I2C_ISR_BUSY)) {};
	if (I2C2->ISR & I2C_ISR_TXIS) I2C2->TXDR = control_byte;	// Отправляю адрес регистра
   // Отправляем байты до тех пор, пока не взлетит TC-флаг.
	// Если взлетит NACK-флаг, отправку прекращаем.
	while ((((I2C2->ISR & I2C_ISR_TC)==0) && ((I2C2->ISR & I2C_ISR_NACKF)==0)) && (I2C2->ISR & I2C_ISR_BUSY))
	{
		if (I2C2->ISR & I2C_ISR_TXIS) I2C2->TXDR=*(data++);	// Отправляю данныеCount++
	}
   

//    //стоп
   I2C2->CR2 |= I2C_CR2_STOP;				// Выдать стоп на шину
	while (I2C2->ISR & I2C_ISR_BUSY) {};		// Ожидать выдачу стопа
	// Очищаю флаги - необходимо для дальнейшей работы шины
	I2C2->ICR |= I2C_ICR_STOPCF;		// STOP флаг
	I2C2->ICR |= I2C_ICR_NACKCF;		// NACK флаг
	// Если есть ошибки на шине - очищаю флаги
	if (I2C2->ISR & (I2C_ISR_ARLO | I2C_ISR_BERR))
	{
		I2C2->ICR |= I2C_ICR_ARLOCF;
		I2C2->ICR |= I2C_ICR_BERRCF;
	}
}

void 	ssd1306_Multisend_4(uint8_t control_byte, uint8_t* data, uint16_t count)
{
//   uint8_t Count=0;	// Счётчик успешно принятых байт
   I2C2->CR2 &= ~I2C_CR2_AUTOEND;
   I2C2->CR2 &= ~I2C_CR2_RD_WRN;			// Режим передачи
	I2C2->CR2 &= ~I2C_CR2_NBYTES;				// Очистить размер данных
	I2C2->CR2 |= ((count+1)<<16);	// Установить размер данных
	I2C2->CR2 &= ~I2C_CR2_SADD;	// Очистить адрес ведомого устройства
	I2C2->CR2 |= (SSD1306_I2C_ADDR4<<1);			// Установить адрес ведомого устройства
	I2C2->CR2 |= I2C_CR2_START;					// Выдать старт на шину
	while ((I2C2->ISR & I2C_ISR_BUSY)==0) {};	// Ожидать выдачу старта
   // Сейчас либо I2C запросит первый байт для отправки,
	// Либо взлетит NACK-флаг, говорящий о том, что микросхема не отвечает.
	// Если взлетит NACK-флаг, отправку прекращаем.
	while ((((I2C2->ISR & I2C_ISR_TXIS)==0) && ((I2C2->ISR & I2C_ISR_NACKF)==0)) && (I2C2->ISR & I2C_ISR_BUSY)) {};
	if (I2C2->ISR & I2C_ISR_TXIS) I2C2->TXDR = control_byte;	// Отправляю адрес регистра
   // Отправляем байты до тех пор, пока не взлетит TC-флаг.
	// Если взлетит NACK-флаг, отправку прекращаем.
	while ((((I2C2->ISR & I2C_ISR_TC)==0) && ((I2C2->ISR & I2C_ISR_NACKF)==0)) && (I2C2->ISR & I2C_ISR_BUSY))
	{
		if (I2C2->ISR & I2C_ISR_TXIS) I2C2->TXDR=*(data++);	// Отправляю данныеCount++
	}
   

//    //стоп
   I2C2->CR2 |= I2C_CR2_STOP;				// Выдать стоп на шину
	while (I2C2->ISR & I2C_ISR_BUSY) {};		// Ожидать выдачу стопа
	// Очищаю флаги - необходимо для дальнейшей работы шины
	I2C2->ICR |= I2C_ICR_STOPCF;		// STOP флаг
	I2C2->ICR |= I2C_ICR_NACKCF;		// NACK флаг
	// Если есть ошибки на шине - очищаю флаги
	if (I2C2->ISR & (I2C_ISR_ARLO | I2C_ISR_BERR))
	{
		I2C2->ICR |= I2C_ICR_ARLOCF;
		I2C2->ICR |= I2C_ICR_BERRCF;
	}
}

void SSD1306_ToggleInvert_1(void) {
	uint16_t i;
	
	/* Toggle invert */
	SSD1306.Inverted = !SSD1306.Inverted;
	
	/* Do memory toggle */
	for (i = 0; i < sizeof(SSD1306_Buffer_1); i++) {
		SSD1306_Buffer_1[i] = ~SSD1306_Buffer_1[i];
	}
}

void SSD1306_DrawPixel_1(uint16_t x, uint16_t y, SSD1306_COLOR_t color) {
	if (
		x >= SSD1306_WIDTH ||
		y >= SSD1306_HEIGHT
	) {
		/* Error */
		return;
	}
	
	/* Check if pixels are inverted */
	if (SSD1306.Inverted) {
		color = (SSD1306_COLOR_t)!color;
	}
	
	/* Set color */
	if (color == SSD1306_COLOR_WHITE) {
		SSD1306_Buffer_1[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
	} else {
		SSD1306_Buffer_1[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
	}
}

void SSD1306_DrawPixel_2(uint16_t x, uint16_t y, SSD1306_COLOR_t color) {
	if (
		x >= SSD1306_WIDTH ||
		y >= SSD1306_HEIGHT
	) {
		/* Error */
		return;
	}
	
	/* Check if pixels are inverted */
	if (SSD1306_2.Inverted) {
		color = (SSD1306_COLOR_t)!color;
	}
	
	/* Set color */
	if (color == SSD1306_COLOR_WHITE) {
		SSD1306_Buffer_1[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
	} else {
		SSD1306_Buffer_1[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
	}
}

void SSD1306_DrawPixel_3(uint16_t x, uint16_t y, SSD1306_COLOR_t color) {
	if (
		x >= SSD1306_WIDTH ||
		y >= SSD1306_HEIGHT
	) {
		/* Error */
		return;
	}
	
	/* Check if pixels are inverted */
	if (SSD1306_3.Inverted) {
		color = (SSD1306_COLOR_t)!color;
	}
	
	/* Set color */
	if (color == SSD1306_COLOR_WHITE) {
		SSD1306_Buffer_2[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
	} else {
		SSD1306_Buffer_2[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
	}
}

void SSD1306_DrawPixel_4(uint16_t x, uint16_t y, SSD1306_COLOR_t color) {
	if (
		x >= SSD1306_WIDTH ||
		y >= SSD1306_HEIGHT
	) {
		/* Error */
		return;
	}
	
	/* Check if pixels are inverted */
	if (SSD1306_4.Inverted) {
		color = (SSD1306_COLOR_t)!color;
	}
	
	/* Set color */
	if (color == SSD1306_COLOR_WHITE) {
		SSD1306_Buffer_2[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
	} else {
		SSD1306_Buffer_2[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
	}
}

void SSD1306_UpdateScreen_My_1(void) {
	uint8_t m;
	
	for (m = 0; m < 8; m++) {
		ssd1306_send_1(0x00,0xB0 + m);
		ssd1306_send_1(0x00,0x00);
		ssd1306_send_1(0x00,0x10);
		
		/* Write multi data */
		ssd1306_Multisend_1(0x40, &SSD1306_Buffer_1[SSD1306_WIDTH * m], SSD1306_WIDTH);
	}
}

void SSD1306_UpdateScreen_My_2(void) {
	uint8_t m;
	
	for (m = 0; m < 8; m++) {
		ssd1306_send_2(0x00,0xB0 + m);
		ssd1306_send_2(0x00,0x00);
		ssd1306_send_2(0x00,0x10);
		
		/* Write multi data */
		ssd1306_Multisend_2(0x40, &SSD1306_Buffer_1[SSD1306_WIDTH * m], SSD1306_WIDTH);
	}
}

void SSD1306_UpdateScreen_My_3(void) {
	uint8_t m;
	
	for (m = 0; m < 8; m++) {
		ssd1306_send_3(0x00,0xB0 + m);
		ssd1306_send_3(0x00,0x00); // Смещение по горизонтали na 2 pix
		ssd1306_send_3(0x00,0x10);
		
		/* Write multi data */
		ssd1306_Multisend_3(0x40, &SSD1306_Buffer_2[(SSD1306_WIDTH) * m], (SSD1306_WIDTH));
	}
}

void SSD1306_UpdateScreen_My_4(void) {
	uint8_t m;
	
	for (m = 0; m < 8; m++) {
		ssd1306_send_4(0x00,0xB0 + m);
		ssd1306_send_4(0x00,0x02); // Смещение по горизонтали na 2 pix
		ssd1306_send_4(0x00,0x10);
		
		/* Write multi data */
		ssd1306_Multisend_4(0x40, &SSD1306_Buffer_2[(SSD1306_WIDTH) * m], (SSD1306_WIDTH));
	}
}

void SSD1306_GotoXY_1(uint16_t x, uint16_t y) {
	/* Set write pointers */
	SSD1306.CurrentX = x;
	SSD1306.CurrentY = y;
}

void SSD1306_GotoXY_2(uint16_t x, uint16_t y) {
	/* Set write pointers */
	SSD1306_2.CurrentX = x;
	SSD1306_2.CurrentY = y;
}

void SSD1306_GotoXY_3(uint16_t x, uint16_t y) {
	/* Set write pointers */
	SSD1306_3.CurrentX = x;
	SSD1306_3.CurrentY = y;
}

void SSD1306_GotoXY_4(uint16_t x, uint16_t y) {
	/* Set write pointers */
	SSD1306_4.CurrentX = x;
	SSD1306_4.CurrentY = y;
}

char SSD1306_Putc_1(char ch, FontDef_t* Font, SSD1306_COLOR_t color) {
	uint32_t i, b, j;
	
	/* Check available space in LCD */
	if (
		SSD1306_WIDTH <= (SSD1306.CurrentX + Font->FontWidth) ||
		SSD1306_HEIGHT <= (SSD1306.CurrentY + Font->FontHeight)
	) {
		/* Error */
		return 0;
	}
	
	/* Go through font */
	for (i = 0; i < Font->FontHeight; i++) {
		b = Font->data[(ch - 32) * Font->FontHeight + i];
		for (j = 0; j < Font->FontWidth; j++) {
			if ((b << j) & 0x8000) {
				SSD1306_DrawPixel_1(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR_t) color);
			} else {
				SSD1306_DrawPixel_1(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR_t)!color);
			}
		}
	}
	
	/* Increase pointer */
	SSD1306.CurrentX += Font->FontWidth;
	
	/* Return character written */
	return ch;
}

char SSD1306_Putc_2(char ch, FontDef_t* Font, SSD1306_COLOR_t color) {
	uint32_t i, b, j;
	
	/* Check available space in LCD */
	if (
		SSD1306_WIDTH <= (SSD1306_2.CurrentX + Font->FontWidth) ||
		SSD1306_HEIGHT <= (SSD1306_2.CurrentY + Font->FontHeight)
	) {
		/* Error */
		return 0;
	}
	
	/* Go through font */
	for (i = 0; i < Font->FontHeight; i++) {
		b = Font->data[(ch - 32) * Font->FontHeight + i];
		for (j = 0; j < Font->FontWidth; j++) {
			if ((b << j) & 0x8000) {
				SSD1306_DrawPixel_2(SSD1306_2.CurrentX + j, (SSD1306_2.CurrentY + i), (SSD1306_COLOR_t) color);
			} else {
				SSD1306_DrawPixel_2(SSD1306_2.CurrentX + j, (SSD1306_2.CurrentY + i), (SSD1306_COLOR_t)!color);
			}
		}
	}
	
	/* Increase pointer */
	SSD1306_2.CurrentX += Font->FontWidth;
	
	/* Return character written */
	return ch;
}

char SSD1306_Putc_3(char ch, FontDef_t* Font, SSD1306_COLOR_t color) {
	uint32_t i, b, j;
	
	/* Check available space in LCD */
	if (
		SSD1306_WIDTH <= (SSD1306_3.CurrentX + Font->FontWidth) ||
		SSD1306_HEIGHT <= (SSD1306_3.CurrentY + Font->FontHeight)
	) {
		/* Error */
		return 0;
	}
	
	/* Go through font */
	for (i = 0; i < Font->FontHeight; i++) {
		b = Font->data[(ch - 32) * Font->FontHeight + i];
		for (j = 0; j < Font->FontWidth; j++) {
			if ((b << j) & 0x8000) {
				SSD1306_DrawPixel_3(SSD1306_3.CurrentX + j, (SSD1306_3.CurrentY + i), (SSD1306_COLOR_t) color);
			} else {
				SSD1306_DrawPixel_3(SSD1306_3.CurrentX + j, (SSD1306_3.CurrentY + i), (SSD1306_COLOR_t)!color);
			}
		}
	}
	
	/* Increase pointer */
	SSD1306_3.CurrentX += Font->FontWidth;
	
	/* Return character written */
	return ch;
}

char SSD1306_Putc_4(char ch, FontDef_t* Font, SSD1306_COLOR_t color) {
	uint32_t i, b, j;
	
	/* Check available space in LCD */
	if (
		SSD1306_WIDTH <= (SSD1306_4.CurrentX + Font->FontWidth) ||
		SSD1306_HEIGHT <= (SSD1306_4.CurrentY + Font->FontHeight)
	) {
		/* Error */
		return 0;
	}
	
	/* Go through font */
	for (i = 0; i < Font->FontHeight; i++) {
		b = Font->data[(ch - 32) * Font->FontHeight + i];
		for (j = 0; j < Font->FontWidth; j++) {
			if ((b << j) & 0x8000) {
				SSD1306_DrawPixel_4(SSD1306_4.CurrentX + j, (SSD1306_4.CurrentY + i), (SSD1306_COLOR_t) color);
			} else {
				SSD1306_DrawPixel_4(SSD1306_4.CurrentX + j, (SSD1306_4.CurrentY + i), (SSD1306_COLOR_t)!color);
			}
		}
	}
	
	/* Increase pointer */
	SSD1306_4.CurrentX += Font->FontWidth;
	
	/* Return character written */
	return ch;
}

char SSD1306_Puts_1(char* str, FontDef_t* Font, SSD1306_COLOR_t color) {
	/* Write characters */
	while (*str) {
		/* Write character by character */
		if (SSD1306_Putc_1(*str, Font, color) != *str) {
			/* Return error */
			return *str;
		}
		
		/* Increase string pointer */
		str++;
	}
	
	/* Everything OK, zero should be returned */
	return *str;
}

char SSD1306_Puts_2(char* str, FontDef_t* Font, SSD1306_COLOR_t color) {
	/* Write characters */
	while (*str) {
		/* Write character by character */
		if (SSD1306_Putc_2(*str, Font, color) != *str) {
			/* Return error */
			return *str;
		}
		
		/* Increase string pointer */
		str++;
	}
	
	/* Everything OK, zero should be returned */
	return *str;
}

char SSD1306_Puts_3(char* str, FontDef_t* Font, SSD1306_COLOR_t color) {
	/* Write characters */
	while (*str) {
		/* Write character by character */
		if (SSD1306_Putc_3(*str, Font, color) != *str) {
			/* Return error */
			return *str;
		}
		
		/* Increase string pointer */
		str++;
	}
	
	/* Everything OK, zero should be returned */
	return *str;
}

char SSD1306_Puts_4(char* str, FontDef_t* Font, SSD1306_COLOR_t color) {
	/* Write characters */
	while (*str) {
		/* Write character by character */
		if (SSD1306_Putc_4(*str, Font, color) != *str) {
			/* Return error */
			return *str;
		}
		
		/* Increase string pointer */
		str++;
	}
	
	/* Everything OK, zero should be returned */
	return *str;
}
  
//void Welcome_Screen(void)
//{

//   SSD1306_GotoXY(5, 15);
//   SSD1306_Puts("VAZ 2107", &Font_11x18, SSD1306_COLOR_BLACK);
//   SSD1306_UpdateScreen_My();
//}

void Init_SSD1306_1(SSD1306_COLOR_t color_init)
{

   ssd1306_send_1(0x00,0xAE); //display off
	ssd1306_send_1(0x00,0x20); //Set Memory Addressing Mode   
	ssd1306_send_1(0x00,0x10); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	ssd1306_send_1(0x00,0xB0); //Set Page Start Address for Page Addressing Mode,0-7
	ssd1306_send_1(0x00,0xC8); //Set COM Output Scan Direction
	ssd1306_send_1(0x00,0x00); //---set low column address
	ssd1306_send_1(0x00,0x10); //---set high column address
	ssd1306_send_1(0x00,0x40); //--set start line address
	ssd1306_send_1(0x00,0x81); //--set contrast control register
	ssd1306_send_1(0x00,0xFF);
	ssd1306_send_1(0x00,0xA1); //--set segment re-map 0 to 127
	ssd1306_send_1(0x00,0xA6); //--set normal display
	ssd1306_send_1(0x00,0xA8); //--set multiplex ratio(1 to 64)
	ssd1306_send_1(0x00,0x3F); //
	ssd1306_send_1(0x00,0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	ssd1306_send_1(0x00,0xD3); //-set display offset
	ssd1306_send_1(0x00,0x00); //-not offset
	ssd1306_send_1(0x00,0xD5); //--set display clock divide ratio/oscillator frequency
	ssd1306_send_1(0x00,0xF0); //--set divide ratio
	ssd1306_send_1(0x00,0xD9); //--set pre-charge period
	ssd1306_send_1(0x00,0x22); //
	ssd1306_send_1(0x00,0xDA); //--set com pins hardware configuration
	ssd1306_send_1(0x00,0x12);
	ssd1306_send_1(0x00,0xDB); //--set vcomh
	ssd1306_send_1(0x00,0x20); //0x20,0.77xVcc
	ssd1306_send_1(0x00,0x8D); //--set DC-DC enable
	ssd1306_send_1(0x00,0x14); //
	ssd1306_send_1(0x00,0xAF); //--turn on SSD1306 panel
   SSD1306.Initialized = 1; 
    /* Clear screen */
	SSD1306_Fill_1(color_init);
	
	/* Update screen */
	SSD1306_UpdateScreen_My_1();
   
}		

void Init_SSD1306_2(SSD1306_COLOR_t color_init)
{

   ssd1306_send_2(0x00,0xAE); //display off
	ssd1306_send_2(0x00,0x20); //Set Memory Addressing Mode   
	ssd1306_send_2(0x00,0x10); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	ssd1306_send_2(0x00,0xB0); //Set Page Start Address for Page Addressing Mode,0-7
	ssd1306_send_2(0x00,0xC8); //Set COM Output Scan Direction
	ssd1306_send_2(0x00,0x00); //---set low column address
	ssd1306_send_2(0x00,0x10); //---set high column address
	ssd1306_send_2(0x00,0x40); //--set start line address
	ssd1306_send_2(0x00,0x81); //--set contrast control register
	ssd1306_send_2(0x00,0xFF);
	ssd1306_send_2(0x00,0xA1); //--set segment re-map 0 to 127
	ssd1306_send_2(0x00,0xA6); //--set normal display
	ssd1306_send_2(0x00,0xA8); //--set multiplex ratio(1 to 64)
	ssd1306_send_2(0x00,0x3F); //
	ssd1306_send_2(0x00,0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	ssd1306_send_2(0x00,0xD3); //-set display offset
	ssd1306_send_2(0x00,0x00); //-not offset
	ssd1306_send_2(0x00,0xD5); //--set display clock divide ratio/oscillator frequency
	ssd1306_send_2(0x00,0xF0); //--set divide ratio
	ssd1306_send_2(0x00,0xD9); //--set pre-charge period
	ssd1306_send_2(0x00,0x22); //
	ssd1306_send_2(0x00,0xDA); //--set com pins hardware configuration
	ssd1306_send_2(0x00,0x12);
	ssd1306_send_2(0x00,0xDB); //--set vcomh
	ssd1306_send_2(0x00,0x20); //0x20,0.77xVcc
	ssd1306_send_2(0x00,0x8D); //--set DC-DC enable
	ssd1306_send_2(0x00,0x14); //
	ssd1306_send_2(0x00,0xAF); //--turn on SSD1306 panel
   SSD1306_2.Initialized = 1; 
    /* Clear screen */
	SSD1306_Fill_2(color_init);
	
	/* Update screen */
	SSD1306_UpdateScreen_My_2();
   
}	

void Init_SSD1306_3(SSD1306_COLOR_t color_init)
{

   ssd1306_send_3(0x00,0xAE); //display off
	ssd1306_send_3(0x00,0x20); //Set Memory Addressing Mode   
	ssd1306_send_3(0x00,0x10); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	ssd1306_send_3(0x00,0xB0); //Set Page Start Address for Page Addressing Mode,0-7
	ssd1306_send_3(0x00,0xC8); //Set COM Output Scan Direction
	ssd1306_send_3(0x00,0x00); //---set low column address
	ssd1306_send_3(0x00,0x10); //---set high column address
	ssd1306_send_3(0x00,0x40); //--set start line address
	ssd1306_send_3(0x00,0x81); //--set contrast control register
	ssd1306_send_3(0x00,0xFF);
	ssd1306_send_3(0x00,0xA1); //A1--set segment re-map 0 to 127
	ssd1306_send_3(0x00,0xA6); //--set normal display
	ssd1306_send_3(0x00,0xA8); //--set multiplex ratio(1 to 64)
	ssd1306_send_3(0x00,0x3F); //
	ssd1306_send_3(0x00,0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	ssd1306_send_3(0x00,0xD3); //-set display offset
	ssd1306_send_3(0x00,0x00); //-not offset
	ssd1306_send_3(0x00,0xD5); //--set display clock divide ratio/oscillator frequency
	ssd1306_send_3(0x00,0xF0); //--set divide ratio
	ssd1306_send_3(0x00,0xD9); //--set pre-charge period
	ssd1306_send_3(0x00,0x22); //
	ssd1306_send_3(0x00,0xDA); //--set com pins hardware configuration
	ssd1306_send_3(0x00,0x12);
	ssd1306_send_3(0x00,0xDB); //--set vcomh
	ssd1306_send_3(0x00,0x20); //0x20,0.77xVcc
	ssd1306_send_3(0x00,0x8D); //--set DC-DC enable
	ssd1306_send_3(0x00,0x14); //
	ssd1306_send_3(0x00,0xAF); //--turn on SSD1306 panel
   SSD1306_3.Initialized = 1; 
    /* Clear screen */
	SSD1306_Fill_3(color_init);
	
	/* Update screen */
	SSD1306_UpdateScreen_My_3();
   
}		
 
void Init_SSD1306_4(SSD1306_COLOR_t color_init)
{

   ssd1306_send_4(0x00,0xAE); //display off
	ssd1306_send_4(0x00,0x20); //Set Memory Addressing Mode   
	ssd1306_send_4(0x00,0x10); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	ssd1306_send_4(0x00,0xB0); //Set Page Start Address for Page Addressing Mode,0-7
	ssd1306_send_4(0x00,0xC8); //Set COM Output Scan Direction
	ssd1306_send_4(0x00,0x00); //---set low column address
	ssd1306_send_4(0x00,0x10); //---set high column address
	ssd1306_send_4(0x00,0x40); //--set start line address
	ssd1306_send_4(0x00,0x81); //--set contrast control register
	ssd1306_send_4(0x00,0xFF);
	ssd1306_send_4(0x00,0xA1); //A1--set segment re-map 0 to 127
	ssd1306_send_4(0x00,0xA6); //--set normal display
	ssd1306_send_4(0x00,0xA8); //--set multiplex ratio(1 to 64)
	ssd1306_send_4(0x00,0x3F); //
	ssd1306_send_4(0x00,0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	ssd1306_send_4(0x00,0xD3); //-set display offset
	ssd1306_send_4(0x00,0x00); //-not offset
	ssd1306_send_4(0x00,0xD5); //--set display clock divide ratio/oscillator frequency
	ssd1306_send_4(0x00,0xF0); //--set divide ratio
	ssd1306_send_4(0x00,0xD9); //--set pre-charge period
	ssd1306_send_4(0x00,0x22); //
	ssd1306_send_4(0x00,0xDA); //--set com pins hardware configuration
	ssd1306_send_4(0x00,0x12);
	ssd1306_send_4(0x00,0xDB); //--set vcomh
	ssd1306_send_4(0x00,0x20); //0x20,0.77xVcc
	ssd1306_send_4(0x00,0x8D); //--set DC-DC enable
	ssd1306_send_4(0x00,0x14); //
	ssd1306_send_4(0x00,0xAF); //--turn on SSD1306 panel
   SSD1306_4.Initialized = 1; 
    /* Clear screen */
	SSD1306_Fill_4(color_init);
	
	/* Update screen */
	SSD1306_UpdateScreen_My_4();
   
}		

