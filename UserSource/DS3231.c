#include "stm32f0xx.h"
#include "DS3231.h"

//// All DS3231 registers
//typedef struct {
//	uint8_t seconds;
//	uint8_t minutes;
//	uint8_t hours;
//	uint8_t day;
//	uint8_t date;
//	uint8_t month;
//	uint8_t year;
//	uint8_t alarm1_secconds;
//	uint8_t alarm1_minutes;
//	uint8_t alarm1_hours;
//	uint8_t alarm1_day;
//	uint8_t alarm1_date;
//	uint8_t alarm2_minutes;
//	uint8_t alarm2_hours;
//	uint8_t alarm2_day;
//	uint8_t alarm2_date;
//	uint8_t control;
//	uint8_t status;
//	uint8_t aging;
//	uint8_t msb_temp;
//	uint8_t lsb_temp;
//} DS3231_registers_TypeDef;

//// DS3231 date
//typedef struct {
//	uint8_t seconds;
//	uint8_t minutes;
//	uint8_t hours;
//	uint8_t day_of_week;
//	uint8_t date;
//	uint8_t month;
//	uint8_t year;
//} DS3231_date_TypeDef;




uint8_t set_time[7];

#define DS3231_I2C_TIMEOUT					1000
//uint16_t I2C_Timout_DS3231 = 0;
//uint8_t Timeout_DS3231 = 0;

//uint8_t buffer[19];

void DS3231_write(uint16_t address, uint8_t data)
{
   
   uint8_t Count=0;	// Счётчик успешно принятых байт
   I2C1->CR2 &= ~I2C_CR2_AUTOEND;
   I2C1->CR2 &= ~I2C_CR2_RD_WRN;			// Режим передачи
	I2C1->CR2 &= ~I2C_CR2_NBYTES;				// Очистить размер данных
	I2C1->CR2 |= ((1+1)<<16);	// Установить размер данных
	I2C1->CR2 &= ~I2C_CR2_SADD;	// Очистить адрес ведомого устройства
	I2C1->CR2 |= (DS3231_addr<<1);			// Установить адрес ведомого устройства
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
		if (I2C1->ISR & I2C_ISR_TXIS) I2C1->TXDR = data; //*(buf++);	// Отправляю данныеCount++
	}
   

//    //стоп
//   I2C1->CR2 |= I2C_CR2_STOP;				// Выдать стоп на шину
//	while (I2C1->ISR & I2C_ISR_BUSY) {};		// Ожидать выдачу стопа
//	// Очищаю флаги - необходимо для дальнейшей работы шины
//	I2C1->ICR |= I2C_ICR_STOPCF;		// STOP флаг
//	I2C1->ICR |= I2C_ICR_NACKCF;		// NACK флаг
//	// Если есть ошибки на шине - очищаю флаги
//	if (I2C1->ISR & (I2C_ISR_ARLO | I2C_ISR_BERR))
//	{
//		I2C1->ICR |= I2C_ICR_ARLOCF;
//		I2C1->ICR |= I2C_ICR_BERRCF;
//	}
   //-------------------------------------
	
//	I2C1->CR2 =  I2C_CR2_AUTOEND | (2<<16) | (DS3231_addr); 
//		
//			/* Check Tx empty */
//	  while (!(I2C1->ISR & I2C_ISR_TXE) );
//	
//	 I2C1->TXDR =(uint8_t) (address); /* Byte to send */
//	 I2C1->CR2 |= I2C_CR2_START; /* Go */
//		
//      //while ((!(I2C1->ISR & I2C_ISR_TXIS) )
//		while ((!(I2C1->ISR & I2C_ISR_TXIS) ) && (Timeout_DS3231 == 0) )
//      {
//         I2C_Timout_DS3231++;
//         if (I2C_Timout_DS3231 == DS3231_I2C_TIMEOUT) {I2C_Timout_DS3231 = 0;Timeout_DS3231 = 1; }
//      };
//      Timeout_DS3231 = 0;
//      I2C_Timout_DS3231 = 0;;
////		I2C1->TXDR = (uint8_t)(address &0x00FF); /* Byte to send */
////	
////		//while (!(I2C1->ISR & I2C_ISR_TXIS) );
////		while ((!(I2C1->ISR & I2C_ISR_TXIS) ) && (Timeout == 0) )
////      {
////         I2C_Timout++;
////         if (I2C_Timout == ssd1306_I2C_TIMEOUT) {I2C_Timout = 0;Timeout = 1; }
////      };
////      Timeout = 0;
////      I2C_Timout = 0;
//      
//		I2C1->TXDR = data ; /* Byte to send */
	 	
}

void DS3231_Init(uint16_t address, uint8_t data)
{
	
//	I2C1->CR2 =  I2C_CR2_AUTOEND | (2<<16) | (DS3231_addr<<1); 
//		
//			/* Check Tx empty */
//	  while (!(I2C1->ISR & I2C_ISR_TXE) );
//	 I2C1->TXDR =(uint8_t)(address);
////	 I2C1->TXDR =(uint8_t)(address &0x00FF);; /* Byte to send */
//	 I2C1->CR2 |= I2C_CR2_START; /* Go */
//		
//      //while ((!(I2C1->ISR & I2C_ISR_TXIS) )
//		while ((!(I2C1->ISR & I2C_ISR_TXIS) ) && (Timeout_DS3231 == 0) )
//      {
//         I2C_Timout_DS3231++;
//         if (I2C_Timout_DS3231 == DS3231_I2C_TIMEOUT) {I2C_Timout_DS3231 = 0;Timeout_DS3231 = 1; }
//      };
//      Timeout_DS3231 = 0;
//      I2C_Timout_DS3231 = 0;;
//		I2C1->TXDR = 0x7c; /* Byte to send */
//      
//		//while (!(I2C1->ISR & I2C_ISR_TXIS) );
//		while ((!(I2C1->ISR & I2C_ISR_TXIS) ) && (Timeout_DS3231 == 0) )
//      {
//         I2C_Timout_DS3231++;
//         if (I2C_Timout_DS3231 == DS3231_I2C_TIMEOUT) {I2C_Timout_DS3231 = 0;Timeout_DS3231 = 1; }
//      };
//      Timeout_DS3231 = 0;
//      I2C_Timout_DS3231 = 0;
//      
////		I2C1->TXDR = 0x08 ; /* Byte to send */
//      
//      //стоп
////   I2C1->CR2 |= I2C_CR2_STOP;				// Выдать стоп на шину
////	while (I2C1->ISR & I2C_ISR_BUSY) {};		// Ожидать выдачу стопа
////	// Очищаю флаги - необходимо для дальнейшей работы шины
////	I2C1->ICR |= I2C_ICR_STOPCF;		// STOP флаг
////	I2C1->ICR |= I2C_ICR_NACKCF;		// NACK флаг
////	// Если есть ошибки на шине - очищаю флаги
////	if (I2C1->ISR & (I2C_ISR_ARLO | I2C_ISR_BERR))
////	{
////		I2C1->ICR |= I2C_ICR_ARLOCF;
////		I2C1->ICR |= I2C_ICR_BERRCF;
////	}
	 	
}

void DS3231_Read(uint16_t address,uint8_t* buf,uint8_t nbytes)
{
   
   uint8_t Count=0;	// Счётчик успешно принятых байт
   I2C1->CR2 &= ~I2C_CR2_AUTOEND;  //авто-СТОП отключаем
   I2C1->CR2 &= ~I2C_CR2_RD_WRN;			// Режим передачи
	I2C1->CR2 &= ~I2C_CR2_NBYTES;				// Очистить размер данных
	I2C1->CR2 |= ((1)<<16);	// Установить размер данных
	I2C1->CR2 &= ~I2C_CR2_SADD;	// Очистить адрес ведомого устройства
	I2C1->CR2 |= (DS3231_addr<<1);			// Установить адрес ведомого устройства
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
	I2C1->CR2 |= (DS3231_addr<<1);			// Установить адрес ведомого устройства
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
   
   
	//---------------------------------
//	uint8_t index=0;
//	I2C1->CR2 =  (1<<16) | (DS3231_addr);//пишем адрес устройства 
////	 while (!(I2C1->ISR & I2C_ISR_TXE) );
//   while ((!(I2C1->ISR & I2C_ISR_TXE) ) && (Timeout_DS3231 == 0) )
//   {
//      I2C_Timout_DS3231++;
//      if (I2C_Timout_DS3231 == DS3231_I2C_TIMEOUT) {I2C_Timout_DS3231 = 0;Timeout_DS3231 = 1; }
//   };
//   Timeout_DS3231 = 0;
//   I2C_Timout_DS3231 = 0;
//   I2C1->CR2 |= I2C_CR2_START; /* Go */
//	I2C1->TXDR = (uint8_t) (address); /* Byte to send (address>>8); */
//	
////		while (!(I2C1->ISR & I2C_ISR_TXIS) );
//   while ((!(I2C1->ISR & I2C_ISR_TC) ) && (Timeout_DS3231 == 0) ) //I2C_ISR_TXIS
//   {
//      I2C_Timout_DS3231++;
//      if (I2C_Timout_DS3231 == DS3231_I2C_TIMEOUT) {I2C_Timout_DS3231 = 0;Timeout_DS3231 = 1; }
//   };
//   Timeout_DS3231 = 0;
//   I2C_Timout_DS3231 = 0;
//	 I2C1->CR2 =  //I2C_CR2_AUTOEND |
//            		 ((nbytes)<<16) | 
//		            (DS3231_addr) |
//		             I2C_CR2_RD_WRN |
//		             I2C_CR2_NACK; 
//	 I2C1->CR2 |= I2C_CR2_START; /* Go */
//		for(index=0;index<=nbytes-1;index++)
//	 { 
////	 while (!(I2C1->ISR & I2C_ISR_RXNE) ){};
//       while ((!(I2C1->ISR & I2C_ISR_RXNE) ) && (Timeout_DS3231 == 0) )
//       {
//          I2C_Timout_DS3231++;
//          if (I2C_Timout_DS3231 == DS3231_I2C_TIMEOUT) {I2C_Timout_DS3231 = 0;Timeout_DS3231 = 1; }
//       };
//       Timeout_DS3231 = 0;
//       I2C_Timout_DS3231 = 0;
//       buf[index]  = I2C1->RXDR ;
//	
//	 }
//    
//    //стоп
//   I2C1->CR2 |= I2C_CR2_STOP;				// Выдать стоп на шину
////	while (I2C1->ISR & I2C_ISR_BUSY) {};		// Ожидать выдачу стопа
//	// Очищаю флаги - необходимо для дальнейшей работы шины
//	I2C1->ICR |= I2C_ICR_STOPCF;		// STOP флаг
//	I2C1->ICR |= I2C_ICR_NACKCF;		// NACK флаг
//	// Если есть ошибки на шине - очищаю флаги
//	if (I2C1->ISR & (I2C_ISR_ARLO | I2C_ISR_BERR))
//	{
//		I2C1->ICR |= I2C_ICR_ARLOCF;
//		I2C1->ICR |= I2C_ICR_BERRCF;
//	}
    
}





void DS3231_WriteDate(uint8_t* buf)
{
   uint8_t temp,i;
   for (i = 0;i <= 6;i++)
   {
      temp = (buf[i]/10) << 4 | (buf[i]%10);
      DS3231_write(i, temp);
   }
}

//void DS3231_ReadDate(uint8_t* buf) { //HRF_date_TypeDef* hrf_date
////	DS3231_date_TypeDef raw_date;
////   DS3231_registers_TypeDef raw_date;
//   DS3231_Read(0,buf,19);
////	DS3231_ReadDateRAW(&raw_date);

//	date.Seconds = (buffer[0] >> 4) * 10 + (buffer[0] & 0x0f);
//	date.Minutes = (buffer[1] >> 4) * 10 + (buffer[1] & 0x0f);
//   date.Hours   = (buffer[2] >> 4) * 10 + (buffer[2] & 0x0f);
//	date.DOW     = (buffer[3] >> 4) * 10 + (buffer[3] & 0x0f);
//   date.Day     = (buffer[4] >> 4) * 10 + (buffer[4] & 0x0f);
//	date.Month   = (buffer[5] >> 4) * 10 + (buffer[5] & 0x0f);
//	date.Year    = (buffer[6] >> 4) * 10 + (buffer[6] & 0x0f) + 2000;
////	hrf_date->Hours   = (raw_date.hours   >> 4) * 10 + (raw_date.hours   & 0x0f);
////	hrf_date->Day     = (raw_date.date    >> 4) * 10 + (raw_date.date    & 0x0f);
////	hrf_date->Month   = (raw_date.month   >> 4) * 10 + (raw_date.month   & 0x0f);
////	hrf_date->Year    = (raw_date.year    >> 4) * 10 + (raw_date.year    & 0x0f) + 2000;
////	hrf_date->DOW     = raw_date.day_of_week;
//}

void DS3231_ReadDate1(HRF_date_TypeDef* date1,uint8_t* buf) { //HRF_date_TypeDef* hrf_date
//	DS3231_date_TypeDef raw_date;
//   DS3231_registers_TypeDef raw_date;
   DS3231_Read(0,buf,19);
//	DS3231_ReadDateRAW(&raw_date);

	date1->Seconds = (buffer[0] >> 4) * 10 + (buffer[0] & 0x0f);
	date1->Minutes = (buffer[1] >> 4) * 10 + (buffer[1] & 0x0f);
   date1->Hours   = (buffer[2] >> 4) * 10 + (buffer[2] & 0x0f);
	date1->DOW     = (buffer[3] >> 4) * 10 + (buffer[3] & 0x0f);
   date1->Day     = (buffer[4] >> 4) * 10 + (buffer[4] & 0x0f);
	date1->Month   = (buffer[5] >> 4) * 10 + (buffer[5] & 0x0f);
	date1->Year    = (buffer[6] >> 4) * 10 + (buffer[6] & 0x0f) + 2000;
//	hrf_date->Hours   = (raw_date.hours   >> 4) * 10 + (raw_date.hours   & 0x0f);
//	hrf_date->Day     = (raw_date.date    >> 4) * 10 + (raw_date.date    & 0x0f);
//	hrf_date->Month   = (raw_date.month   >> 4) * 10 + (raw_date.month   & 0x0f);
//	hrf_date->Year    = (raw_date.year    >> 4) * 10 + (raw_date.year    & 0x0f) + 2000;
//	hrf_date->DOW     = raw_date.day_of_week;
}


