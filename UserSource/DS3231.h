#ifndef DS3231_H
#define DS3231_H

#include "stm32f0xx.h" 

uint8_t buffer[19];

#define DS3231_addr     0xD0 // I2C 7-bit slave address shifted for 1 bit to the left
#define DS3231_seconds  0x00 // DS3231 seconds address
#define DS3231_minutes  0x01
#define DS3231_hours    0x02
#define DS3231_day      0x03
#define DS3231_date     0x04
#define DS3231_month    0x05
#define DS3231_year     0x06
#define DS3231_control  0x0E // DS3231 control register address
#define DS3231_tmp_MSB  0x11 // DS3231 temperature MSB

// Human Readable Format date
typedef struct {
	uint8_t  Seconds;
	uint8_t  Minutes;
	uint8_t  Hours;
	uint8_t  Day;
	uint8_t  Month;
	uint16_t Year;
	uint8_t  DOW;
} HRF_date_TypeDef;

HRF_date_TypeDef date,date1;



void DS3231_write(uint16_t address, uint8_t data);
void DS3231_Init(uint16_t address, uint8_t data);
void DS3231_Read(uint16_t address,uint8_t* buf,uint8_t nbytes);
void DS3231_WriteDate(uint8_t* buf);
void DS3231_ReadDate(uint8_t* buf);
void DS3231_ReadDate1(HRF_date_TypeDef* date1,uint8_t* buf);


#endif
