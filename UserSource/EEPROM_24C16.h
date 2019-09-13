#ifndef EEPROM_24C16_H
#define EEPROM_24C16_H

#include "stm32f0xx.h" 


void Delay_ms(uint32_t ms);
void Delay_us(uint32_t us);
void eeprom_pagewrite(uint8_t address, uint8_t *buf,uint8_t nbytes);
void eeprom_pageread(uint8_t address,uint8_t *buf,uint8_t nbytes);
void EEPROM_float_write(uint8_t addr, float val); // çàïèñü â ÅÅÏĞÎÌ
float EEPROM_float_read(uint8_t addr); // ÷òåíèå èç ÅÅÏĞÎÌ
void EEPROM_16bit_write(uint8_t addr, uint16_t val);
uint16_t EEPROM_16bit_read(uint8_t addr);
void EEPROM_8bit_write(uint8_t addr, uint8_t val);
uint8_t EEPROM_8bit_read(uint8_t addr);


#endif

