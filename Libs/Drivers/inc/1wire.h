#ifndef _1WIRE_H
#define _1WIRE_H 

#include "stm32f0xx.h"


#define OW_SEND_RESET		1
#define OW_NO_RESET		2

// ?????? ???????? ???????
#define OW_OK			1
#define OW_ERROR		2
#define OW_NO_DEVICE	3

#define OW_NO_READ		0xff

#define OW_READ_SLOT	0xff



 
#define APB1CLK 48000000UL
#define BAUD115200 115200UL //90000UL //для бит но влияет и на датчик температуры
#define BAUD9600 9600UL //это для Reset и presence
void OW_Init(void);
uint8_t OW_Reset(void);
uint8_t OW_Send(uint8_t sendReset, char *command, uint8_t cLen, uint8_t *data, uint8_t dLen, uint8_t readStart);
uint8_t OW_Scan(uint8_t *buf, uint8_t num);

#endif //1WIRE_H
