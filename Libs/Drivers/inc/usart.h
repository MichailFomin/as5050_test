#ifndef _USART_H
#define _USART_H 

#include "stm32f0xx.h"

 
#define APBCLK 48000000UL
#define BAUDRATE 115200UL
#define RXSIZE 64
#define TXSIZE 64


typedef struct {
     char * buf;
     int head;
     int tail;
     int size;
	   int count;
} fifo_t;
 
 
void Usart_init (void); 
void Usart_Transmit(uint8_t );
void USART1_str (const char * );
void fifo_copy_buf (char *);
#endif 
