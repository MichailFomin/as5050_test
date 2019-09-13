#include "usart.h"
extern uint8_t rx_buff;


 
uint8_t RxBuffer[RXSIZE];
fifo_t RxFifo[1];
 

 
uint8_t TxBuffer[TXSIZE];
fifo_t TxFifo[1];


uint8_t ReadStat;
uint8_t ReadCount;

//This initializes the FIFO structure with the given buffer and size
void fifo_init(fifo_t * f, char * buf, int size){
     f->head = 0;
     f->tail = 0;
     f->size = size;
     f->buf = buf;
	   f->count=0;
}
 
//This reads nbytes bytes from the FIFO
//The number of bytes read is returned
int fifo_read(fifo_t * f, void * buf, int nbytes){
     int i;
     char * p;
     p = buf;
     for(i=0; i < nbytes; i++){
          if( f->tail != f->head ){ //see if any data is available
               *p++ = f->buf[f->tail];  //grab a byte from the buffer
               f->tail++;  //increment the tail
               if( f->tail == f->size ){  //check for wrap-around
                    f->tail = 0;
               }
          } else {
               return i; //number of bytes read 
          }
     }
     return nbytes;
}
 
//This writes up to nbytes bytes to the FIFO
//If the head runs in to the tail, not all bytes are written
//The number of bytes written is returned

int fifo_write(fifo_t * f, const void * buf, int nbytes){
     int i;
     const char * p;
     p = buf;
     for(i=0; i < nbytes; i++){
           //first check to see if there is space in the buffer
           if( (f->head + 1 == f->tail) ||( (f->head + 1 == f->size) && (f->tail == 0) ))
						 {
                 return i; //no more room
           } else {
               f->buf[f->head] = *p++;
               f->head++;  //increment the head
               if( f->head == f->size ){  //check for wrap-around
                    f->head = 0;
               }
           }
     }
     return nbytes;
}


void fifo_copy_buf (char *buf)
{
	ReadCount=fifo_read(RxFifo, buf, RXSIZE-1);
	buf[ReadCount-1]=0;
}










void Usart_init (void) 
    {
      RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
      RCC->APB2ENR|=RCC_APB2ENR_USART1EN;
      
      fifo_init(RxFifo, (void *)RxBuffer, sizeof(RxBuffer));
      fifo_init(TxFifo, (void *)TxBuffer, sizeof(TxBuffer));
	  	                               ;
      
      GPIOA->MODER &= ~(GPIO_MODER_MODER1 | GPIO_MODER_MODER10 |GPIO_MODER_MODER9);
      GPIOA->MODER |=  GPIO_MODER_MODER1_0 | GPIO_MODER_MODER10_1 |GPIO_MODER_MODER9_1; 
    
      GPIOA->AFR[1] |=(1<<(4*1)) |(1<<(4*2));
      USART1->BRR =(APBCLK+BAUDRATE/2)/BAUDRATE; //usart
      USART1->CR1 |= USART_CR1_TE |USART_CR1_RE|USART_CR1_RXNEIE; // USART1
      USART1->CR1 |= USART_CR1_UE; //   USART1  
			
      NVIC_SetPriority(USART1_IRQn, 0); 
      NVIC_EnableIRQ(USART1_IRQn); 
          
    }
    
 void Usart_Transmit(uint8_t Data)
{
  //while(!(USART1->ISR & USART_ISR_TC));
	 while(!(USART1->ISR & USART_ISR_TXE));
  USART1->TDR = Data;
}

void USART1_IRQHandler(void)
{
	uint16_t rx;
	if(USART1->ISR & USART_ISR_RXNE)
	{

 
    rx = (uint8_t)(USART1->RDR); // Receive data, clear flag 
 
    fifo_write(RxFifo, &rx, 1); // Place in reception fifo
		 if(rx==0x0D) 
		 {
			ReadStat= 1;
		 }
	}
}


void USART1_str (const char * data)
{
//	char c;
//	while((c=*data ++)){ Usart_Transmit (c);}
	
	while((*data )){ 
	  Usart_Transmit (*data);
		data++;
	  }
}

