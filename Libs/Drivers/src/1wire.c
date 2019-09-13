#include "1wire.h"


// Буфер для приема/передачи по 1-wire
uint8_t ow_buf[8];

#define OW_0	0x00
#define OW_1	0xff
#define OW_R_1	0xff

//-----------------------------------------------------------------------------
// функция преобразует один байт в восемь, для передачи через USART
// ow_byte - байт, который надо преобразовать
// ow_bits - ссылка на буфер, размером не менее 8 байт
//-----------------------------------------------------------------------------
void OW_toBits(uint8_t ow_byte, uint8_t *ow_bits) {
	uint8_t i;
	for (i = 0; i < 8; i++) {
		if (ow_byte & 0x01) {
			*ow_bits = OW_1;
		} else {
			*ow_bits = OW_0;
		}
		ow_bits++;
		ow_byte = ow_byte >> 1;
	}
}

//-----------------------------------------------------------------------------
// обратное преобразование - из того, что получено через USART опять собирается байт
// ow_bits - ссылка на буфер, размером не менее 8 байт
//-----------------------------------------------------------------------------
uint8_t OW_toByte(uint8_t *ow_bits) {
	uint8_t ow_byte, i;
	ow_byte = 0;
	for (i = 0; i < 8; i++) {
		ow_byte = ow_byte >> 1;
		if (*ow_bits == OW_R_1) {
			ow_byte |= 0x80;
		}
		ow_bits++;
	}

	return ow_byte;
}






//-----------------------------------------------------------------------------
// инициализиция USART и DMA
//-----------------------------------------------------------------------------
void OW_Init(void)
    {
			//USART1 TX A9

      RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
      RCC->APB2ENR|= RCC_APB2ENR_USART1EN;
    
      
      GPIOA->MODER &= ~(GPIO_MODER_MODER9 );
      GPIOA->MODER |=   GPIO_MODER_MODER9_1 ; 
      GPIOA->OTYPER |= GPIO_OTYPER_OT_9; 
//      GPIOB->AFR[1] |=(4<<(4*2)) ; // B10
			GPIOA->AFR[1] |=(1<<(4*1)) ; // A9   AF1 << сдвиг на 4*1 бита влево
			
      USART1->BRR =(APB1CLK+BAUD115200/2)/BAUD115200; //
      USART1->CR1 |= USART_CR1_TE |USART_CR1_RE; // 
			USART1->CR3 |=USART_CR3_HDSEL;
      //USART2->CR1 |= USART_CR1_UE; //  
			
			RCC->AHBENR |=RCC_AHBENR_DMA1EN;       //
			DMA1_Channel3->CPAR = (uint32_t) &(USART1->RDR);//RX DMA1_Channel6
			DMA1_Channel3->CMAR = (uint32_t) ow_buf;        //
			DMA1_Channel3->CNDTR = 0x08;                //

			DMA1_Channel3->CCR=0;
			DMA1_Channel3->CCR  = 
		                     
													DMA_CCR_MINC
													//DMA_CCR_CIRC|
													//DMA_CCR_DIR
                          ;
		
			DMA1_Channel2->CPAR = (uint32_t)&(USART1->TDR); // TX DMA1_Channel7
			DMA1_Channel2->CMAR = (uint32_t) ow_buf;        //
			DMA1_Channel2->CNDTR = 0x08;                //

			DMA1_Channel2->CCR=0;
			DMA1_Channel2->CCR  = 
													 
													DMA_CCR_MINC|
													//DMA_CCR_CIRC|
													DMA_CCR_DIR
													 ;
													

    }
//-----------------------------------------------------------------------------
// осуществляет сброс и проверку на наличие устройств на шине
//-----------------------------------------------------------------------------		
	uint8_t OW_Reset() {
	uint8_t ow_presence;
		
		USART1->CR1 &= ~USART_CR1_UE; //   USART1  	
		USART1->BRR =(APB1CLK+BAUD9600/2)/BAUD9600; //usart2
		
		USART1->CR1 |= USART_CR1_UE; //   USART1  
			
			
		USART1->ICR|=USART_ICR_TCCF;	
		USART1->TDR= 0xf0;
		
			
		 while(!(USART1->ISR & USART_ISR_TC));
		ow_presence = USART1->RDR;
			
			
		USART1->CR1 &= ~USART_CR1_UE; //   USART1  	
		USART1->BRR =(APB1CLK+BAUD115200/2)/BAUD115200; //usart2
		USART1->CR1 |= USART_CR1_UE; //   USART1  	
	
	if (ow_presence != 0xf0) {
		return OW_OK;
	  }

	return OW_NO_DEVICE;
   }
	
	 
	 
//-----------------------------------------------------------------------------
// процедура общения с шиной 1-wire
// sendReset - посылать RESET в начале общения.
// 		OW_SEND_RESET или OW_NO_RESET
// command - массив байт, отсылаемых в шину. Если нужно чтение - отправляем OW_READ_SLOTH
// cLen - длина буфера команд, столько байт отошлется в шину
// data - если требуется чтение, то ссылка на буфер для чтения
// dLen - длина буфера для чтения. Прочитается не более этой длины
// readStart - с какого символа передачи начинать чтение (нумеруются с 0)
//		можно указать OW_NO_READ, тогда можно не задавать data и dLen
//-----------------------------------------------------------------------------
	 
uint8_t OW_Send(uint8_t sendReset, char *command, uint8_t cLen,
		uint8_t *data, uint8_t dLen, uint8_t readStart) {

	
	if (sendReset == OW_SEND_RESET) {
		if (OW_Reset() == OW_NO_DEVICE) {
			return OW_NO_DEVICE;
		}
	}

	while (cLen > 0) {

		OW_toBits(*command, ow_buf);
		command++;
		cLen--;

    DMA1_Channel3->CNDTR = 0x08; 
		DMA1_Channel2->CNDTR = 0x08; 
		
		DMA1_Channel3->CCR  |=  DMA_CCR_EN; 
    DMA1_Channel2->CCR  |=  DMA_CCR_EN;   
		USART1->CR3|=USART_CR3_DMAR | USART_CR3_DMAT;
		
	
    while(!(DMA1->ISR & DMA_ISR_TCIF3)){};//3
		DMA1->IFCR |= DMA_IFCR_CTCIF3;
		
		DMA1_Channel3->CCR  &=  ~DMA_CCR_EN; 
    DMA1_Channel2->CCR  &=  ~DMA_CCR_EN;   
		USART1->CR3&= ~(USART_CR3_DMAR | USART_CR3_DMAT);
		

		// 
		if (readStart == 0 && dLen > 0) {
			*data = OW_toByte(ow_buf);
			data++;
			dLen--;
		} else {
			if (readStart != OW_NO_READ) {
				readStart--;
			}
		}
	}

	return OW_OK;
}

// внутренняя процедура. Записывает указанное число бит
void OW_SendBits(uint8_t num_bits) {
	 DMA1_Channel3->CPAR = (uint32_t) &(USART1->RDR);
			DMA1_Channel3->CMAR = (uint32_t) ow_buf;        //
			DMA1_Channel3->CNDTR = num_bits;                //

			DMA1_Channel3->CCR=0;
			DMA1_Channel3->CCR  = 
		                     
						DMA_CCR_MINC
						//DMA_CCR_CIRC|
						//DMA_CCR_DIR
                          ;
		
			DMA1_Channel2->CPAR = (uint32_t)&(USART1->TDR);
			DMA1_Channel2->CMAR = (uint32_t) ow_buf;        //
			DMA1_Channel2->CNDTR = num_bits;                //

			DMA1_Channel2->CCR=0;
			DMA1_Channel2->CCR  = 
													 
						DMA_CCR_MINC|
						//DMA_CCR_CIRC|
						DMA_CCR_DIR;

    DMA1_Channel3->CCR  |=  DMA_CCR_EN; 
    DMA1_Channel2->CCR  |=  DMA_CCR_EN;   
	  USART1->CR3|=USART_CR3_DMAR | USART_CR3_DMAT;
		
	
    while(!(DMA1->ISR & DMA_ISR_TCIF3)){};
		DMA1->IFCR |= DMA_IFCR_CTCIF3;
		
	  DMA1_Channel3->CCR  &=  ~DMA_CCR_EN; 
    DMA1_Channel2->CCR  &=  ~DMA_CCR_EN;   
	  USART1->CR3&= ~(USART_CR3_DMAR | USART_CR3_DMAT);

}

//-----------------------------------------------------------------------------
// Данная функция осуществляет сканирование сети 1-wire и записывает найденные
//   ID устройств в массив buf, по 8 байт на каждое устройство.
// переменная num ограничивает количество находимых устройств, чтобы не переполнить
// буфер.
//-----------------------------------------------------------------------------
uint8_t OW_Scan(uint8_t *buf, uint8_t num) {

        uint8_t found = 0;
        uint8_t *lastDevice;
        uint8_t *curDevice = buf;
        uint8_t numBit, lastCollision, currentCollision, currentSelection;

        lastCollision = 0;
        while (found < num) {
                numBit = 1;
                currentCollision = 0;

                // посылаем команду на поиск устройств
                OW_Send(OW_SEND_RESET, (uint8_t*)"\xf0", 1, 0, 0, OW_NO_READ);

                for (numBit = 1; numBit <= 64; numBit++) {
                        // читаем два бита. Основной и комплементарный
                        OW_toBits(OW_READ_SLOT, ow_buf);
                        OW_SendBits(2);

                        if (ow_buf[0] == OW_R_1) {
                                if (ow_buf[1] == OW_R_1) {
                                        // две единицы, где-то провтыкали и заканчиваем поиск
                                        return found;
                                } else {
                                        // 10 - на данном этапе только 1
                                        currentSelection = 1;
                                }
                        } else {
                                if (ow_buf[1] == OW_R_1) {
                                        // 01 - на данном этапе только 0
                                        currentSelection = 0;
                                } else {
                                        // 00 - коллизия
                                        if (numBit < lastCollision) {
                                                // идем по дереву, не дошли до развилки
                                                if (lastDevice[(numBit - 1) >> 3]
                                                                & 1 << ((numBit - 1) & 0x07)) {
                                                        // (numBit-1)>>3 - номер байта
                                                        // (numBit-1)&0x07 - номер бита в байте
                                                        currentSelection = 1;

                                                        // если пошли по правой ветке, запоминаем номер бита
                                                        if (currentCollision < numBit) {
                                                                currentCollision = numBit;
                                                        }
                                                } else {
                                                        currentSelection = 0;
                                                }
                                        } else {
                                                if (numBit == lastCollision) {
                                                        currentSelection = 0;
                                                } else {
                                                        // идем по правой ветке
                                                        currentSelection = 1;

                                                        // если пошли по правой ветке, запоминаем номер бита
                                                        if (currentCollision < numBit) {
                                                                currentCollision = numBit;
                                                        }
                                                }
                                        }
                                }
                        }

                        if (currentSelection == 1) {
                                curDevice[(numBit - 1) >> 3] |= 1 << ((numBit - 1) & 0x07);
                                OW_toBits(0x01, ow_buf);
                        } else {
                                curDevice[(numBit - 1) >> 3] &= ~(1 << ((numBit - 1) & 0x07));
                                OW_toBits(0x00, ow_buf);
                        }
                        OW_SendBits(1);
                }
                found++;
                lastDevice = curDevice;
                curDevice += 8;
                if (currentCollision == 0)
                        return found;

                lastCollision = currentCollision;
        }

        return found;
}


