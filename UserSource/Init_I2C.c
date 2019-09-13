#include "Init_I2C.h"
#include "stm32f0xx.h"
#include "stm32f0xx_gpio.h"

    


void Init_I2C1(void)
{
   
      RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
		
		RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
		RCC->CFGR3 |= RCC_CFGR3_I2C1SW; 
		
		//GPIOA->AFR[0] |= (1<<(4*0)) |(1<<(4*1));
		//это для 48 пин 
//      GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_5);//4
//      GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_5);
//		GPIOB->MODER &= ~(GPIO_MODER_MODER6 |GPIO_MODER_MODER7);
//		GPIOB->MODER |= (GPIO_MODER_MODER6_1 |GPIO_MODER_MODER7_1);
//		GPIOB->OTYPER |=(GPIO_OTYPER_OT_6 |GPIO_OTYPER_OT_7);

		//инит портов 20 pin
      GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_4);//5 для 48 пин
      GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_4);
		GPIOA->MODER &= ~(GPIO_MODER_MODER9 |GPIO_MODER_MODER10);
		GPIOA->MODER |= (GPIO_MODER_MODER9_1 |GPIO_MODER_MODER10_1);
		GPIOA->OTYPER |=(GPIO_OTYPER_OT_9 |GPIO_OTYPER_OT_10);
   
   //инит портов 8 pin
//   GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_5);//4
//   GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_5);
//	GPIOB->MODER &= ~(GPIO_MODER_MODER6 |GPIO_MODER_MODER7);
//	GPIOB->MODER |= (GPIO_MODER_MODER6_1 |GPIO_MODER_MODER7_1);
//	GPIOB->OTYPER |=(GPIO_OTYPER_OT_6 |GPIO_OTYPER_OT_7);
   
	I2C1->TIMINGR = (uint32_t)0x00B01A3B;//0x00901A53;//было - 0x00B01A3B;//0x00B01A4B; - 400   0x00700701 - 1.24 MHz
	I2C1->CR1 = I2C_CR1_PE; 
   

   
}

void Init_I2C2(void)
{
   
      RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
		
		RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
//		RCC->CFGR3 |= RCC_CFGR3_I2C1SW; 
		
		//GPIOA->AFR[0] |= (1<<(4*0)) |(1<<(4*1));
		//это для 48 пин 
      GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_1);//4
      GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_1);
		GPIOB->MODER &= ~(GPIO_MODER_MODER10 |GPIO_MODER_MODER11);
		GPIOB->MODER |= (GPIO_MODER_MODER10_1 |GPIO_MODER_MODER11_1);
		GPIOB->OTYPER |=(GPIO_OTYPER_OT_10 |GPIO_OTYPER_OT_11);

		//инит портов 20 pin
//      GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_4);//5 для 48 пин
//      GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_4);
//		GPIOA->MODER &= ~(GPIO_MODER_MODER9 |GPIO_MODER_MODER10);
//		GPIOA->MODER |= (GPIO_MODER_MODER9_1 |GPIO_MODER_MODER10_1);
//		GPIOA->OTYPER |=(GPIO_OTYPER_OT_9 |GPIO_OTYPER_OT_10);
   
   
   
	I2C2->TIMINGR = (uint32_t)0x00B01A3B;//0x00901A53;//было - 0x00B01A3B;//0x00B01A4B; - 400   0x00700701 - 1.24 MHz
	I2C2->CR1 = I2C_CR1_PE; 
   

   
}

