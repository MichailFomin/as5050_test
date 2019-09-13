#ifndef Init_SSD1306_SPI_H
#define Init_SSD1306_SPI_H

#include "stm32f0xx.h" 
#include "stm32f0xx_gpio.h"
#include "fonts.h"

#define LCDWIDTH                  	130
#define LCDHEIGHT                 	64

#define CS_ON     GPIO_ResetBits(GPIOA, GPIO_Pin_15)
#define CS_OFF    GPIO_SetBits(GPIOA, GPIO_Pin_15)

//Reset Slave - pin PB0
#define Reset_high		GPIO_SetBits(GPIOB, GPIO_Pin_0)		// Set bit 0, Normal Working
#define Reset_low		   GPIO_ResetBits(GPIOB, GPIO_Pin_0)		// Reset bit 0, Reset Slave

//Command/Data select - pin PB1
//#define wr_Data			GPIO_SetBits(GPIOF, GPIO_Pin_7)		// Set bit 1, Write Data
//#define wr_Command		GPIO_ResetBits(GPIOF, GPIO_Pin_7)		// Reset bit 1, Write Command
#define wr_Data			GPIO_SetBits(GPIOB, GPIO_Pin_4)		// Set bit 1, Write Data
#define wr_Command		GPIO_ResetBits(GPIOB, GPIO_Pin_4)		// Reset bit 1, Write Command

   #define DC_On    GPIOB->BSRR   =  GPIO_BSRR_BS_4
    #define DC_Off   GPIOB->BSRR   =  GPIO_BSRR_BR_4
    #define CS_ENABLE GPIOA->BSRR = GPIO_BSRR_BR_15
    #define CS_DISABLE GPIOA->BSRR = GPIO_BSRR_BS_15


/* Private SSD1306 structure */
typedef struct {
	uint16_t CurrentX;
	uint16_t CurrentY;
	uint8_t Inverted;
	uint8_t Initialized;
} SSD1306_SPI_t;

/* Private variable */
static SSD1306_SPI_t SSD1306_SPI;

typedef enum {
	SSD1306_SPI_COLOR_BLACK = 0x00, /*!< Black color, no pixel */
	SSD1306_SPI_COLOR_WHITE = 0x01  /*!< Pixel is set. Color depends on LCD */
} SSD1306_SPI_COLOR_t;


void LCD_Reset(void);
void send_command(uint8_t byte_to_send);
void send_data(uint8_t byte_to_send);
void SSD1306_SPI_Fill(SSD1306_SPI_COLOR_t color);
void SSD1306_DrawPixel_SPI(uint16_t x, uint16_t y, SSD1306_SPI_COLOR_t color);
void SSD1306_GotoXY_SPI(uint16_t x, uint16_t y);
char SSD1306_Putc_SPI(char ch, FontDef_t* Font, SSD1306_SPI_COLOR_t color);
char SSD1306_Puts_SPI(char* str, FontDef_t* Font, SSD1306_SPI_COLOR_t color);
void 	ssd1306_SPI_Multisend(uint8_t control_byte, uint8_t* data, uint16_t count);
void SSD1306_SPI_UpdateScreen_My(void);
void show_logo(uint8_t offset);

void Fill_RAM(uint8_t Data);
void LCD_Goto(uint8_t x, uint8_t y);

void LCD_init(void);

#endif

