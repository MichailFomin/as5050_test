#ifndef Init_SSD1306_I2C_H
#define Init_SSD1306_I2C_H

#include "stm32f0xx.h" 
#include "stm32f0xx_gpio.h"
#include "fonts.h"
#include "string.h"

//uint16_t I2C_Timout = 0;
//uint8_t Timeout = 0;






/* Private SSD1306 structure */
typedef struct {
	uint16_t CurrentX;
	uint16_t CurrentY;
	uint8_t Inverted;
	uint8_t Initialized;
} SSD1306_t;

/* Private variable */
static SSD1306_t SSD1306;
static SSD1306_t SSD1306_2;
static SSD1306_t SSD1306_3;
static SSD1306_t SSD1306_4;

//static uint32_t ssd1306_I2C_Timeout;
//static uint32_t ssd1306_I2C_Timeout_2;
//static uint32_t ssd1306_I2C_Timeout_3;
//static uint32_t ssd1306_I2C_Timeout_4;

typedef enum {
	SSD1306_COLOR_BLACK = 0x00, /*!< Black color, no pixel */
	SSD1306_COLOR_WHITE = 0x01  /*!< Pixel is set. Color depends on LCD */
} SSD1306_COLOR_t;



void SSD1306_Fill_1(SSD1306_COLOR_t color);
void SSD1306_Fill_2(SSD1306_COLOR_t color);
void SSD1306_Fill_3(SSD1306_COLOR_t color);
void SSD1306_Fill_4(SSD1306_COLOR_t color);
void ssd1306_send_1(uint8_t control_byte, uint8_t data);
void ssd1306_send_2(uint8_t control_byte, uint8_t data);
void ssd1306_send_3(uint8_t control_byte, uint8_t data);
void ssd1306_send_4(uint8_t control_byte, uint8_t data);
void ssd1306_Multisend_1(uint8_t control_byte, uint8_t* data, uint16_t count);
void ssd1306_Multisend_2(uint8_t control_byte, uint8_t* data, uint16_t count);
void ssd1306_Multisend_3(uint8_t control_byte, uint8_t* data, uint16_t count);
void ssd1306_Multisend_4(uint8_t control_byte, uint8_t* data, uint16_t count);
void SSD1306_ToggleInvert_1(void);
void SSD1306_DrawPixel_1(uint16_t x, uint16_t y, SSD1306_COLOR_t color) ;
void SSD1306_DrawPixel_2(uint16_t x, uint16_t y, SSD1306_COLOR_t color) ;
void SSD1306_DrawPixel_3(uint16_t x, uint16_t y, SSD1306_COLOR_t color) ;
void SSD1306_DrawPixel_4(uint16_t x, uint16_t y, SSD1306_COLOR_t color) ;
void SSD1306_UpdateScreen_My_1(void);
void SSD1306_UpdateScreen_My_2(void);
void SSD1306_UpdateScreen_My_3(void);
void SSD1306_UpdateScreen_My_4(void);
void SSD1306_GotoXY_1(uint16_t x, uint16_t y);
void SSD1306_GotoXY_2(uint16_t x, uint16_t y);
void SSD1306_GotoXY_3(uint16_t x, uint16_t y);
void SSD1306_GotoXY_4(uint16_t x, uint16_t y);
char SSD1306_Putc_1(char ch, FontDef_t* Font, SSD1306_COLOR_t color);
char SSD1306_Putc_2(char ch, FontDef_t* Font, SSD1306_COLOR_t color);
char SSD1306_Putc_3(char ch, FontDef_t* Font, SSD1306_COLOR_t color);
char SSD1306_Putc_4(char ch, FontDef_t* Font, SSD1306_COLOR_t color);
char SSD1306_Puts_1(char* str, FontDef_t* Font, SSD1306_COLOR_t color);
char SSD1306_Puts_2(char* str, FontDef_t* Font, SSD1306_COLOR_t color);
char SSD1306_Puts_3(char* str, FontDef_t* Font, SSD1306_COLOR_t color);
char SSD1306_Puts_4(char* str, FontDef_t* Font, SSD1306_COLOR_t color);
void Welcome_Screen_1(void);
void Init_SSD1306_1(SSD1306_COLOR_t color_init);
void Init_SSD1306_2(SSD1306_COLOR_t color_init);
void Init_SSD1306_3(SSD1306_COLOR_t color_init);
void Init_SSD1306_4(SSD1306_COLOR_t color_init);
 







#endif

