/*
   PC13 - SPEED_BK_ON/OFF OUT
   PC14 - TAH_BK_ON/OFF OUT
   PC15 - Menu IN
   PF0 - BT+ IN
   PF1 - BT- IN
   PA0 - ADC0 TEMPER
   PA1 - ADC1 PRESURE
   
   PA4 - ADC4 BENZ
   PA5 - ADC5 AMP
   PA6 - ADC6 VOLT
   PA7 - ADC7 LUMENS
   PB0 - PWM1 TIM3_CH3 подсветка приборки
   PB1 - PWM2 TIM3_CH4 подсветка стрелок
   PB2 - Gabarit IN
   PB10 - SCL2
   PB11 - SDA2
   
   PB12 - DOOR IN
   PB13 - MOT_ON/OFF OUT
   PB14 - SPEED IN TIM15_CH1
   PB15 - TAH IN TIM15_CH2
   PA8 - TIM1_CH1 PWM3_Salon
   PA9 - UART_TX DS18B20
   PA12 - CS2
   
   PA15 - CS1
   PB3 - SPI1_SCK
   PB4 - SPI1_DC OUT
   PB5 - SPI1_MOSI
   PB6 - SCL1
   PB7 - SDA1
   PB8 - TAH_PWM OUT TIM16_CH1
   PB9 - SPEED_PWM OUT TIM17_CH1

*/

#include "stm32f0xx.h" 
#include "Init_I2C.h"
#include "SSD1306_I2C.h"
#include "EEPROM_24C16.h"
#include "Init_SPI.h"
#include "SSD1306_SPI.h"
//#include "DS3231.h"
#include "EEPROM_24C16.h"
#include <stdio.h>
#include "stdlib.h"
#include "string.h"
#include "fonts.h"
#include <math.h>
#include "stm32f0xx_flash.h"
#include "stm32f0xx_i2c.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_syscfg.h"
#include "usart.h"
#include "1wire.h"

#define V_REF 3.0




int analog_AO;

// Структура для Меню
//typedef struct {
struct Settings {
	uint8_t  	Imp;
	float  		m_sens; // сколько метров за 1 оборот датчика скорости
	uint16_t  TAH1;
	uint16_t  TAH2;
	uint16_t  TAH3;
	uint8_t  	Light_mode;
	uint16_t  Speed_max;
	uint8_t  	SP_Test_On_Off;
	uint16_t	speed_test_time;
	uint16_t	sp_test_delay_ms;
  float 		SP_Scale; //масштабирование шкалы для узменения макс угла отклонения стрелки
  float 		Frq_display_SP; // частота отображения скорости на спидометре, на дисплее;
	uint8_t		Tah_imp;
  uint16_t  TAH_max;
	uint8_t  	TAH_Test_On_Off;
	uint16_t	Tah_test_time;
	uint16_t	Tah_test_delay_ms;
	
	float 		TAH_Scale; //масштабирование шкалы для узменения макс угла отклонения стрелки
  float 		Frq_display_TAH; // частота отображения оборотов на тахометре, на дисплее;
	uint8_t 	Display_Num; // кол-во дисплеев
	float 		Callibrate_V1;
	uint8_t		Acceleration_60;
	uint8_t 	Acceleration_100;
	uint16_t	Acceleration_X;
	uint8_t		Temp_sensor;
	uint8_t		Presure_sensor;
	uint8_t		Fuel_sensor;
	uint16_t	Soft_light_time;
   uint16_t generate_SP;
   uint16_t generate_TAH;
   uint16_t LED_max;
   uint16_t LED_min;
   uint16_t Strelki_time;
	
};   

//Menu_TypeDef Settings;
struct Settings Set_Menu;
struct Settings *pointerSettings = &Set_Menu;

char     str[20],str2[20],str3[20],str4[20],str5[20],str6[20],str7[20],str8[20],str9[20],str10[20],
         str11[20],str12[20],str13[20],str14[20],str15[20],str16[20],str17[20],str18[20],str19[20],str20[20],str21[20],
				 str22[20],
				 txt[80],str_DS1[16],str_DS2[16],
         charset;
uint16_t msec,
         ACP[6],
         AS_5050,
         Angle_adc,
         Angle_old,
         x,
         y,
         z,
         t,
         speed_OUT_sr,
         speed,
         speed_sr,
         x_speed,
         speed_OUT,
         speed_OUT_mas[15],
         x_TAH,
         x_speed_TAH,
         y_TAH,
         z_TAH,
         Tach_OUT,
         t_TAH,
         speed_TAH,
         i_speed_TAH,
         speed_sr_TAH,
         test_SP,
         test_TAH,
         temp_sp_test_delay_ms,
         temp_Tah_test_delay_ms,
				 timer_DS,
             PWM14 = 175,
             cal_0,
             cal_50,
             cal_100,
             Ang_true
;



uint8_t  ADC_calibrate,
				 i_cal,
				 DATA_SPI,
				 FLASH_MEM[10],
				 FLASH_Save[10] = {1,2,3,4,5,6,7,8,9,10},
         i[6],
				 set_time[7],
         buffer[19],
         msec20,
         Flag_math,
         screen,
         sw_state_UP,
         sw_state_DN,
         sw_state_MENU,
         Gabarit_ON,
         Door_OPEN,
         Flag_upd,
         sw_state1,
         start_tim_100km,
         Imp,
         i_temp2,
         i1,
         i_speed,
         start_speed,
         start_tah,
         enter_level_menu,
         enter_menu,
         Button_UP_press,
         Button_DN_press,
         Button_MENU_press,
				 ROM[16],
				 temp2,
					temp1,
					temp3,
					temp4,
					mesure = 0,
					buf[2],
					FPS,
					FPS_max
         ;
				 
char 			number = 0;

float       ADC_sum[6],
            ADC_RMS[6],
            ADC_temp[6],
            ADC_data[6],
            ADC_chanel[6],
            ADC_chanel_avr_tmp[6],
            ADC_chanel_avr[6],
            mV_chanel[6],
            TEMP_V[6],
            TOK[6],
            Angle,
            kmetres,
            odometer,
            m_sens,
            mult_speed,
            mult_tah,
            test_SP_float,
            test_TAH_float,
            divider_soft_LED,
            divider_strelki_LED,
            PWM_LED,
            PWM_LED2,
            mul = 2.77,
						T_sr[2];

uint32_t    num_sample = 0,
            max_num_sample = 0,
            rms_sample = 0,
            max_rms_sample = 0,
            speed_temp,
            speed_temp_TAH
;

/*void Delay_ms(uint32_t ms)
{
        volatile uint32_t nCount;
        RCC_ClocksTypeDef RCC_Clocks;
				RCC_GetClocksFreq (&RCC_Clocks);

        nCount=(RCC_Clocks.HCLK_Frequency/10000)*ms;
        for (; nCount!=0; nCount--);
}

void Delay_us(uint32_t us)
{
        volatile uint32_t nCount;
        RCC_ClocksTypeDef RCC_Clocks;
				RCC_GetClocksFreq (&RCC_Clocks);

        nCount=(RCC_Clocks.HCLK_Frequency/10000000)*us;
        for (; nCount!=0; nCount--);
}
*/
void Init_port(void)
{
   RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN| RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOFEN;
   
//00: Input mode (reset state)
//01: General purpose output mode
//10: Alternate function mode
//11: Analog mode   
   //reset bits MODER
   GPIOC->MODER &= ~GPIO_MODER_MODER13 | 
                  GPIO_MODER_MODER14; 
   //OUT
   GPIOC->MODER |= ~GPIO_MODER_MODER13_0 | 
                  GPIO_MODER_MODER14_0; 
   
//   0: Output push-pull (reset state)
//   1: Output open-drain
   //reset OTYPER
   GPIOC->OTYPER &= ~GPIO_OTYPER_OT_13 
                  | GPIO_OTYPER_OT_14;
   //Output open-drain
   GPIOC->OTYPER |= GPIO_OTYPER_OT_13 
                  | GPIO_OTYPER_OT_14;
   
//   00: No pull-up, pull-down
//   01: Pull-up
//   10: Pull-down
//   11: Reserved
//reset PUPDR
   GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR13
                  | GPIO_PUPDR_PUPDR14;
   //   01: Pull-down               
   GPIOC->PUPDR |= GPIO_PUPDR_PUPDR13_1
                  | GPIO_PUPDR_PUPDR14_1;
                  
//x0: Low speed
//01: Medium speed
//11: High speed
//reset OSPEEDR
   GPIOC->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR13 | GPIO_OSPEEDER_OSPEEDR14;
   
   //Input BUTTON
   GPIOC->MODER &= ~GPIO_MODER_MODER15;
   GPIOC->OTYPER &= ~GPIO_OTYPER_OT_15;
   //Output open-drain
   GPIOC->OTYPER |= GPIO_OTYPER_OT_15;
   GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR15;
   //   01: Pull-up               
   GPIOC->PUPDR |= GPIO_PUPDR_PUPDR15_0;
   
   //Input BUTTON
   GPIOF->MODER &= ~GPIO_MODER_MODER0 | GPIO_MODER_MODER1;
   GPIOF->OTYPER &= ~GPIO_OTYPER_OT_0 | GPIO_OTYPER_OT_1;
   //Output open-drain
   GPIOF->OTYPER |= GPIO_OTYPER_OT_0 | GPIO_OTYPER_OT_1;
   GPIOF->PUPDR &= ~GPIO_PUPDR_PUPDR0 | GPIO_PUPDR_PUPDR1;
   //   01: Pull-up               
   GPIOF->PUPDR |= GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR1_0;
   
   //Input RESET_CAL
   GPIOA->MODER &= ~GPIO_MODER_MODER1;
   GPIOA->OTYPER &= ~GPIO_OTYPER_OT_1;
   //Output open-drain
   GPIOA->OTYPER |= GPIO_OTYPER_OT_1;
   GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR1;
   //   01: Pull-up               
   GPIOA->PUPDR |= GPIO_PUPDR_PUPDR1_0;
   
   //Input GABARIT
   GPIOB->MODER &= ~GPIO_MODER_MODER2;
   GPIOB->OTYPER &= ~GPIO_OTYPER_OT_2;
   //Output open-drain
   GPIOB->OTYPER |= GPIO_OTYPER_OT_2;
   GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR2;
   //   01: Pull-down               
   GPIOB->PUPDR |= GPIO_PUPDR_PUPDR2_1;
}

void InitTimer15(void)
{
	//B14 - CH1 - SPEED
   //B15 - CH2 - TAH
   
//	TIM_TimeBaseInitTypeDef TIM_Time_user;
//	TIM_ICInitTypeDef TIM_InputCapture;
//   GPIO_InitTypeDef GPIO_InitStructure;
   NVIC_InitTypeDef NVIC_InitStructure;
   
   RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
   RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
   
   //10: Alternate function mode
   GPIOB->MODER |= (    GPIO_MODER_MODER14_1
                     |  GPIO_MODER_MODER15_1
                     );
   
   //11: High speed
   GPIOB->OSPEEDR |= (  GPIO_OSPEEDR_OSPEEDR14
                     |  GPIO_OSPEEDR_OSPEEDR15
                     );
   
   //00: No pull-up, pull-down
   //01: Pull-up
   //10: Pull-down
   GPIOB->PUPDR &= ~(   GPIO_PUPDR_PUPDR14
                     |  GPIO_PUPDR_PUPDR15
                     );
   //AF1 TIM15_CH1, TIM15_CH2                  
   GPIOB->AFR[1] |= (0x01 << (4*6)) | (0x01 << (4*7));
   
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM15, ENABLE);
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
//   
//   /* TIM1 channel 2 pin (PE.11) configuration */
//  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_14 | GPIO_Pin_15;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//  GPIO_Init(GPIOB, &GPIO_InitStructure);

//  /* Connect TIM pins to AF2 */
//  GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_1);//2,  5
//  GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_1);//2,  5
   
   RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
   
   TIM15->CCMR1 |= TIM_CCMR1_CC2S_0 | TIM_CCMR1_CC1S_0;//включаем захват по второму каналу
   TIM15->CCER |= (TIM_CCER_CC2E //включаем ножку для входа захвата.
                  | TIM_CCER_CC1E
                  );
   TIM15->DIER |= TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_UIE;
   TIM15->PSC = 480 - 1;
   TIM15->ARR = 65000;
   TIM15->CR1 |= TIM_CR1_CEN; // tim15 enable
   
//	TIM_Time_user.TIM_Prescaler = 480 - 1; //100 000 kHz считает с такой частотой
//	TIM_Time_user.TIM_CounterMode = TIM_CounterMode_Up;
//	TIM_Time_user.TIM_Period = 65000;  //?? ????? ?????? ??????? ? ????????????
//	TIM_Time_user.TIM_ClockDivision = TIM_CKD_DIV1;//32 MHz
//	TIM_TimeBaseInit(TIM15, &TIM_Time_user);
//	
//   TIM_InputCapture.TIM_Channel = TIM_Channel_1;// | TIM_Channel_2;      
//   TIM_InputCapture.TIM_ICPolarity = TIM_ICPolarity_Rising;   
//   TIM_InputCapture.TIM_ICSelection = TIM_ICSelection_DirectTI;  
//   TIM_InputCapture.TIM_ICPrescaler = TIM_ICPSC_DIV1;  
//   TIM_InputCapture.TIM_ICFilter = 0x0;     /*!< Specifies the input capture filter.
//                                  This parameter can be a number between 0x0 and 0xF */
//   TIM_ICInit(TIM15, &TIM_InputCapture);
//   TIM15->CCMR1 |= TIM_CCMR1_CC2S_0;//включаем захват по второму каналу
//   TIM15->CCER |= TIM_CCER_CC2E; //включаем ножку для входа захвата.
//   
//   TIM_Cmd(TIM15, ENABLE);
//   TIM_ITConfig(TIM15, TIM_IT_CC1, ENABLE);
//   TIM_ITConfig(TIM15, TIM_IT_CC2, ENABLE);
//   TIM_ITConfig(TIM15, TIM_IT_Update,ENABLE);
   NVIC_InitStructure.NVIC_IRQChannel = TIM15_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//NVIC_EnableIRQ(TIM14_IRQn);
   NVIC_Init(&NVIC_InitStructure);
	
	
	
	
	
}

void TIM15_IRQHandler(void)
{
	
   if (TIM_GetITStatus(TIM15, TIM_IT_CC1) != RESET)
   {
      //******************************
      //обработка сигнала датчика скорости
      if  ( (screen == 4) && (sw_state1 == 0) ) {start_tim_100km = 1; sw_state1 = 1;  } //запуск таймера разгона до 100 км/ч
//      count_imp++;     
m_sens = 1;
Imp = 1;			
      kmetres += (m_sens/Imp)*0.001;
      odometer += (m_sens/Imp)*0.001;
      TIM_Cmd(TIM16, ENABLE); // скорость
      
//      if (count_imp == Imp) {kmetres += (m_sens/Imp); count_imp=0;}
      /*if (metres > 999) 
      {
         
         metres = 0;
         
         km_0099++;
         if (km_0099 == 100) 
         {
            
            km_0099 = 0;
            km_0099_old = 0;
            save_eep_0099 = 1;
            km_900++;
            if (km_900 == 10) {km_900 = 0;km_900_old = 0;save_eep = 1;}
         }
         km_000099++;
         if (km_000099 == 100) 
         {
            km_000099 = 0;save_eep_000099 = 1;
            km_009900++;
            if (km_009900 == 100) 
            {
               km_009900 = 0;save_eep_009900 = 1;
               km_990000++;
               if (km_990000 == 100) {km_990000 = 0;save_eep_990000 = 1;}
            }
         }
      }
       */  
      if(x==0) //в первый заход считываем таймер в переменную х
      {
         x_speed=0;
         x=TIM_GetCapture1(TIM15);
      }
      else   //во второй заход считываем в y, находим разницу
      {
         x_speed=0;
         y=TIM_GetCapture1(TIM15);
         //ICR1=0;
         //TCNT1=0;
         z=(y-x);
         x=y;
         
//1 м/сек = 3.6 км/ч
//1 м/сек = 60 м/мин
//1 м/сек = 3600 м/ч
//1 м/сек = 2.236936292 миль/час
//1 м/сек = 60000 мм/мин
//1 м/сек = 1000 мм/сек
//1 м/сек = 100 см/сек
         
/*

Датчик скорости 6 имп на оборот
1 оборот равен 1.13 метра
 speed=t*((3.6*1.13)/Imp);        
         
*/
         
         t=(100000 + 800)/z;
         speed_OUT=t;
         //для плавного поворота стрелки сглаживаем 
         speed_OUT_mas[i_temp2] = speed_OUT;
         i_temp2++;
         if (i_temp2 == 14) i_temp2 = 0;
         for(i1 = 0;i1 < 14; i1++)
         {
            speed_OUT_sr += speed_OUT_mas[i1];
         }
         speed_OUT_sr /= 15;// это значение берем для генерации частоты на выходе сигнала поворота стрелки
         /////*********************///
         
         speed=t*((3.6*m_sens)/Imp);//(t/6)*3.6;//t*0.68;
         
         speed_temp+=speed;
         i_speed++;
         if (i_speed >= 10) {
            i_speed=0;
            //for (k = 0;k++;k<10)
            //{
               //speed_temp+=speed_int[k];
            //}
            speed_sr=speed_temp*0.1;  speed_temp=0;
            
         }
         if (speed_sr >= 100)  start_tim_100km = 0;
         if (speed_sr >=999) speed_sr = 999;
      }
      TIM_ClearITPendingBit(TIM15, TIM_IT_CC1);
    
      
   }
   //*******************************
   
   if (TIM_GetITStatus(TIM15, TIM_IT_CC2) != RESET)
   {
      //обработка сигнала тахометра
     
         
      if(x_TAH==0) //в первый заход считываем таймер в переменную х
      {
         x_speed_TAH=0;
         x_TAH=TIM_GetCapture2(TIM15);
      }
      else   //во второй заход считываем в y, находим разницу
      {
         x_speed_TAH=0;
         y_TAH=TIM_GetCapture2(TIM15);
         //ICR1=0;
         //TCNT1=0;
         z_TAH=(y_TAH-x_TAH);
         x_TAH=y_TAH;
         Tach_OUT=100000/z_TAH;
         t_TAH=100000/(z_TAH*2)*60;
         speed_TAH=t_TAH*1;
         speed_temp_TAH+=speed_TAH;
         i_speed_TAH++;
         if (i_speed_TAH == 20) {
            i_speed_TAH=0;
            //for (k = 0;k++;k<10)
            //{
               //speed_temp+=speed_int[k];
            //}
            speed_sr_TAH=speed_temp_TAH*0.05;  speed_temp_TAH=0;
            if (speed_sr_TAH >=9999) speed_sr_TAH = 9999;
         }
      }
      TIM_ClearITPendingBit(TIM15, TIM_IT_CC2);
      
   }
   
   
   
	if (TIM_GetITStatus(TIM15, TIM_IT_Update) != RESET)
	{
		
//		timer_temp_1++;
      x_speed++;
      x_speed_TAH++;
	
	x=0; x_TAH=0;
	y=0; y_TAH=0;
	z=0; z_TAH=0;
	t=0; t_TAH=0;
	speed=0; speed_TAH=0;
//   if ((start_speed == 2) && (speed_OUT == 0)) {   
////      TIM_Cmd(TIM16, DISABLE); // две строчки чтобы выключить ножку скорость
//      TIM16->BDTR |= TIM_BDTR_BKE;
//	} else TIM16->BDTR &= ~TIM_BDTR_BKE; // выключаем выход таймера OC
	if (x_speed > 1) {speed_sr=0;speed_OUT_sr = 0; x_speed=0;speed_temp = 0;speed_OUT = 0;}
   if (x_speed_TAH > 1) {speed_sr_TAH=0;x_speed_TAH=0;speed_temp_TAH = 0;Tach_OUT = 0;}
   
		

     TIM_ClearITPendingBit(TIM15, TIM_IT_Update); 
	}
	
	
}





void InitTimer16(void)
{
	// PB14,PB15 - speed, Tah input AF1, AF1
	//PORTB8 -  OUT Test strelok
	TIM_TimeBaseInitTypeDef TIM_Time_user;
	TIM_OCInitTypeDef Hard_PWM_ini;
	GPIO_InitTypeDef Pin;
   
   
   
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16, ENABLE);// ???????? ???????????? ??????? 3
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);
   
   Pin.GPIO_Pin = GPIO_Pin_8;// OUT
		Pin.GPIO_Mode = GPIO_Mode_AF;
		Pin.GPIO_Speed = GPIO_Speed_10MHz;
		Pin.GPIO_OType = GPIO_OType_PP;
		Pin.GPIO_PuPd = GPIO_PuPd_NOPULL;//GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOB, &Pin);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_2); //2,5.0
//      GPIOB->AFR[1] |= 0x02<<(4 * (GPIO_PinSource8 - 8)) |    // AF-2 PB8 
//                     0x02<<(4 * (GPIO_PinSource9 - 8));        // AF-2 PB9 
   
	TIM_Time_user.TIM_Prescaler = 24000 - 1; //100 000 kHz///620
	TIM_Time_user.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_Time_user.TIM_Period = 200;  //?? ????? ?????? ??????? ? ????????????
	TIM_Time_user.TIM_ClockDivision = TIM_CKD_DIV2;//26 MHz тактирование
	TIM_TimeBaseInit(TIM16, &TIM_Time_user);
	
	
	
		Hard_PWM_ini.TIM_OCMode = TIM_OCMode_PWM1;
		Hard_PWM_ini.TIM_OutputState = TIM_OutputState_Enable;
		//Hard_PWM_ini.TIM_OutputNState = ;  This parameter is valid only for TIM1 and TIM8
		Hard_PWM_ini.TIM_Pulse = 100;
		Hard_PWM_ini.TIM_OCPolarity = TIM_OCPolarity_High;
		//Hard_PWM_ini.TIM_OCNPolarity = ;This parameter is valid only for TIM1 and TIM8
		//Hard_PWM_ini.TIM_OCIdleState = ;This parameter is valid only for TIM1 and TIM8
		//Hard_PWM_ini.TIM_OCNIdleState = ;This parameter is valid only for TIM1 and TIM8

		//NVIC_EnableIRQ(TIM3_IRQn); //????????? ????????? ?????????? ??????? 2
		//TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
		
		
		
      
	
	TIM_OC1Init(TIM16, &Hard_PWM_ini);//???????? ??????
   TIM_OC1PreloadConfig(TIM16, TIM_OCPreload_Enable);
   //TIM16->BDTR |= TIM_BDTR_MOE | TIM_CR1_ARPE;
   //TIM16->CCMR1 |= TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1; 
TIM16->CCER |= TIM_CCER_CC1E; 
TIM16->BDTR |= TIM_BDTR_MOE | TIM_BDTR_AOE;  
		TIM_Cmd(TIM16, ENABLE);//???????? ??????
      
        
      
}

/*** инит таймера 17 канал 1, PORTB 9 для ШИМ-выхода. Тест стрелок 
*/
void InitTimer17(void)
{
	
	//PORTB9 -  OUT Test strelok tahom
	TIM_TimeBaseInitTypeDef TIM_Time_user;
	TIM_OCInitTypeDef Hard_PWM_ini;
	GPIO_InitTypeDef Pin;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM17, ENABLE);// ???????? ???????????? ??????? 3
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);
   
   Pin.GPIO_Pin = GPIO_Pin_9;// OUT
		Pin.GPIO_Mode = GPIO_Mode_AF;
		Pin.GPIO_Speed = GPIO_Speed_10MHz;
		Pin.GPIO_OType = GPIO_OType_PP;
		Pin.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOB, &Pin);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_2); //2,5
   
	TIM_Time_user.TIM_Prescaler = 24000 - 1; //100 000 kHz///620
	TIM_Time_user.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_Time_user.TIM_Period = 200;  //?? ????? ?????? ??????? ? ????????????
	TIM_Time_user.TIM_ClockDivision = TIM_CKD_DIV2;//24 MHz тактирование
	TIM_TimeBaseInit(TIM17, &TIM_Time_user);
	
	
	
		Hard_PWM_ini.TIM_OCMode = TIM_OCMode_PWM1;
		Hard_PWM_ini.TIM_OutputState = TIM_OutputState_Enable;
		//Hard_PWM_ini.TIM_OutputNState = ;  This parameter is valid only for TIM1 and TIM8
		Hard_PWM_ini.TIM_Pulse = 50;
		Hard_PWM_ini.TIM_OCPolarity = TIM_OCPolarity_High;
		//Hard_PWM_ini.TIM_OCNPolarity = ;This parameter is valid only for TIM1 and TIM8
		//Hard_PWM_ini.TIM_OCIdleState = ;This parameter is valid only for TIM1 and TIM8
		//Hard_PWM_ini.TIM_OCNIdleState = ;This parameter is valid only for TIM1 and TIM8
      
      
      
		//NVIC_EnableIRQ(TIM3_IRQn); //????????? ????????? ?????????? ??????? 2
		//TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
		TIM_OC1Init(TIM17, &Hard_PWM_ini);//???????? ??????
      TIM_OC1PreloadConfig(TIM17, TIM_OCPreload_Enable);
      //TIM17->CCMR1 |= TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1; /* (4) */
TIM17->CCER |= TIM_CCER_CC1E; /* (5) */
TIM17->BDTR |= TIM_BDTR_MOE | TIM_BDTR_AOE;  /* (6) */
		TIM_Cmd(TIM17, ENABLE);//???????? ??????
		
		
	
	
}


void SetSysClockToHSI48(void)
{
   uint32_t StartUpCounter = 0, HSIStatus = 0;
  
  /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration ---------------------------*/    
  /* Enable HSE */    
//  RCC->CR |= ((uint32_t)RCC_CR_HSEON);
 
  /* Wait till HSE is ready and if Time out is reached exit */
  do
  {
    HSIStatus = RCC->CR & RCC_CR_HSIRDY;
    StartUpCounter++;  
  } while((HSIStatus == 0) && (StartUpCounter != HSI_STARTUP_TIMEOUT));

  if ((RCC->CR & RCC_CR_HSIRDY) != RESET)
  {
    HSIStatus = (uint32_t)0x01;
  }
  else
  {
    HSIStatus = (uint32_t)0x00;
  }  

  if (HSIStatus == (uint32_t)0x01)
  {

    /* Enable Prefetch Buffer */
    //FLASH->ACR |= FLASH_ACR_PRFTBE;
		FLASH_PrefetchBufferCmd(ENABLE);
		
    /* Flash 0 wait state */
//    FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
//    FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_0;    
FLASH->ACR |= (uint32_t)FLASH_Latency_1;
//	FLASH_SetLatency(FLASH_Latency_1);
//    /* HCLK = SYSCLK */
//    RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;
//      
//    /* PCLK2 = HCLK */
//    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV1;
//    
//    /* PCLK1 = HCLK */
//    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV1;
    
		/*  PLL configuration:  = (HSI / 2) * 12 = 48 MHz */
		RCC_PREDIV1Config(RCC_PREDIV1_Div1);
		RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_12);
		

    /* Enable PLL */
    RCC->CR |= RCC_CR_PLLON;

    /* Wait till PLL is ready */
    while((RCC->CR & RCC_CR_PLLRDY) == 0)
    {
    }

    /* Select PLL as system clock source */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;    

    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x08)
    {
    }
  }
  else
  { /* If HSE fails to start-up, the application will have wrong clock 
         configuration. User can add here some code to deal with this error */
  } 
}



void Math_ADC(uint8_t num)
{
   
   if (!ADC_calibrate) 
      {
				for (i_cal = 0; i_cal < 6; i_cal++)
				{
					ADC_RMS[i_cal] = ADC_sum[i_cal] / rms_sample;
					ADC_sum[i_cal] = 0;
					ADC_calibrate = 1;
					ADC_chanel[i_cal] = sqrt(ADC_data[i_cal] / rms_sample);
					ADC_data[i_cal] = 0;
					ADC_chanel_avr_tmp[i_cal] += ADC_chanel[i_cal];
					ADC_chanel_avr[i_cal] = ADC_chanel_avr_tmp[i_cal];ADC_chanel_avr_tmp[i_cal] = 0;
					mV_chanel[i_cal] = ADC_chanel_avr[i_cal] * (V_REF/4096);
					TEMP_V[i_cal] = mV_chanel[i_cal] * (V_REF/4096);
				}
//         ADC_RMS[0] = ADC_sum[0] / rms_sample;
//         ADC_sum[0] = 0; //}
//         ADC_RMS[1] = ADC_sum[1] / rms_sample;
//         ADC_sum[1] = 0;
//          
//         ADC_calibrate = 1;
//         
//         ADC_chanel[0] = sqrt(ADC_data[0] / rms_sample);
//         ADC_data[0] = 0;
//         ADC_chanel[1] = sqrt(ADC_data[1] / rms_sample);
//         ADC_data[1] = 0;
//         
//         
//         ADC_chanel_avr_tmp[0] += ADC_chanel[0]; 
//         ADC_chanel_avr_tmp[1] += ADC_chanel[1]; 
//         
//         
//         ADC_chanel_avr[0] = ADC_chanel_avr_tmp[0];ADC_chanel_avr_tmp[0] = 0;
//         ADC_chanel_avr[1] = ADC_chanel_avr_tmp[1];ADC_chanel_avr_tmp[1] = 0;
//         
//         
//         mV_chanel[0] = ADC_chanel_avr[0] * (V_REF/4096);
//         mV_chanel[1] = ADC_chanel_avr[1] * (V_REF/4096);
//         
//         
//         TEMP_V[0] = mV_chanel[0] * (V_REF/4096);
//         TEMP_V[1] = mV_chanel[1] * (V_REF/4096);
         
         
      }
//         max_rms_sample = rms_sample;
//         rms_sample = 0;
         
//         ADC_RMS_avr_tmp[num] += ADC_RMS[num] ;
//         ADC_RMS_avr_tmp[1] += ADC_RMS[1] ;
//         ADC_RMS_avr_tmp[2] += ADC_RMS[2] ;
         
         ADC_chanel[num] = sqrt(ADC_data[num] / rms_sample);
         ADC_data[num] = 0;
//         ADC_chanel[1] = sqrt(ADC_data[1] / rms_sample);
//         ADC_data[1] = 0;
//         ADC_chanel[2] = sqrt(ADC_data[2] / rms_sample);
//         ADC_data[2] = 0;
         
         ADC_chanel_avr_tmp[num] += ADC_chanel[num]; 
//         ADC_chanel_avr_tmp[1] += ADC_chanel[1]; 
//         ADC_chanel_avr_tmp[2] += ADC_chanel[2]; 
         
         i[num]++;
         if (i[num] == 10) 
         {
//            ADC_RMS_avr[num] = ADC_RMS_avr_tmp[num]*0.1;
//            ADC_RMS_avr_tmp[num] = 0;
//            ADC_RMS_avr[1] = ADC_RMS_avr_tmp[1]*0.05;
//            ADC_RMS_avr_tmp[1] = 0;
//            ADC_RMS_avr[2] = ADC_RMS_avr_tmp[2]*0.05;
//            ADC_RMS_avr_tmp[2] = 0;
//            
            ADC_chanel_avr[num] = ADC_chanel_avr_tmp[num] /10;ADC_chanel_avr_tmp[num] = 0;
//            ADC_chanel_avr[1] = ADC_chanel_avr_tmp[1] * 0.05;ADC_chanel_avr_tmp[1] = 0;
//            ADC_chanel_avr[2] = ADC_chanel_avr_tmp[2] * 0.05;ADC_chanel_avr_tmp[2] = 0;
//            
//            mV_RMS[num] = ADC_RMS_avr[num] * (3.3/4096);
//            mV_RMS[1] = ADC_RMS_avr[1] * (3.3/4096);
//            mV_RMS[2] = ADC_RMS_avr[2] * (3.3/4096);
//            
            
            
            i[num] = 0;
         }
   mV_chanel[num] = ( ADC_chanel_avr[num] * (V_REF/4096) ) - 0.003;
//            mV_chanel[1] = ADC_chanel_avr[1] * (3.3/4096);
//            mV_chanel[2] = ADC_chanel_avr[2] * (3.3/4096);
}

void SysTick_Handler (void) 
{
   msec++;
   msec20++;
   if (mesure == 1) timer_DS++;
	 if (timer_DS == 750)
   {
		 
      mesure = 2;
      timer_DS = 0;
   }	
	
   if (msec >999) 
   {
      msec = 0;
      Flag_upd = 1;
		  FPS_max = FPS;
		  FPS = 0;
   }
   pointerSettings->SP_Test_On_Off  = 1;
   pointerSettings->TAH_Test_On_Off = 1;
   
   //Тест стрелок
   
   if ( ((start_speed == 3) || (start_tah == 3)) && (enter_menu == 0) ) 
   {
      GPIOC->BSRR |= GPIO_BSRR_BS_13; GPIOC->BSRR |= GPIO_BSRR_BS_14; GPIOB->BSRR |= GPIO_BSRR_BS_13; 
      
   } 
   else {GPIOC->BSRR |= GPIO_BSRR_BR_13; GPIOC->BSRR |= GPIO_BSRR_BR_14; GPIOB->BSRR |= GPIO_BSRR_BR_13;}
   
   //gabarit
   if (GPIO_ReadInputDataBit( GPIOB, GPIO_Pin_2) == 1 ) Gabarit_ON = 1;
   if (GPIO_ReadInputDataBit( GPIOB, GPIO_Pin_2) == 0 ) Gabarit_ON = 0;
   
   //DOOR
   if (GPIO_ReadInputDataBit( GPIOB, GPIO_Pin_12) == 1 ) Door_OPEN = 0;
   if (GPIO_ReadInputDataBit( GPIOB, GPIO_Pin_12) == 0 ) Door_OPEN = 1;
   
   if (msec20 == 200) 
   {
//      if (PWM14 < 999) PWM14++; else PWM14 = 175;
      Math_ADC(0);
      Math_ADC(1);
			Math_ADC(2);
      Math_ADC(3);
			Math_ADC(4);
      Math_ADC(5);
      msec20 = 0;
      Flag_math = 1;
      max_rms_sample = rms_sample;
      rms_sample = 0;
   }
}



void InitTimer3(void)
{
	//PORTB0 -  OUT PWM подсветка плавная - panel
	//PORTB1 -  OUT PWM подсветка плавная - strelki
	TIM_TimeBaseInitTypeDef TIM_Time_user;
	TIM_OCInitTypeDef Hard_PWM_ini;
	GPIO_InitTypeDef Pin;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);// ???????? ???????????? ??????? 3
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);
   
   Pin.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;// OUT
   Pin.GPIO_Mode = GPIO_Mode_AF;
   Pin.GPIO_Speed = GPIO_Speed_10MHz;
   Pin.GPIO_OType = GPIO_OType_PP;
   Pin.GPIO_PuPd = GPIO_PuPd_NOPULL;
   GPIO_Init(GPIOB, &Pin);
   GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_1); //2,5
   GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_1);
   
	TIM_Time_user.TIM_Prescaler = 48 - 1; //1 kHz///
	TIM_Time_user.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_Time_user.TIM_Period = 1000;  //?? ????? ?????? ??????? ? ????????????
	TIM_Time_user.TIM_ClockDivision = TIM_CKD_DIV1;//24 MHz тактирование
	TIM_TimeBaseInit(TIM3, &TIM_Time_user);
	
	
	
		Hard_PWM_ini.TIM_OCMode = TIM_OCMode_PWM1;
		Hard_PWM_ini.TIM_OutputState = TIM_OutputState_Enable;
		//Hard_PWM_ini.TIM_OutputNState = ;  This parameter is valid only for TIM1 and TIM8
		Hard_PWM_ini.TIM_Pulse = 0;
		Hard_PWM_ini.TIM_OCPolarity = TIM_OCPolarity_High;
		//Hard_PWM_ini.TIM_OCNPolarity = ;This parameter is valid only for TIM1 and TIM8
		//Hard_PWM_ini.TIM_OCIdleState = ;This parameter is valid only for TIM1 and TIM8
		//Hard_PWM_ini.TIM_OCNIdleState = ;This parameter is valid only for TIM1 and TIM8
      
      
      
		//NVIC_EnableIRQ(TIM3_IRQn); //????????? ????????? ?????????? ??????? 2
		//TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
		TIM_OC3Init(TIM3, &Hard_PWM_ini);//???????? ??????
      TIM_OC4Init(TIM3, &Hard_PWM_ini);
      TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
      TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
      //TIM17->CCMR1 |= TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1; /* (4) */
TIM3->CCER |= TIM_CCER_CC4E; /* (5) */
//TIM3->CCR2 = 200; // PWM
TIM3->BDTR |= TIM_BDTR_MOE | TIM_BDTR_AOE;  /* (6) */
		TIM_Cmd(TIM3, ENABLE);//???????? ??????
		
		
	
	
}

void InitTimer1(void)
{
	//PORTA8 -  OUT PWM подсветка плавная - salon
	
	TIM_TimeBaseInitTypeDef TIM_Time_user;
	TIM_OCInitTypeDef Hard_PWM_ini;
	GPIO_InitTypeDef Pin;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);// ???????? ???????????? ??????? 3
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);
   
   Pin.GPIO_Pin = GPIO_Pin_8;// OUT
   Pin.GPIO_Mode = GPIO_Mode_AF;
   Pin.GPIO_Speed = GPIO_Speed_10MHz;
   Pin.GPIO_OType = GPIO_OType_PP;
   Pin.GPIO_PuPd = GPIO_PuPd_NOPULL;
   GPIO_Init(GPIOA, &Pin);
   GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_2); //2,5
   
   
	TIM_Time_user.TIM_Prescaler = 48 - 1; //1 kHz///
	TIM_Time_user.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_Time_user.TIM_Period = 1000;  //?? ????? ?????? ??????? ? ????????????
	TIM_Time_user.TIM_ClockDivision = TIM_CKD_DIV1;//24 MHz тактирование
	TIM_TimeBaseInit(TIM1, &TIM_Time_user);
	
	
	
   Hard_PWM_ini.TIM_OCMode = TIM_OCMode_PWM1;
   Hard_PWM_ini.TIM_OutputState = TIM_OutputState_Enable;
   //Hard_PWM_ini.TIM_OutputNState = ;  This parameter is valid only for TIM1 and TIM8
   Hard_PWM_ini.TIM_Pulse = 0;
   Hard_PWM_ini.TIM_OCPolarity = TIM_OCPolarity_High;
   //Hard_PWM_ini.TIM_OCNPolarity = ;This parameter is valid only for TIM1 and TIM8
   //Hard_PWM_ini.TIM_OCIdleState = ;This parameter is valid only for TIM1 and TIM8
   //Hard_PWM_ini.TIM_OCNIdleState = ;This parameter is valid only for TIM1 and TIM8
   
   
   
   //NVIC_EnableIRQ(TIM3_IRQn); //????????? ????????? ?????????? ??????? 2
   //TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
   TIM_OC1Init(TIM1, &Hard_PWM_ini);//???????? ??????
   TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
   
   //TIM17->CCMR1 |= TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1; /* (4) */
   TIM1->CCER |= TIM_CCER_CC4E; /* (5) */
   //TIM3->CCR2 = 200; // PWM
   TIM1->BDTR |= TIM_BDTR_MOE | TIM_BDTR_AOE;  /* (6) */
   TIM_Cmd(TIM1, ENABLE);//???????? ??????
			
}




void InitTimer14(void)
{
	
	//PORTB1 -  OUT PWM подсветка плавная
	TIM_TimeBaseInitTypeDef TIM_Time_user;
	TIM_OCInitTypeDef Hard_PWM_ini;
	GPIO_InitTypeDef Pin;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);// ???????? ???????????? ??????? 3
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);
   
   Pin.GPIO_Pin = GPIO_Pin_4;// OUT
		Pin.GPIO_Mode = GPIO_Mode_AF;
		Pin.GPIO_Speed = GPIO_Speed_10MHz;
		Pin.GPIO_OType = GPIO_OType_PP;
		Pin.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOA, &Pin);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_4); //2,5
   
	TIM_Time_user.TIM_Prescaler = 2 - 1; //100 000 kHz///620
	TIM_Time_user.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_Time_user.TIM_Period = 1000;  //?? ????? ?????? ??????? ? ????????????
	TIM_Time_user.TIM_ClockDivision = TIM_CKD_DIV1;//24 MHz тактирование
	TIM_TimeBaseInit(TIM14, &TIM_Time_user);
	
	
	
		Hard_PWM_ini.TIM_OCMode = TIM_OCMode_PWM1;
		Hard_PWM_ini.TIM_OutputState = TIM_OutputState_Enable;
		//Hard_PWM_ini.TIM_OutputNState = ;  This parameter is valid only for TIM1 and TIM8
		Hard_PWM_ini.TIM_Pulse = 0;
		Hard_PWM_ini.TIM_OCPolarity = TIM_OCPolarity_High;
		//Hard_PWM_ini.TIM_OCNPolarity = ;This parameter is valid only for TIM1 and TIM8
		//Hard_PWM_ini.TIM_OCIdleState = ;This parameter is valid only for TIM1 and TIM8
		//Hard_PWM_ini.TIM_OCNIdleState = ;This parameter is valid only for TIM1 and TIM8
      
      
      
		//NVIC_EnableIRQ(TIM3_IRQn); //????????? ????????? ?????????? ??????? 2
		//TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
		TIM_OC1Init(TIM14, &Hard_PWM_ini);//???????? ??????
      TIM_OC1PreloadConfig(TIM14, TIM_OCPreload_Enable);
      //TIM17->CCMR1 |= TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1; /* (4) */
TIM14->CCER |= TIM_CCER_CC4E; /* (5) */
//TIM3->CCR2 = 200; // PWM
TIM14->BDTR |= TIM_BDTR_MOE | TIM_BDTR_AOE;  /* (6) */
		TIM_Cmd(TIM14, ENABLE);//???????? ??????
		
		
	
	
}




void Init_Port_ADC(void)
{
   RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
   
//   00: Input mode (reset state)
//   01: General purpose output mode
//   10: Alternate function mode
//   11: Analog mode
   GPIOA->MODER |= (    GPIO_MODER_MODER0
											| GPIO_MODER_MODER1
											| GPIO_MODER_MODER4
											| GPIO_MODER_MODER5
											| GPIO_MODER_MODER6
											| GPIO_MODER_MODER7
                     );
	
//   0: Output push-pull (reset state)
//   1: Output open-drain
   //reset OTYPER
   GPIOA->OTYPER &= ~GPIO_OTYPER_OT_0 
                  | ~GPIO_OTYPER_OT_1
									| ~GPIO_OTYPER_OT_4
									| ~GPIO_OTYPER_OT_5
									| ~GPIO_OTYPER_OT_6
									| ~GPIO_OTYPER_OT_7;
   //Output open-drain
   GPIOA->OTYPER |= GPIO_OTYPER_OT_0 
                  | GPIO_OTYPER_OT_1
									| GPIO_OTYPER_OT_4
									| GPIO_OTYPER_OT_5
									| GPIO_OTYPER_OT_6
									| GPIO_OTYPER_OT_7;
   
   //11: High speed
   GPIOA->OSPEEDR |= (  GPIO_OSPEEDR_OSPEEDR0
											| GPIO_OSPEEDR_OSPEEDR1
											| GPIO_OSPEEDR_OSPEEDR4
											| GPIO_OSPEEDR_OSPEEDR5
                      | GPIO_OSPEEDR_OSPEEDR6
                      | GPIO_OSPEEDR_OSPEEDR7
                     );
   
   //00: No pull-up, pull-down
   //01: Pull-up
   //10: Pull-down
   GPIOA->PUPDR &= ~(  GPIO_PUPDR_PUPDR0_1 
                     | GPIO_PUPDR_PUPDR1_1
                     | GPIO_PUPDR_PUPDR4_1
										 | GPIO_PUPDR_PUPDR5_1
										 | GPIO_PUPDR_PUPDR6_1
										 | GPIO_PUPDR_PUPDR7_1
                     );
}

void ADC1_IRQHandler(void)
{
   if ( (ADC1->ISR & ADC_ISR_EOC) ) 
   {
      
//      ADC1->SR &= ~ADC_SR_EOC;
//      ADC1->ISR &= ~ADC_ISR_START;
//      ADC1->CR2 |= ADC_CR2_SWSTART;
//      switch(num_adc)
//      {
//         case 0:     ADC[0] = ADC1->DR;num_adc = 1;
//            break;
//         case 1:     ADC[1] = ADC1->DR;num_adc = 2;
//            break;
//         case 2:     ADC[2] = ADC1->DR; num_adc = 0;
//            break;
//      }
		
//      num_adc++;
//      if (num_adc > 2) num_adc = 0;
//      num_sample++;
      ADC1->ISR &= ~ADC_ISR_EOC;
//      ADC1->CR |= ADC_CR_ADSTART;	
//      ADC1->CR2 |= ADC_CR2_JSWSTART; // Start conversion of injected channels
      
   }
}

void DMA1_Channel1_IRQHandler(void)
{
   if ( (DMA1->ISR & DMA_ISR_TCIF1) )
   {
      DMA1->IFCR |= DMA_IFCR_CTCIF1;	// clear flag TC

//            mV1 = ACP[0]; // читаем значение преобразования

//            mV2 = ACP[1]; // читаем значение преобразования
      if (!ADC_calibrate)
		 {
				ADC_sum[0]+= ACP[0];
				ADC_sum[1]+= ACP[1];
				ADC_sum[2]+= ACP[2];
				ADC_sum[3]+= ACP[3];
				ADC_sum[4]+= ACP[4];
				ADC_sum[5]+= ACP[5];
				
		 }
      
      
      ADC_temp[0] = ACP[0];//ADC_RMS[0];
      ADC_data[0] += ADC_temp[0] * ADC_temp[0];
      ADC_temp[1] = ACP[1];//ADC_RMS[1]; - ADC_RMS[1]
      ADC_data[1] += ADC_temp[1] * ADC_temp[1];
      ADC_temp[2] = ACP[2];// - ADC_RMS[2]; //ADC_RMS[2];                                       
      ADC_data[2] += ADC_temp[2] * ADC_temp[2];
		  ADC_temp[3] = ACP[3];//ADC_RMS[0];
      ADC_data[3] += ADC_temp[3] * ADC_temp[3];
      ADC_temp[4] = ACP[4];//ADC_RMS[1]; - ADC_RMS[1]
      ADC_data[4] += ADC_temp[4] * ADC_temp[4];
      ADC_temp[5] = ACP[5];// - ADC_RMS[2]; //ADC_RMS[2];                                       
      ADC_data[5] += ADC_temp[5] * ADC_temp[5];
      
      rms_sample++;
      num_sample++;
      

      
      ADC1->CR |= ADC_CR_ADSTART; // запускаем преобразование
      
   }
}

void ADC_init(void)
{
	 /*
	 PA0 - ADC0 TEMPER
   PA1 - ADC1 PRESURE
   
   PA4 - ADC4 BENZ
   PA5 - ADC5 AMP
   PA6 - ADC6 VOLT
   PA7 - ADC7 LUMENS
	
	000: 1.5 ADC clock cycles
001: 7.5 ADC clock cycles
010: 13.5 ADC clock cycles
011: 28.5 ADC clock cycles
100: 41.5 ADC clock cycles
101: 55.5 ADC clock cycles
110: 71.5 ADC clock cycles
111: 239.5 ADC clock cycles
	*/
	
   RCC->APB2ENR |= RCC_APB2ENR_ADCEN; // включаем тактирование АЦП
   
   
   
   //ADC1->ISR |= ADC_ISR_EOC; // ФЛАГ по окончании преобразования
   ADC1->IER |= ADC_IER_EOCIE; // прерывание по окончании преобразования
   ADC1->SMPR |= ADC_SMPR_SMP_2 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_0; // 239.5 длительность выборки  ADC_SMPR_SMP_0 | 
   ADC1->CHSELR |= ADC_CHSELR_CHSEL0 | ADC_CHSELR_CHSEL1 | ADC_CHSELR_CHSEL4 | ADC_CHSELR_CHSEL5 | ADC_CHSELR_CHSEL6 | ADC_CHSELR_CHSEL7;//  | ADC_CHSELR_CHSEL4;
   ADC1->CFGR1 |= //ADC_CFGR1_DISCEN
                 ADC_CFGR1_DMACFG
               | ADC_CFGR1_DMAEN
//               | ADC_CFGR1_CONT
               ;
//   NVIC_EnableIRQ(ADC1_IRQn); // включаем прерывания АЦП
   ADC1->CR |= ADC_CR_ADCAL; // Запуск калибровки 
   while ((ADC1->CR & ADC_CR_ADCAL)) ; //ждем окончания калибровки
   
   ADC1->CR |= ADC_CR_ADEN; // включаем АЦП
   while ( !(ADC1->ISR & ADC_ISR_ADRDY) ) ; // ждем готовности АЦП
   ADC1->CR |= ADC_CR_ADSTART; // запускаем преобразование
   
}

void DMA_ini_ADC1(void)
{
   

   
   RCC->AHBENR    |= RCC_AHBENR_DMA1EN;
   
   DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;      //peripheral address
   DMA1_Channel1->CMAR = (uint32_t)&ACP[0];
   DMA1_Channel1->CNDTR = 6;
   
   DMA1_Channel1->CCR |= DMA_CCR_PL_0     //Channel priority level - medium
                     | DMA_CCR_MSIZE_0    //Memory size 16
                     | DMA_CCR_PSIZE_0    //Peripheral size 16
                     | DMA_CCR_MINC       //Memory increment mode
                     | DMA_CCR_CIRC     //Circular mode
                     | DMA_CCR_TCIE    //Transfer complete interrupt enable
                     | DMA_CCR_EN      //Channel enable
                     ;
   NVIC_EnableIRQ(DMA1_Channel1_IRQn);
   NVIC_SetPriority(DMA1_Channel1_IRQn,1);
}


void Button_UP(void)
{
   if ( ( (GPIOF->IDR & GPIO_IDR_0)) && (sw_state_UP) ) // Отпустили
   {
      Button_UP_press = 0;
      sw_state_UP = 0;
   }
   
   if ( ( !(GPIOF->IDR & GPIO_IDR_0)) && (!sw_state_UP) ) // Нажали
   {
      Button_UP_press = 1;
      sw_state_UP = 1;
   }
}

void Button_DN(void)
{
   if ( ( (GPIOF->IDR & GPIO_IDR_1)) && (sw_state_DN) ) // Отпустили
   {
      Button_DN_press = 0;
      sw_state_DN = 0;
   }
   
   if ( ( !(GPIOF->IDR & GPIO_IDR_1)) && (!sw_state_DN) ) // Нажали
   {
      Button_DN_press = 1;
      sw_state_DN = 1;
   }
}

void Button_MENU(void)
{
   if ( ( (GPIOC->IDR & GPIO_IDR_15)) && (sw_state_MENU) ) // Отпустили
   {
      Button_MENU_press = 0;
      sw_state_MENU = 0;
   }
   
   if ( ( !(GPIOC->IDR & GPIO_IDR_15)) && (!sw_state_MENU) ) // Нажали
   {
      Button_MENU_press = 1;
      sw_state_MENU = 1;
   }
}

int main(void)
{
   RCC_DeInit();
   SetSysClockToHSI48();
   Init_port();
//   InitTimer15();
//   InitTimer16();
//   InitTimer17();
   InitTimer14();
	Init_I2C1();
   
   
   Delay_ms(20);
	
   SysTick_Config(SystemCoreClock/1000); //1 ms
   
//   SPI_ini();
//		AT45DB161_Init();
   AS5050_Init();
//   CS_LOW;
//   AS_5050 = SPI_SendByteAS(0x3FFF);
//   CS_HIGH;
//   Delay_ms(50);
//	CS_LOW; 50165     17397
//   AS_5050 = SPI_SendByteAS(0x3FFF); // 0x8001
//   CS_HIGH; 
//	 Save_Parameters_Frst_Init_EE();
	
	   
      analog_AO = analog_AO - Angle;
	 
	 // на 5 сек отстают от notebook
//set_time[0] = 0; // sec
//set_time[1] = 00; // minutes
//set_time[2] = 0; // hours
//set_time[3] = 4; // day;
//set_time[4] = 30; // date
//set_time[5] = 8; //month
//set_time[6] = 18; // year

//DS3231_WriteDate(set_time);
	 
   Delay_ms(10);
//   AT45DB161_PageProgram(0, FLASH_Save, 5);
   //PWM14 = 500;
   
   if ( !(GPIOA->IDR & GPIO_IDR_1))  // Нажали
   {
      Delay_ms(1);
      CS_LOW;
      Delay_us(200);
      AS_5050 = SPI_SendByteAS(0xFFFF); // 0x842B
      CS_HIGH;
      Delay_ms(1);
      CS_LOW;
      Delay_us(200);
      AS_5050 = SPI_SendByteAS(0xFFFF); // 0x8001  0x8417
      CS_HIGH; 
      Angle_adc = AS_5050 & 0x7ff;
      //Angle = Angle_adc * 0.7;
      Angle = (Angle_adc * 180) / 1024;
      
      Delay_ms(1);
      CS_LOW;
      Delay_us(200);
      AS_5050 = SPI_SendByteAS(0xFFFF); // 0x842B
      CS_HIGH;
      Delay_ms(1);
      CS_LOW;
      Delay_us(200);
      AS_5050 = SPI_SendByteAS(0xFFFF); // 0x8001  0x8417
      CS_HIGH; 
      Angle_adc = AS_5050 & 0x7ff;
      //Angle = Angle_adc * 0.7;
      Angle = (Angle_adc * 180) / 1024;
      
      cal_50 = Angle;
			EEPROM_16bit_write(0,cal_50);
   } else {cal_50 = EEPROM_16bit_read(0);}
   //pointerSettings->Speed_max = EEPROM_16bit_read(12);
//   cal_50 = 119;
//   cal_50 = 250;
   if (cal_50 >= 135) {
      cal_0 = cal_50 - 135;
      cal_100 = cal_50 + 135;
      if (cal_100 > 360) cal_100 = cal_100 - 360;
   } else    
      if (cal_50 < 135) {
         cal_0 = 360 - (135 - cal_50);
         cal_100 = cal_50 + 135;
      }
//   cal_50 = 135;
//   cal_100 = 270;
   while(1) 
   {
      
//      Button_UP();
//      Button_DN();
//      Button_MENU();
      
//      AS_5050 = spiReadData();
//      DATA_SPI = AT45DB161_Read_Status();
//		 Delay_ms(10);
//		 AT45DB161_Read_Data(0, 0, 5, FLASH_MEM);
//		 Delay_ms(10);
      
   Delay_ms(1);
   CS_LOW;
      Delay_us(200);
   AS_5050 = SPI_SendByteAS(0xFFFF); // 0x842B
   CS_HIGH;
   Delay_ms(1);
	CS_LOW;
      Delay_us(200);
   AS_5050 = SPI_SendByteAS(0xFFFF); // 0x8001  0x8417
   CS_HIGH; 
      Angle_adc = AS_5050 & 0x7ff;
      //Angle = Angle_adc * 0.7;
      Angle = (Angle_adc * 180) / 1024; 
      
      //если середина больше чем 135 градусов
      if ( cal_50 >= 135 ) {
         if ( ( Angle <= cal_50 ) && (Angle >= cal_0 ) ) Ang_true = Angle - cal_0;//0-135
            else
               if ( Angle > cal_50 ) {
                  Ang_true = Angle - cal_0; // 135-270
                  if (Ang_true > 360) Ang_true = Ang_true - 360; // 135-270
               } else 
               if (Angle < cal_0) Ang_true = (270 - cal_100) + Angle;// 135-270
            
      } else
      //если середина меньше 135 градусов
      if (cal_50 < 135) {
         if (Angle > 270) Ang_true = (360 - cal_0) - (360 - Angle); else Ang_true = Angle + (360 - cal_0);
      }
      
      //границы начала и конца 4-20 мА
//      if (Angle < 270) PWM14 = 175 + (Angle * 2); else
//         if (Angle >= 270) PWM14 = 900;
      
      if (Ang_true < 270) PWM14 = 152 + (Ang_true * mul); else
         if (Ang_true >= 270) PWM14 = 900;
       TIM14->CCR1 = PWM14;
      
//      if (PWM14 < 999) PWM14++; else PWM14 = 175;
      
//      if ( (abs(Angle_old - Angle) < 10) && (Angle ) ) 
//      {
//         Angle = (Angle_adc * 180) / 1024; 
//         analog_AO = Angle + 180;
//         Angle_old = Angle;
//      }
//      else
//      {
//         Angle = (Angle_adc * 180) / 1024; 
//         analog_AO = Angle;
//         Angle_old = Angle;
//      }
      
//		 DS3231_ReadDate(buffer);
//		 DS3231_Init(DS3231_control,0);
		 
      
			
			
			
     
			 
//			FPS++;
//			if (Flag_upd) 
//      {
//         Flag_upd = 0;
//         
//      }
//			 Delay_ms(20);
   }

}
