

#include "led_fc.h"
#include "include.h"
#include "mpu6050.h"
#include "hml5833l.h"
void LEDRGB_COLOR(u8 color);
void LED_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(ANO_RCC_LED,ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Pin   = ANO_Pin_LED1| ANO_Pin_LED2| ANO_Pin_LED3;
	GPIO_Init(ANO_GPIO_LED, &GPIO_InitStructure);
	
	
	 //SD insert
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Pin   =GPIO_Pin_0 ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0| GPIO_Pin_1| GPIO_Pin_2|GPIO_Pin_3;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	power_sd(1);
	LEDRGB_COLOR(BLUE);
	Delay_ms(500);
	LEDRGB_COLOR(RED);
	Delay_ms(500);
	LEDRGB_COLOR(GREEN);
  Delay_ms(500);

}
u8 sd_insert;
void sd_check(void)
{
sd_insert=	!GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0);
}
void power_sd(u8 sel)
{
if(sel)
	GPIO_SetBits(GPIOC,GPIO_Pin_0| GPIO_Pin_1| GPIO_Pin_2|GPIO_Pin_3);
else
	GPIO_ResetBits(GPIOC,GPIO_Pin_0| GPIO_Pin_1| GPIO_Pin_2|GPIO_Pin_3);
}	
/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/


void LEDRGB(u8 sel,u8 on)
{
switch(sel)
{
case BLUE:
if(!on)
GPIO_ResetBits(GPIOA,GPIO_Pin_4);
else
GPIO_SetBits(GPIOA,GPIO_Pin_4);
break;
case RED:
if(!on)
GPIO_ResetBits(GPIOA,GPIO_Pin_5);
else
GPIO_SetBits(GPIOA,GPIO_Pin_5);
break;
case GREEN:
if(!on)
GPIO_ResetBits(GPIOA,GPIO_Pin_6);
else
GPIO_SetBits(GPIOA,GPIO_Pin_6);
break;
}
}

void LEDRGB_COLOR(u8 color)
{
switch (color)
{
case RED:
LEDRGB(RED,1);
LEDRGB(BLUE,0);
LEDRGB(GREEN,0);
break;
case BLUE:
LEDRGB(RED,0);
LEDRGB(BLUE,1);
LEDRGB(GREEN,0);
break;
case GREEN:
LEDRGB(RED,0);
LEDRGB(BLUE,0);
LEDRGB(GREEN,1);
break;
case WHITE:
LEDRGB(RED,1);
LEDRGB(BLUE,1);
LEDRGB(GREEN,1);
break;
case BLACK:
LEDRGB(RED,0);
LEDRGB(BLUE,0);
LEDRGB(GREEN,0);
break;
case YELLOW:
LEDRGB(RED,1);
LEDRGB(BLUE,0);
LEDRGB(GREEN,1);
break;
}
}
#define IDLE 0
#define CAL_MPU 1
#define CAL_M  2
#include "circle.h"
#include "sd.h"
void LEDRGB_STATE(void)
{
static u8 main_state;
static u8 mpu_state,m_state,idle_state;
static u16 cnt,cnt_idle;
u8 mode_control;
	
	
mode_control=mode.mode_fly;
//   | | | |    | | | |   | | | |   | | | |   | | | |
//    ARM          GPS1     GPS2      GPS3      MODE  
#define RGB_DELAY 3
static u8 cnt_gps;
static u8 flag_cnt_gps;
if(en_save)
LEDRGB_COLOR(WHITE); 	
else {
main_state=0;
switch(idle_state)
{//ARM
	case 0:
		if(main_state==IDLE)
			{idle_state=1;cnt_idle=0;}
	break;
	case 1:
	 if(1)//state_v==SG_LOW_CHECK)
				LEDRGB_COLOR(WHITE); 
		 else
				LEDRGB_COLOR(RED);
	if(cnt_idle++>0.1/0.05)
	{idle_state=2;cnt_idle=0;}
	break;
	case 2:
		LEDRGB_COLOR(BLACK);	
	if(cnt_idle++>0.1/0.05)
	{idle_state=3;cnt_idle=0;}
	break;
	
	case 3:
		if(Rc_Get_PPM.update&&Rc_Get.update)
				LEDRGB_COLOR(WHITE); 
		 else if(Rc_Get.update)
				LEDRGB_COLOR(GREEN); 
		 else if(Rc_Get_PPM.update)
				LEDRGB_COLOR(BLUE); 
		 else
				LEDRGB_COLOR(RED);
	if(cnt_idle++>0.1/0.05)
	{idle_state=4;cnt_idle=0;}
	break;
	case 4:
		LEDRGB_COLOR(BLACK);	
	if(cnt_idle++>0.1/0.05)
	{idle_state=5;cnt_idle=0;}
	break;
	
	case 5:
	 if(m100_data_refresh&&!dji_rc_miss&&m100.GPS_STATUS>=3)
				LEDRGB_COLOR(WHITE); 
		 else if(m100_data_refresh&&!dji_rc_miss)
			  LEDRGB_COLOR(YELLOW);
		 else
				LEDRGB_COLOR(RED);
	if(cnt_idle++>0.1/0.05)
	{idle_state=6;cnt_idle=0;}
	break;
	case 6:
		LEDRGB_COLOR(BLACK);	
	if(cnt_idle++>1.2/0.05)
	{idle_state=0;cnt_idle=0;}
	break;
}
}
}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

