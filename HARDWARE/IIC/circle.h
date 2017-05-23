#ifndef __CIRCLE_H
#define __CIRCLE_H	 
#include "stm32f4xx.h" 
#include "include.h" 
#include "gps.h"
 typedef struct
{
 int x,y;
 int r;
 int x_flp,y_flp;
 u8 check;
 u8 connect,lose_cnt;
 int control[2];
 float control_k;
 float control_k_miss; 
	float control_yaw;
 float forward;
 float forward_end_dj_pwm;
	u8 dj_fly_line;
}CIRCLE;
extern CIRCLE circle,track,mouse;
extern float nav_circle[2],nav_land[2];
void circle_control(float T);
#define MID_Y 125
#define MID_X 140
extern float circle_use[2];
extern float  integrator[2];
void  GPS_hold(nmea_msg *gpsx_in,float T);


#endif











