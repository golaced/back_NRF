


#ifndef _M100_H
#define _M100_H

#include "stm32f4xx.h"
#include "circle.h"
void m100_disarm(u16 delay);
void m100_arm(u16 delay);
void m100_activate(u16 delay);
void m100_activate_long(u8 times);
void m100_obtain_control(u16 delay);
void m100_obtain_control_long(u8 times);
void m100_dis_control(u16 delay);
void m100_dis_control_long(u8 times);
void m100_rth(u16 delay);
void m100_take_off(u16 delay);
void m100_take_off_long(u8 times);
void m100_land_control(u16 delay);
void m100_land_control_long(u8 times);
void m100_vRc_on_A(u16 delay);
void m100_vRc_on_F(u16 delay);
void m100_vRc_off(u16 delay);
void m100_contrl(float x,float y,float z,float yaw,u8 mode);
void m100_data(u16 delay);
void m100_rst(u16 delay);


extern u8 dji_rst_protect;
extern u8 dji_rst;
extern u8 DJI_CONNECT;
extern u16 dji_miss_cnt;
extern u8 dji_rc_miss;
extern u16 dji_rc_miss_cnt;
extern u16 cnt_m100_data_refresh;
extern u8 m100_data_refresh;
#endif

