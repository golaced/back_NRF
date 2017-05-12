#include "include.h" 
#include "iic_soft.h"
#include "hml_sample.h"
#include "ms5611.h"
#include "ms5611_2.h"
#include "hml5833l.h"
#include "alt_kf.h"
#include "flash.h"
#include "led_fc.h"
#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"
#include "ucos_task.h"
#include "pwm_in.h"
#include "flow.h"
#include "circle.h"
#include "eso.h"
#include "gps.h"
#include "m100.h"
#include "sd.h"
#include "sbus.h"



OS_STK INNER_TASK_STK[INNER_STK_SIZE];
void inner_task(void *pdata)
{NVIC_InitTypeDef NVIC_InitStructure;
 u8 i;
 static u8 dj_fly_line=0;
 static u8 init;	
 static int flag_scan=1;
 	while(1)
  {
	}
}		



//========================外环  任务函数============================
OS_STK OUTER_TASK_STK[OUTER_STK_SIZE];
float outer_loop_time;
float Pitch_R,Roll_R,Yaw_R;
void outer_task(void *pdata)
{	static u8 cnt,cnt1,cnt2;						  
 	while(1)
	{	
	outer_loop_time = Get_Cycle_T(GET_T_OUTTER);								//获取外环准确的执行周期
	delay_ms(5);
	}
}		
//=========================射频 任务函数======================
OS_STK NRF_TASK_STK[NRF_STK_SIZE];
u8 en_shoot=0;
void nrf_task(void *pdata)
{							 
	static u8 cnt,cnt2;
 	while(1)
	{
  	Nrf_Check_Event();
	
		delay_ms(20);
		if(cnt++>1){cnt=0;
		RC_Send_Task();}
	}
}		

#include "sdio.h"
//=======================光流 任务函数==================
OS_STK FLOW_TASK_STK[FLOW_STK_SIZE];
float sd_time;
u16 sd_dt=80;
void flow_task(void *pdata)
{static u8 state,sd_insert_reg,cnt;
 static float hc_speed_i[2],h_speed[2],wz_speed_0[2],tempacc_lpf[2];				
 float c_nb_dtb[3][3],a_br[3],tmp[3],acc[3];	
 	while(1)
	{
			
  sd_check();
		
	switch (state)
	{
		case 0:
			if(sd_insert==1&&sd_had_init==0)
			{ power_sd(1);
			
				if(!SD_Init())
				{state=1;
				exfuns_init();							//为fatfs相关变量申请内存				 
				SD_INIT();
				sd_had_init=1;
				}
			}
			if(sd_insert==0)
			sd_had_init=0;

		break;
		case 1:
			if(sd_insert==0){
		  sd_had_init=0;
			state=0;
			power_sd(0);	
			}
	
	  break;
	}		 
	sd.sd_insert=sd_insert;	
	sd.init=sd_had_init;
	en_save=(sd.en_save&&sd_insert)||(sd.en_save_force&&sd_insert);
		
	sd.task_detal = Get_Cycle_T(GET_T_SD);	
	
	SD_Save_Task(sd.task_detal);	
	delay_ms(sd_dt);
	}
}	
	
//=======================M100 任务函数==================
OS_STK M100_TASK_STK[M100_STK_SIZE];

u8 en_vrc;
u8 m100_control_mode = 0x4A;
float k_m100[5]={1,1,1,1,1};//pit rol thr yaw avoid
void m100_task(void *pdata)
{		
	static u8 cnt_m100;
		static u8 en_vrcr,flag1;
	static int m100_Rc_gr;
	
		while(1)
	{
	if(cnt_m100++>2-1){cnt_m100=0;
	m100_data(0);
	
		
	if(mode.auto_fly_up==1&&m100_Rc_gr==0)
	{
		m100_obtain_control_long(10);
		m100_take_off_long(10);
	}
  
	if(m100_Rc_gr==1&&mode.auto_fly_up==0)
	{m100_land_control_long(10);}
	m100_Rc_gr=mode.auto_fly_up;
	
  if(m100.Rc_gear<=-9000)en_vrc=1;
  if(en_vrc&&m100.Rc_gear>-5000)
	en_vrc=0;
	if(en_vrc)
	{   		
	m100_obtain_control(10);
//	if(mode.en_dji_yaw)
//  m100_contrl(Rc_Pwm_Out_mine_USE[1],Rc_Pwm_Out_mine_USE[0],Rc_Pwm_Out_mine_USE[2],Rc_Pwm_Out_mine_USE[3],m100_control_mode); 
//  else	
//	m100_contrl(Rc_Pwm_Out_mine_USE[1],Rc_Pwm_Out_mine_USE[0],Rc_Pwm_Out_mine_USE[2],1500,m100_control_mode);
	delay_ms(20); }
	
	if(en_vrc==0&&en_vrcr==1)
		{m100_dis_control_long(5);}

		en_vrcr=en_vrc;
		
	if(dji_rst)
		m100_rst(10);
	}
	delay_ms(5);
	}
}
#include "filter.h"
//=======================串口 任务函数===========================
OS_STK  UART_TASK_STK[UART_STK_SIZE];
u8 UART_UP_LOAD_SEL=4;//<------------------------------UART UPLOAD DATA SEL
u8 state_v_test=0;
u8 num_need_to_check;
u8 save_rc;
void uart_task(void *pdata)
{	static u8 cnt[4],state,sd_save_reg;	
  static u8 sd_sel;	
	u16 temp;
 	while(1)
	{		
		if(save_rc)
		{save_rc=0;
			WRITE_PARM();
		}
		temp=(Moving_Median(20,10,channels[0])-SBUS_MID)*500/((SBUS_MAX-SBUS_MIN)/2)+1500;
		if(temp>900&&temp<2100)
		Rc_Get_SBUS.ROLL=		 temp;
		temp=(Moving_Median(21,10,channels[1])-SBUS_MID)*500/((SBUS_MAX-SBUS_MIN)/2)+1500;
		if(temp>900&&temp<2100)
		Rc_Get_SBUS.PITCH=		 temp;
		temp=(Moving_Median(22,10,channels[2])-SBUS_MID)*500/((SBUS_MAX-SBUS_MIN)/2)+1500;
		if(temp>900&&temp<2100)
		Rc_Get_SBUS.THROTTLE=		 temp;
		temp=(Moving_Median(23,10,channels[3])-SBUS_MID)*500/((SBUS_MAX-SBUS_MIN)/2)+1500;
		if(temp>900&&temp<2100)
		Rc_Get_SBUS.YAW=		 temp;
		temp=(Moving_Median(24,5,channels[4])-SBUS_MID_A)*500/((SBUS_MAX_A-SBUS_MIN_A)/2)+1500;
		if(temp>900&&temp<2100)
		Rc_Get_SBUS.AUX1=		 temp;
		temp=(Moving_Median(25,5,channels[5])-SBUS_MID_A)*500/((SBUS_MAX_A-SBUS_MIN_A)/2)+1500;
		if(temp>900&&temp<2100)
		Rc_Get_SBUS.AUX2=		 temp;
		temp=(Moving_Median(26,5,channels[6])-SBUS_MID_A)*500/((SBUS_MAX_A-SBUS_MIN_A)/2)+1500;
		if(temp>900&&temp<2100)
		Rc_Get_SBUS.AUX3=		 temp;
		temp=(Moving_Median(27,5,channels[7])-SBUS_MID_A)*500/((SBUS_MAX_A-SBUS_MIN_A)/2)+1500;
		if(temp>900&&temp<2100)
		Rc_Get_SBUS.AUX4=		 temp;

    if(sd.en_save==0){
		Rc_Get_PPM.ROLL=		(Moving_Median(11,5,ppm_rx[1])-PWM_PPM_MID)*500/((PWM_PPM_MAX-PWM_PPM_MIN)/2)+1500;
		Rc_Get_PPM.PITCH=		(Moving_Median(12,5,ppm_rx[2])-PWM_PPM_MID)*500/((PWM_PPM_MAX-PWM_PPM_MIN)/2)+1500;
		Rc_Get_PPM.THROTTLE=(Moving_Median(13,5,ppm_rx[3])-PWM_PPM_MID)*500/((PWM_PPM_MAX-PWM_PPM_MIN)/2)+1500;
		Rc_Get_PPM.YAW=			(Moving_Median(14,5,ppm_rx[4])-PWM_PPM_MID)*500/((PWM_PPM_MAX-PWM_PPM_MIN)/2)+1500;
		Rc_Get_PPM.AUX1=		(Moving_Median(15,5,ppm_rx[5])-PWM_PPM_MID)*500/((PWM_PPM_MAX-PWM_PPM_MIN)/2)+1500;
		Rc_Get_PPM.AUX2=		(Moving_Median(16,5,ppm_rx[6])-PWM_PPM_MID)*500/((PWM_PPM_MAX-PWM_PPM_MIN)/2)+1500;
		Rc_Get_PPM.AUX3=		(Moving_Median(17,5,ppm_rx[7])-PWM_PPM_MID)*500/((PWM_PPM_MAX-PWM_PPM_MIN)/2)+1500;
		Rc_Get_PPM.AUX4=		(Moving_Median(18,5,ppm_rx[8])-PWM_PPM_MID)*500/((PWM_PPM_MAX-PWM_PPM_MIN)/2)+1500;
		Rc_Get_PPM.AUX5=		(Moving_Median(19,5,ppm_rx[9])-PWM_PPM_MID)*500/((PWM_PPM_MAX-PWM_PPM_MIN)/2)+1500;
		}
	  else
		{
		Rc_Get_PPM.THROTTLE=620;
		}
		sd_save_reg=sd.en_save;
		delay_ms(10);  
	}
}	


//------------------------------软件定时器--------------------------------//
OS_TMR   * tmr1;			//软件定时器1
OS_TMR   * tmr2;			//软件定时器2
OS_TMR   * tmr3;			//软件定时器3

//软件定时器1的回调函数	
//每100ms执行一次,用于显示CPU使用率和内存使用率		
 u16 cpuusage=0;
void tmr1_callback(OS_TMR *ptmr,void *p_arg) 
{
	static u8 tcnt=0;	    

	if(tcnt==5)
	{
 		
		cpuusage=0;
		tcnt=0; 
	}
	cpuusage+=OSCPUUsage;
	tcnt++;				    
}

//软件定时器2的回调函数				  50ms	 
u8 dji_rst_protect;
u8 dji_rst;
u8 DJI_CONNECT;
u16 dji_miss_cnt;
u8 dji_rc_miss;
u16 dji_rc_miss_cnt;
float time_led;
void tmr2_callback(OS_TMR *ptmr,void *p_arg) 
{	
static u16 cnt_1,cnt_2;	
static u8 cnt;
	time_led = Get_Cycle_T(GET_T_LED); 		
	LEDRGB_STATE();
	
	if(fabs(m100.Pit)>90||fabs(m100.Rol)>90)
	{cnt_1=0;dji_rst_protect=1;}
	
	if(cnt_1>3/0.05)
	{cnt_1=dji_rst_protect=0;}
	else if(dji_rst_protect)
		cnt_1++;
	
	if(dji_miss_cnt++>1/0.05)
	{DJI_CONNECT=0;dji_miss_cnt=65530;}
	if(m100.Rc_mode<-7000&&DJI_CONNECT)
	{cnt_2=0;dji_rst=1;}
	
	if(cnt_2>3/0.05)
	{cnt_2=dji_rst=0;}
	else if(dji_rst)
		cnt_2++;
	
	if(cnt_m100_data_refresh++>0.88/0.05)
	{	cnt_m100_data_refresh=65530;
	  m100_data_refresh=0;}
	
		
	if(dji_rc_miss_cnt++>1/0.05)
	{dji_rc_miss=1;dji_rc_miss_cnt=65530;}
	if(m100.Rc_gear!=0)
	{dji_rc_miss_cnt=0;dji_rc_miss=0;}	
	
  if(Rc_Get_PPM.lose_cnt++>4/0.05)
		Rc_Get_PPM.update=0;
	
	 if(Rc_Get_SBUS.lose_cnt++>4/0.05)
		Rc_Get_SBUS.connect=0;
	 
	 	if(Rc_Get.lose_cnt++>4/0.05)
		Rc_Get.update=0;
	
}
//软件定时器3的回调函数				  	   
void tmr3_callback(OS_TMR *ptmr,void *p_arg) 
{	
 
} 


//