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
altUkfStruct_t altUkfData,altUkfData_sonar,altUkfData_bmp;
float ALT_POS_SONAR2,q_nav[4],ALT_VEL_BMP_EKF;
float Roll,Yaw,Pitch;
u32 Rc_Pwm_Inr_mine[8];
u32 Rc_Pwm_In_mine[8],Rc_Pwm_Out_mine[8];
PWMIN pwmin;
//==============================������ ������==========================
OS_STK MEMS_TASK_STK[MEMS_STK_SIZE];
void mems_task(void *pdata)
{		static u8 cnt,cnt1;						 
 	while(1)
	{
	}
}		
//--------------------

OS_STK INNER_TASK_STK[INNER_STK_SIZE];
u16 Rc_Pwm_In[8];
float PWM_DJ[4]={1615,1500,0};
float inner_loop_time;
float dj_k=0.0018*1.8,dj_k2=0.01015,dj_k_mouse=0.015;

//float k_dj[2][3]={1.6,0.8,3,  3.5,0.5,10};float d_t[2]={3,4.5};
float k_dj[2][3]={1,1.0,0.85,  1,1,0.85};float d_t[2]={0.45,0.45/2};float dt_flt=0.9;
float flt_track[3]={0.75,0.9,0.5};
float DJ_YAW_OFF=0;
int Rc_Pwm_off[8]={2,2,2,3};

float k_yaw_z=0.2;
float k_reset=10;
u8 state_v_test2=13;
float dead_pan_z=4;
float k_pan_z=0.04;
u8 rc_thr_mid=0;
float flt_gro_z=0.8;
float Yaw_Follow_Dead= 25/2;
#if USE_M100
float SHOOT_PWM_OFF0=-66,
#else
float SHOOT_PWM_OFF0=70,
#endif
	SHOOT_PWM_OFF1=6,SHOOT_PWM_DEAD0=80,SHOOT_PWM_DEAD1=60;
#if USE_M100
int YUN_PER_OFF=50;
#else
int YUN_PER_OFF=40;
#endif
float T_SHOOT_CHECK=0.4;//2;
u16 DJ_TEST[3]={1523,1520,1500};
float set_angle_dj[3];
float kp_dj[3]={10,50,8},kd_dj[3],i_yaw;
float k_scan=0.25;//0.25;
u8 SCAN_RANGE=88;//116;
int flag_yun[2]={-1,1};
u16 Rc_Pwm_Out_mine_USE[4];
float k_m100_gps[3]=  {2.26,2.26,0.66}; //p r t
float k_m100_scan[3]= {2.26,2.26,0.66};
float k_m100_track[3]={2.26,0.88,0.66};
float k_m100_shoot[3]={2.26,0.88,0.66};
float k_m100_laser_avoid=0.3888;
float k_m100_yaw=1;
void inner_task(void *pdata)
{NVIC_InitTypeDef NVIC_InitStructure;
 u8 i;
 static u8 dj_fly_line=0;
 static u8 init;	
 static int flag_scan=1;
 	while(1)
	{
	inner_loop_time = Get_Cycle_T(GET_T_INNER); 						//��ȡ�ڻ�׼ȷ��ִ������
	RC_Duty( inner_loop_time , Rc_Pwm_In );	
	CTRL_1( inner_loop_time ); 										//�ڻ����ٶȿ��ơ����룺ִ�����ڣ��������ٶȣ��������ٶȣ��Ƕ�ǰ������������PWMռ�ձȡ�<����δ��װ>
		
	if(!init){init=1;	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PWMIN_P;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PWMIN_S;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PWMIN_P;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PWMIN_S;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	 
	}
	//------------------------------������Ƴ�ʼ��---------------------------------------------------
	
	#if USE_M100
	Rc_Pwm_Inr_mine[RC_PITCH]=(float)LIMIT(m100.Rc_pit,-10000,10000)/10000.*500+1500;	
	Rc_Pwm_Inr_mine[RC_ROLL] =(float)LIMIT(m100.Rc_rol,-10000,10000)/10000.*500+1500;	
	Rc_Pwm_Inr_mine[RC_YAW]=  (float)LIMIT(m100.Rc_yaw,-10000,10000)/10000.*500+1500;	
	Rc_Pwm_Inr_mine[RC_THR]=  (float)LIMIT(m100.Rc_thr,-10000,10000)/10000.*500+1500;	
	Rc_Pwm_Inr_mine[RC_MODE]= (float)LIMIT(m100.Rc_mode,-10000,10000)/8000.*500+1500;	
	Rc_Pwm_Inr_mine[RC_GEAR]= (float)LIMIT(m100.Rc_gear,-10000,10000)/10000.*500+1500;	
	
	for(i=0;i<8;i++){	
	if(Rc_Pwm_Inr_mine[i]<=pwmin.max+200&&Rc_Pwm_Inr_mine[i]>=pwmin.min-200&&i!=RC_THR)
	Rc_Pwm_Out_mine[i]=0.5*Rc_Pwm_Out_mine[i]+0.5*Rc_Pwm_Inr_mine[i];
	}
	#else
	for(i=0;i<8;i++){	
	if(Rc_Pwm_In_mine[i]<=pwmin.max+200&&Rc_Pwm_In_mine[i]>=pwmin.min-200){
	if(i!=RC_THR)//avoid THR 
	Rc_Pwm_Out_mine[i]=0.5*Rc_Pwm_Out_mine[i]+0.5*Rc_Pwm_In_mine[i];
	Rc_Pwm_Inr_mine[i]=Rc_Pwm_In_mine[i];}
	}
	#endif
	//----------------------------------------------------------------------------------------------
	#if USE_M100
	#define MAX_NAV_RC 200//180
	#else
	#define MAX_NAV_RC 180//180
	#endif
	#define DEAD_NAV_RC 100
	#define USE_YAW 1
	#define USE_CIRCLE 1
	#if USE_CIRCLE
	int temp1=(int)Rc_Pwm_Inr_mine[RC_PITCH]-OFF_RC_PIT;
	int temp2=(int)Rc_Pwm_Inr_mine[RC_ROLL]-OFF_RC_ROL;
	int temp3=(int)Rc_Pwm_Inr_mine[RC_YAW]-OFF_RC_YAW;
	
	#if USE_M100
	
 if(state_v==SU_CHECK_TAR||state_v==SU_TO_START_POS||state_v==SD_TO_HOME)
	 {
	 k_m100[0]=k_m100_gps[0];
	 k_m100[1]=k_m100_gps[1];	 
	 }
	 else if(state_v==SU_TO_CHECK_POS)
	 {
	 k_m100[0]=k_m100_gps[0];
	 if(fabs(nav_Data.gps_ero_dis_lpf[1])>366*2)	 
	 k_m100[1]=k_m100_gps[1]*1.5;
   else	 
	 k_m100[1]=k_m100_gps[1];
	 } 
	 else if(state_v==SD_HOLD||state_v==SD_HOLD_BACK)
	 {
	 k_m100[0]=k_m100_scan[0];
	 if(fabs(nav_Data.gps_ero_dis_lpf[1])>366*2)	 
	 k_m100[1]=k_m100_scan[1]*1.5;	 
	 else
	 k_m100[1]=k_m100_scan[1]; 
	 	 }
	  else if(state_v==SD_HOLD2)
	 {
	 k_m100[0]=k_m100_track[0];
	 k_m100[1]=k_m100_track[1];	 
	 	 }
	 else if(state_v==SD_SHOOT)
	 {
	  k_m100[0]=k_m100_track[0];
		#if SHOOT_USE_YUN	 
		k_m100[1]=k_m100_track[1];
		#else
		if((fabs(ultra_ctrl_head.err1)<86)&&fabs(PWM_DJ[1]-PWM_DJ1)<25)	 
		k_m100[1]=k_m100_shoot[1];
		else
		k_m100[1]=k_m100_track[1];
		#endif	
	 	 }
	 else
	 {
	 k_m100[0]=1;
	 k_m100[1]=1;
	 }
	#else
   k_m100[0]=1;
	 k_m100[1]=1;
	#endif
	
	
	
//��Բ���� 
	if((fabs(temp1)<DEAD_NAV_RC)&&(fabs(temp2)<DEAD_NAV_RC)&&(fabs(temp3)<DEAD_NAV_RC))
	{ 
		#if USE_M100
		if(!mode.dj_by_hand&&!dji_rst_protect&&ALT_POS_SONAR2>0.3){
		#else
		if(!mode.dj_by_hand&&ALT_POS_SONAR2>0.3){//ע��---����
		#endif
		Rc_Pwm_Out_mine[RC_PITCH]=LIMIT(nav_land[PITr]*k_m100[0]+OFF_RC_PIT,OFF_RC_PIT-MAX_NAV_RC,OFF_RC_PIT+MAX_NAV_RC);//ע��ң��ƫִ
		Rc_Pwm_Out_mine[RC_ROLL] =LIMIT(nav_land[ROLr]*k_m100[1]+OFF_RC_ROL,OFF_RC_ROL-MAX_NAV_RC,OFF_RC_ROL+MAX_NAV_RC);
		Rc_Pwm_Out_mine[RC_YAW]  =LIMIT(yaw_ctrl_out*k_m100_yaw+OFF_RC_YAW,OFF_RC_YAW-MAX_NAV_RC,OFF_RC_YAW+MAX_NAV_RC);	
		}
	}
	#endif
	// ��̨����
	static u8 dj_mode_reg;
	if((mode.dj_by_hand&&!dj_mode_reg))
	{
   PWM_DJ[0]=PWM_DJ0;//( 1 / ( 1 + 1 / (k_reset*3.14f *0.01 ) ) ) * ( (float)(1250) -  PWM_DJ[0] );
	 PWM_DJ[1]=PWM_DJ1;//( 1 / ( 1 + 1 / (k_reset*3.14f *0.01 ) ) ) * ( (float)(1500) -  PWM_DJ[1] );
	}
	if(mouse.check)
	{
		PWM_DJ[0]+=0.02*my_deathzoom(mouse.y-120,5)*flag_yun[0];
		PWM_DJ[1]+=-0.01*my_deathzoom(mouse.x-160,5)*flag_yun[1];	
	}	
	else if(mode.dj_by_hand){

	if(Rc_Get.ROLL>1000)
	PWM_DJ[0]+=-dj_k*my_deathzoom(Rc_Get.ROLL-1500,80)*flag_yun[0];
	if(Rc_Get.PITCH>1000)
	PWM_DJ[1]+=-dj_k*my_deathzoom(Rc_Get.PITCH-1500,80)*flag_yun[1];	

	}
	else if(state_v==SU_CHECK_TAR)//�½���Բ ��ֱ��̨
	{
	 PWM_DJ[0]=PWM_DJ0;//( 1 / ( 1 + 1 / (k_reset*3.14f *0.01 ) ) ) * ( (float)(1830) -  PWM_DJ[0] );
	
   if(PWM_DJ[1]<PWM_DJ1-SCAN_RANGE)	
	 { flag_scan=1;PWM_DJ[1]=PWM_DJ1-SCAN_RANGE+5; }
   else if(PWM_DJ[1]>PWM_DJ1+SCAN_RANGE)	
	 { flag_scan=-1;PWM_DJ[1]=PWM_DJ1+SCAN_RANGE-5;}
	 PWM_DJ[1]+=flag_scan*k_scan*flag_yun[1];//( 1 / ( 1 + 1 / (k_reset*3.14f *0.01 ) ) ) * ( (float)(1500) -  PWM_DJ[1] );
	}
	else  if(state_v==SD_HOLD)
			{
	 PWM_DJ[0]=PWM_DJ0;//( 1 / ( 1 + 1 / (k_reset*3.14f *0.01 ) ) ) * ( (float)(1830) -  PWM_DJ[0] );
	 PWM_DJ[1]=PWM_DJ1-YUN_PER_OFF*mode.en_yun_per_off;//( 1 / ( 1 + 1 / (k_reset*3.14f *0.01 ) ) ) * ( (float)(1500) -  PWM_DJ[1] );
	}
	#if !SHOOT_USE_YUN
  else if(state_v==SD_SHOOT)
	{
	if((fabs(ultra_ctrl_head.err1)<86)&&fabs(PWM_DJ[1]-PWM_DJ1)<88)
   PWM_DJ[1]=PWM_DJ1;
	}
	#endif
		else if(state_v==SD_HOLD_BACK)
			{
	 PWM_DJ[0]=PWM_DJ0;//( 1 / ( 1 + 1 / (k_reset*3.14f *0.01 ) ) ) * ( (float)(1830) -  PWM_DJ[0] );
   PWM_DJ[1]=PWM_DJ1+YUN_PER_OFF*mode.en_yun_per_off;//( 1 / ( 1 + 1 / (k_reset*3.14f *0.01 ) ) ) * ( (float)(1500) -  PWM_DJ[1] );
	}
	else if(track.check&&circle.connect&&(state_v==SD_HOLD_BREAK||state_v==SD_HOLD2||state_v==SG_LOW_CHECK||state_v==SD_SHOOT)){
	float ero[2],ero2[2],ero3[2],ero4[2];
  static float ero_r[2],ero_r2[2],ero_r3[2];
  ero[0]=my_deathzoom_2(-circle.y+120,30);//,0.5,(float)68/1200);
	ero2[0]=my_deathzoom_2(-circle.y+120,20);//,0.5,(float)68/1200);
	ero3[0]=my_deathzoom_2(-circle.y+120,10);//,0.5,(float)68/1200);	
	ero4[0]=my_deathzoom_2(-circle.y+120,5);//,0.5,(float)68/1200);	
	PWM_DJ[0]+=-(dj_k2*ero[0]*track.control_k)*k_dj[0][0]*flag_yun[0];	//k=2.2
	PWM_DJ[0]+=-(dj_k2*ero2[0])*track.control_k*k_dj[0][1]*flag_yun[0];
	PWM_DJ[0]+=-(dj_k2*ero3[0])*track.control_k*k_dj[0][2]*flag_yun[0];	
	PWM_DJ[0]+=-(dj_k2*ero4[0])*track.control_k*k_dj[0][2]*flag_yun[0];		
	//if(fabs(-circle.y+120)<20)	
		
	PWM_DJ[0]+=-(ero_r2[0])*d_t[0]*flag_yun[0];
  ero_r2[0]=	(-circle.y+120-ero_r[0])*dt_flt+(1-dt_flt)*ero_r3[0]	;
  ero_r3[0]=ero_r2[0];
	ero_r[0]=-circle.y+120;
//-------------------------------------------------------------------//
	ero[1]=my_deathzoom_2(circle.x-160,35);//,0.5,(float)68/1200);
	ero2[1]=my_deathzoom_2(circle.x-160,25);//,0.5,(float)68/1200);
	ero3[1]=my_deathzoom_2(circle.x-160,15);//,0.5,(float)68/1200);	
	ero4[1]=my_deathzoom_2(circle.x-160,5);//,0.5,(float)68/1200);
	PWM_DJ[1]+=-(dj_k2*ero[1]*track.control_k_miss)*k_dj[1][0]*flag_yun[1];	//k=27
	PWM_DJ[1]+=-(dj_k2*ero2[1])*track.control_k_miss*k_dj[1][1]*flag_yun[1];
	PWM_DJ[1]+=-(dj_k2*ero3[1])*track.control_k_miss*k_dj[1][2]*flag_yun[1];
	PWM_DJ[1]+=-(dj_k2*ero4[1])*track.control_k_miss*k_dj[1][2]*flag_yun[1];
 // if(fabs(circle.x-160)<20)	
	PWM_DJ[1]+=-(ero_r2[1])*d_t[1]*flag_yun[1];
  ero_r2[1]=	(circle.x-160-ero_r[1])*dt_flt+(1-dt_flt)*ero_r3[1]	;
	ero_r3[1]=ero_r2[1];
	ero_r[1]=circle.x-160;
	}
	static float temp_gro_z;
	temp_gro_z=flt_gro_z*temp_gro_z+(1-flt_gro_z);//*mpu6050.Gyro_deg.z;
	if((track.check&&circle.connect)&&!mode.dj_by_hand)
		PWM_DJ[1]+=-my_deathzoom(temp_gro_z,dead_pan_z)*k_pan_z;//yaw fix
	
	static u16 cnt_miss_track;
	if(!mode.dj_by_hand&&(state_v==SD_HOLD2||state_v==SD_SHOOT||state_v==SG_LOW_CHECK))
	{
	if(cnt_miss_track++>6.5/0.005)//hold2
	{cnt_miss_track=0;
	PWM_DJ[0]=PWM_DJ0;
	PWM_DJ[1]=PWM_DJ1;}
	else if(circle.check&&circle.connect)
	cnt_miss_track=0;
	}
	else
	cnt_miss_track=0;
	

	//PWM_DJ[0]=DJ_TEST[0];//Pitch_DJ
	//PWM_DJ[1]=DJ_TEST[1];//ROLL_DJ
	
	/*duoji*/
		float out[3];
	float ero_dj[3];
	static float ero_djr[3];
	
	set_angle_dj[0]=PWM_DJ[0];
	set_angle_dj[2]=PWM_DJ[1];
	
	
	ero_dj[0]=my_deathzoom(set_angle_dj[0]+Pitch_DJ,1);
	
	out[0]=ero_dj[0]*kp_dj[0]+(ero_dj[0]-ero_djr[0])*kd_dj[0];
	
	ero_dj[1]=my_deathzoom(set_angle_dj[1]+Roll,1);
	if(fabs(ero_dj[1])<10)
	out[1]=ero_dj[1]*kp_dj[1]+(ero_dj[1]-ero_djr[1])*kd_dj[1];
	else 	if(fabs(ero_dj[1])<20)
	out[1]=ero_dj[1]*kp_dj[1]*4+(ero_dj[1]-ero_djr[1])*kd_dj[1];
	else
	out[1]=ero_dj[1]*kp_dj[1]*8+(ero_dj[1]-ero_djr[1])*kd_dj[1];
	
	ero_dj[2]=my_deathzoom(set_angle_dj[2]-Yaw,1.5);
	static u16 cnt_dj_delay;
	if(cnt_dj_delay++<200){cnt_dj_delay=202;set_angle_dj[2]=Yaw;}
	//
	
	if(fabs(ero_dj[2])>20)set_angle_dj[2]=Yaw;
	
	static float int_dj;
	 int_dj+=i_yaw*ero_dj[2];
	out[2]=ero_dj[2]*kp_dj[2]+(ero_dj[2]-ero_djr[2])*kd_dj[2]+int_dj;
	
	
	#define YAW_PWM_RANGE 250
	PWM_DJ[0]=LIMIT(PWM_DJ[0],1020,1980);PWM_DJ[1]=LIMIT(PWM_DJ[1],1500-YAW_PWM_RANGE,1500+YAW_PWM_RANGE);
	Rc_Pwm_Out_mine[4]=(1-flt_track[1])*Rc_Pwm_Out_mine[4]+(flt_track[1])*PWM_DJ[0];//out put pit
//-----------------------
	
//	SHOOT_PWM_OFF1=((Rc_Get.AUX4-500)/10)*0.8;//��̨���ƫִ

	
//	if(mode.en_yun_per_off)
//	{
////	  if(state_v==SD_HOLD)
////			Rc_Pwm_Out_mine[5]=(1-flt_track[1])*Rc_Pwm_Out_mine[5]+(flt_track[1])*PWM_DJ[1]+DJ_YAW_OFF+SHOOT_PWM_OFF1-YUN_PER_OFF;//out put yaw
////		else if(state_v==SD_HOLD_BACK)
////			Rc_Pwm_Out_mine[5]=(1-flt_track[1])*Rc_Pwm_Out_mine[5]+(flt_track[1])*PWM_DJ[1]+DJ_YAW_OFF+SHOOT_PWM_OFF1+YUN_PER_OFF;//out put yaw
////		else
//			Rc_Pwm_Out_mine[5]=(1-flt_track[1])*Rc_Pwm_Out_mine[5]+(flt_track[1])*PWM_DJ[1]+DJ_YAW_OFF+SHOOT_PWM_OFF1;//out put yaw
//	}	
//	else
	Rc_Pwm_Out_mine[5]=(1-flt_track[1])*Rc_Pwm_Out_mine[5]+(flt_track[1])*PWM_DJ[1]+DJ_YAW_OFF+SHOOT_PWM_OFF1;//out put yaw
	
		
//--------------------Yaw follow  
 #if SHOOT_USE_YUN
	if(fabs((int)PWM_DJ[1]-1500)<Yaw_Follow_Dead)
	{track.dj_fly_line=1;
	PWM_DJ[2]=LIMIT(-track.control_yaw*(my_deathzoom(PWM_DJ[1]-PWM_DJ1,0))+0,0-100,0+100);	
		}
	else{
	PWM_DJ[2]=LIMIT(-track.control_yaw*(my_deathzoom(PWM_DJ[1]-PWM_DJ1,0))+0,0-100,0+100);
		track.dj_fly_line=0;
	}
	#else
	if(
		#if !DEBUG_IN_ROOM
		(fabs(ultra_ctrl_head.err1)<86)&&
	  #endif
		fabs(PWM_DJ[1]-PWM_DJ1)<100)
	PWM_DJ[2]=LIMIT(track.control_yaw*my_deathzoom_2(circle.x-160,5)+0,0-100,0+100);
	else{
	if(fabs((int)PWM_DJ[1]-1500)<Yaw_Follow_Dead)
	{track.dj_fly_line=1;
	PWM_DJ[2]=LIMIT(-track.control_yaw*(my_deathzoom(PWM_DJ[1]-PWM_DJ1,0))+0,0-100,0+100);	
		}
	else{
	PWM_DJ[2]=LIMIT(-track.control_yaw*(my_deathzoom(PWM_DJ[1]-PWM_DJ1,0))+0,0-100,0+100);
		track.dj_fly_line=0;
	}}
	
	#endif
//---------------ң������ť	
/*	
	SHOOT_PWM_DEAD1=Rc_Get.AUX2/10;//����//0~1000
	SHOOT_PWM_DEAD0=Rc_Get.AUX1/10;//����
	#if USE_M100
	SHOOT_PWM_OFF0=-((float)Rc_Get.AUX3)*2/10.;
	#else
	SHOOT_PWM_OFF0=-((float)Rc_Get.AUX3)/10.;
	#endif
*/
	//if(mode.dj_yaw_follow&&!mode.dj_by_hand&&fabs((int)Rc_Pwm_In_mine[RC_YAW]-1500)<DEAD_NAV_RC&&track.check&&circle.connect&&state_v==13)
	//Rc_Pwm_Out_mine[RC_YAW]=PWM_DJ[2];//(1-flt_track[2])*Rc_Pwm_Out_mine[RC_YAW]+(flt_track[2])*PWM_DJ[2];
	
	//SetPwm(Rc_Pwm_Out_mine,Rc_Pwm_off,pwmin.min,pwmin.max);

	dj_mode_reg=mode.dj_by_hand;
	
	Rc_Pwm_Out_mine_USE[0]=Rc_Pwm_Out_mine[0];
	Rc_Pwm_Out_mine_USE[1]=Rc_Pwm_Out_mine[1];
	Rc_Pwm_Out_mine_USE[2]=Rc_Pwm_Out_mine[2];
	Rc_Pwm_Out_mine_USE[3]=Rc_Pwm_Out_mine[3];
	delay_ms(5);
	}
}		



//========================�⻷  ������============================
OS_STK OUTER_TASK_STK[OUTER_STK_SIZE];
float outer_loop_time;
float Pitch_R,Roll_R,Yaw_R;
void outer_task(void *pdata)
{	static u8 cnt,cnt1,cnt2;						  
 	while(1)
	{	
	outer_loop_time = Get_Cycle_T(GET_T_OUTTER);								//��ȡ�⻷׼ȷ��ִ������
	/*IMU������̬�����룺���ִ�����ڣ��������������ݣ�ת������ÿ�룩��������ٶȼ����ݣ�4096--1G���������ROLPITYAW��̬��*/
 	//IMUupdate(0.5f *outer_loop_time,mpu6050.Gyro_deg.x, mpu6050.Gyro_deg.y, mpu6050.Gyro_deg.z, mpu6050.Acc.x, mpu6050.Acc.y, mpu6050.Acc.z,&Roll_R,&Pitch_R,&Yaw_R);		
 	//CTRL_2( outer_loop_time ); 														// �⻷�Ƕȿ��ơ����룺ִ�����ڣ������Ƕȣ�ҡ����������̬�Ƕȣ�������������ٶȡ�<����δ��װ>
  
  if(cnt1++>9){cnt1=0;GPS_calc_poshold(); 
	//circle_control(2*outer_loop_time);
	}
	#if USE_M100
	if(cnt2++>4-1){
	#else
	if(cnt2++>40-1){
	#endif	
		cnt2=0;GPS_hold(&gpsx,0.2); 
	}
	delay_ms(5);
	}
}		
//=========================��Ƶ ������======================
OS_STK NRF_TASK_STK[NRF_STK_SIZE];
u8 en_shoot=0;
void nrf_task(void *pdata)
{							 
	static u8 cnt,cnt2;
 	while(1)
	{
  	Nrf_Check_Event();
		//------------0 1   |   2 3  KEY_SEL
		
		mode.dj_by_hand=!KEY_SEL[0];//�ֶ�������̨
		mode.en_track_forward=1;//KEY_SEL[1];//ʹ�ܲ��
		mode.en_pid_sb_set=KEY_SEL[2];//ʹ��PID����	
		mode.test3=1;//ǰ�����
		mode.set_point1=KEY_SEL[3];//���ú���
		mode.en_hold2_h_off_fix=1;//=KEY_SEL[3];//���������߶�
//		if(mode.en_gps)
//		mode.en_dji_h=0;//KEY_SEL[3];	
//		else
			mode.en_dji_h=1;
			//------------7 6 5 4  |  3 2 1 0  KEY
		mode.en_flow_break=1;
		mode.en_shoot= 1;//
		//mode.en_gps=KEY[7];//1
		
		
		force_check_pass=KEY[7];//ǿ������ͼ���жϲ��ֳ���  Ϊ��odroid gpsѲ��������
		//mode.auto_fly_up=KEY[5];

		mode.en_rth_mine=1;
		mode.en_yun_per_off=1;//���Ԥƫ
		//mode.en_dji_yaw=KEY[4];
		mode.hold_use_flow=1;//KEY[4];//en lock height
		//Mag_CALIBRATED=KEY[3];//avoid with laser
		
		#if USE_M100
	  mode.en_dji_yaw=0;//KEY[7];
		mode.en_dji_h=1;//KEY[7];//h
		mode.test3=1;//KEY_SEL[2];//avoid
		mode.en_flow_break=0;
		mode.en_gps=1;
		//force_check_pass=0;
		mode.hold_use_flow=0;		
		mode.en_rth_mine=1;//KEY[7];
		#endif
		
		//EN_SHOOT(en_shoot||KEY[2]);
			
		//-----
		#if  DEBUG_WITHOUT_SB
		if(cnt2++>150)//
		{fly_ready=1;cnt2=151;}
		#else
		if(Rc_Pwm_Inr_mine[RC_THR]>200)
		fly_ready=1;
		else
		fly_ready=0;//KEY_SEL[3];//����
		#endif
		delay_ms(20);
		if(cnt++>1){cnt=0;
		RC_Send_Task();}
	}
}		

//��ѹ�� ������
OS_STK BARO_TASK_STK[BARO_STK_SIZE];
void baro_task(void *pdata)
{							  
 	while(1)
	{ static u8 cnt;
		delay_ms(10);  
	}
}	

//=======================������ ������==================
OS_STK SONAR_TASK_STK[SONAR_STK_SIZE];
void sonar_task(void *pdata)
{							  
 	while(1)
	{if(1){

	}
	delay_ms(100);  
	}
}	

#include "sdio.h"
//=======================���� ������==================
OS_STK FLOW_TASK_STK[FLOW_STK_SIZE];
#define ACC_SPEED_NUM 25
u8 ACC_SPEED_NUM_USE=10;
u16 acc_cnt[2];
float acc_v[3];
float acc_speed_arr[ACC_SPEED_NUM + 1];
float wz_speed_flow[2];
float w_acc_spd=0.915;
float w_acc_fix=0.1;
float sd_time;
u16 sd_dt=100;
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
				exfuns_init();							//Ϊfatfs��ر��������ڴ�				 
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
	
	//if(cnt++>0){cnt=0;	
	sd.task_detal = Get_Cycle_T(GET_T_SD);		
	SD_Save_Task(sd.task_detal);	
	//}
	delay_ms(sd_dt);
	}
}	
	
//=======================M100 ������==================
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
	
  if(m100.Rc_gear<=-9000&&ALT_POS_SONAR2>0.32666)en_vrc=1;
  if(en_vrc&&m100.Rc_gear>-5000)
	en_vrc=0;
	if(en_vrc)
	{   		
	m100_obtain_control(10);
	if(mode.en_dji_yaw)
  m100_contrl(Rc_Pwm_Out_mine_USE[1],Rc_Pwm_Out_mine_USE[0],Rc_Pwm_Out_mine_USE[2],Rc_Pwm_Out_mine_USE[3],m100_control_mode); 
  else	
	m100_contrl(Rc_Pwm_Out_mine_USE[1],Rc_Pwm_Out_mine_USE[0],Rc_Pwm_Out_mine_USE[2],1500,m100_control_mode);
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
//=======================���� ������===========================
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
		temp=(Moving_Median(20,5,channels[0])-SBUS_MID)*500/((SBUS_MAX-SBUS_MIN)/2)+1500;
		if(temp>900&&temp<2100)
		Rc_Get_SBUS.ROLL=		 temp;
		temp=(Moving_Median(21,5,channels[1])-SBUS_MID)*500/((SBUS_MAX-SBUS_MIN)/2)+1500;
		if(temp>900&&temp<2100)
		Rc_Get_SBUS.PITCH=		 temp;
		temp=(Moving_Median(22,5,channels[2])-SBUS_MID)*500/((SBUS_MAX-SBUS_MIN)/2)+1500;
		if(temp>900&&temp<2100)
		Rc_Get_SBUS.THROTTLE=		 temp;
		temp=(Moving_Median(23,5,channels[3])-SBUS_MID)*500/((SBUS_MAX-SBUS_MIN)/2)+1500;
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
//    delay_ms(2500);
//    else{		
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
		static u8 sel;
		switch(sel){
			case 0:Send_RC_TO_FC(0);
			sel=1;
		  break;
			case 1:Send_RC_TO_FC(1);
			sel=0;
		  break;
		}
//		if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)!=RESET)//�ȴ�DMA2_Steam7�������
//							{ 	
//						  DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);//���DMA2_Steam7������ɱ�־
//							clear_nrf_uart();
//							switch(sel){
//								case 0:;//data_per_uart1_dma(SEND_NRF_RC);sel=1;	
//									break;
//								case 1:
//									   sel=0;
//								      //data_per_uart1_dma(SEND_NRF_RC_PPMSBUS);
//										//data_per_uart1_dma(SEND_NRF_RC);
//										if(cnt[0]++>2){
//											cnt[0]=0;	
//										;//data_per_uart1_dma(SEND_NRF_PID);	
//										}
//								  break;
//							}
//							
//							USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  //ʹ�ܴ���1��DMA����     
//							MYDMA_Enable(DMA2_Stream7,nrf_uart_cnt+2);     //��ʼһ��DMA���䣡	  
//							}	
	}
}	


//------------------------------�����ʱ��--------------------------------//
OS_TMR   * tmr1;			//�����ʱ��1
OS_TMR   * tmr2;			//�����ʱ��2
OS_TMR   * tmr3;			//�����ʱ��3

//�����ʱ��1�Ļص�����	
//ÿ100msִ��һ��,������ʾCPUʹ���ʺ��ڴ�ʹ����		
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
 #include "circle.h"
//�����ʱ��2�Ļص�����				  50ms	 
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
	Mode_FC();
	LEDRGB_STATE();
	if(circle.lose_cnt++>4/0.05)
		circle.connect=0;
	if(mouse.lose_cnt++>2/0.05)
		mouse.connect=0;
	
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
//�����ʱ��3�Ļص�����				  	   
void tmr3_callback(OS_TMR *ptmr,void *p_arg) 
{	
 
} 


//