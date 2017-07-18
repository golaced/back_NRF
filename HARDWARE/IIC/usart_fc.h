#ifndef _USART_H
#define _USART_H

#include "stm32f4xx.h"
extern void Send_Data_GOL_LINK(u8 *dataToSend , u8 length);
extern int16_t BLE_DEBUG[16];
extern u8 Rx_Buf[];
void Usart2_Init(u32 br_num);
void Usart2_IRQ(void);
void Usart3_IRQ(void);
void Usart2_Send(unsigned char *DataToSend ,u8 data_num);

void Uart5_Init(u32 br_num);
void Uart5_IRQ(void);
void Uart5_Send(unsigned char *DataToSend ,u8 data_num);
void CPU_LINK_TASK(void);
typedef struct PID_STA{u16 OP,OI,OD,IP,II,ID,YP,YI,YD;}PID_STA;
extern PID_STA HPID,SPID,FIX_PID,NAV_PID;
void UsartSend_M100(uint8_t ch);
typedef struct int16_rcget{
				int16_t ROLL;
				int16_t PITCH;
				int16_t THROTTLE;
				int16_t YAW;
				int16_t AUX1;
				int16_t AUX2;
				int16_t AUX3;
				int16_t AUX4;
				int16_t AUX5;
	      u8 update;
	      u8 connect;
	      u16 lose_cnt,lose_cnt_rx;
				u8 RST;}RC_GETDATA;

extern RC_GETDATA Rc_Get,Rc_Get_PPM,Rc_Get_SBUS;;//接收到的RC数据,1000~2000
				
				
struct _float{
	      float x;
				float y;
				float z;};

struct _int16{
       int16_t x;
	     int16_t y;
	     int16_t z;};		

struct _trans{
     struct _int16 origin;  //原始值
	   struct _float averag;  //平均值
	   struct _float histor;  //历史值
	   struct _int16 quiet;   //静态值
	   struct _float radian;  //弧度值 
          };

struct _alt{
int32_t altitude;
float altitude_f;
int32_t Temperature;
int32_t Pressure;
int Temperat;
};
					
struct _sensor{   
	struct _trans acc;
	struct _trans gyro;
	struct _trans hmc;
	struct _alt alt;
              };

extern struct _sensor sensor;	

	

struct _speed{   
	int altitude;
	int bmp;
	int pitch;
	int roll;
	int gps;
	int filter;
	int sonar;
	int acc;
              };

struct _altitude{   
	int bmp;
	int sonar;
	int gps;
	int acc;
	int filter;
              };
struct _angle{   
float pitch;
float roll;
float yaw;
              };

							
struct _get{   
	struct _speed speed;
	struct _altitude altitude;
	struct _angle AngE;
              };
struct _plane{   
	struct _speed speed;
	struct _altitude altitude;
	struct _get get;
              };

extern struct _plane plane;	
							
							
struct _slam{   
	 int16_t spd[5];
	 int16_t dis[5];
              };
extern struct _slam slam;	
							
							
struct _SPEED_FLOW_NAV{
float west;
float east;
float x;
float y;
float x_f;
float y_f;
};	
struct _POS_FLOW_NAV {
float	east;
float	west;
long LAT;
long LON;
long Weidu_Dig;
long Jingdu_Dig;
u8 flow_qual;
};
struct _POS_GPS_NAV {
long J;
long W;
long X_O;
long Y_O;
long X_UKF;
long Y_UKF;
u8 gps_mode;
u8 star_num;
float spd,angle,yaw;
u8 svnum;
};

struct _FLOW_NAV{
struct _SPEED_FLOW_NAV speed;
struct _SPEED_FLOW_NAV speed_h;	
struct _POS_FLOW_NAV position;
u8 rate;
};	

struct _IMU_NAV{   
struct _FLOW_NAV flow;
struct _POS_GPS_NAV gps;
	
};
extern struct _IMU_NAV imu_nav;
extern float ALT_POS_SONAR_HEAD;

struct _PID2{
float p;
float i;
float d;
float i_max;
float limit;
float dead;	
float dead2;	
};
struct _PID1{
struct _PID2 out;
struct _PID2 in;	
};
struct _PID_SET{
struct _PID1 nav;
struct _PID1 high;
struct _PID1 avoid;
struct _PID1 circle;	
};
extern struct _PID_SET pid;

struct _MODE
{
u8 thr_fix;
u8 en_pid_out_pit;
u8 en_pid_out_rol;
u8 en_pid_out_yaw;
u8 en_pid_fuzzy_p;
u8 en_pid_sb_set;
u8 en_dji_yaw;
u8 set_point1; 
u8 en_hold2_h_off_fix;	
u8 en_flow_break;
u8 rst_h_m100;
u8 en_gps1;	
u8 en_gps;
u8 en_yun_per_off;
u8 en_shoot;
u8 en_fuzzy_angle_pid;
u8 en_sensor_equal_flp;
u8 pit_rol_pid_out_flp;	
u8 en_pid_yaw_angle_control;	
u8 en_pid_yaw_control_by_pit_rol;	
u8 circle_miss_fly;
u8 thr_add_pos;
u8 dj_by_hand;	
u8 en_circle_locate;
u8 en_track_forward;
u8 dj_yaw_follow;
u8 spid;
u8 mpu6050_bw_42;
u8 en_imu_q_update_mine;	
u8 en_moto_smooth;
u8 pid_mode;
u8 no_head;
u8 sonar_avoid;	
u8 yaw_sel;
u8 sb_smooth;
u8 flow_hold_position;
u8 en_circle_nav;
u8 flow_hold_position_use_global;
u8 flow_hold_position_high_fix;
u8 height_safe;
u8 hunman_pid;
u8 yaw_imu_control;	
u8 cal_sel;
u8 flow_sel;
u8 video_save;
u8 height_in_speed;
u8 height_upload;
u8 en_h_mode_switch;
u8 en_dj_cal;
u8 en_sd_save;
u8 hold_use_flow;
u8 en_rth_mine;
u8 en_sonar_avoid;
u8 thr_fix_test;
u8 en_imu_ekf;
u8 att_pid_tune;
u8 high_pid_tune;
u8 dj_lock;
u8 en_eso;
u8 en_eso_h_out;
u8 en_eso_h_in;
u8 en_dji_h;
u8 en_circle_control;
u8 test1;
u8 test2;
u8 test3;
u8 test4;	
u8 mode_fly;
u8 auto_fly_up,auto_land;
u8 en_flow_hold;
u8 debug_without_odroid;
//flow
u8 en_flow_gro_fix;
u8 flow_size;
};
	

typedef struct{
	unsigned int x;//目标的x坐标
	unsigned int y;//目标的y坐标
	unsigned int w;//目标的宽度
	unsigned int h;//目标的高度
	
	u8 check;
}RESULT;//识别结果
extern float angle_imu_dj[3];
extern RESULT color;
extern u8 LOCK, KEY[8],KEY_SEL[4],NAV_BOARD_CONNECT;
extern struct _MODE mode;
extern void GOL_LINK_TASK(void);
extern void SD_LINK_TASK(void);
extern void Usart1_Init(u32 br_num);//SD_board
extern void Usart4_Init(u32 br_num);//-------SD_board
extern void Usart3_Init(u32 br_num);//-------CPU_board
extern u8 cnt_nav_board;
extern u16 data_rate_gol_link;
extern void UART2_ReportMotion(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz);
extern void UART2_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll
,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec);
void SD_LINK_TASK2(u8 sel);
extern void UsartSend_GPS(uint8_t ch);
void Send_IMU_NAV(void);
#define SEND_IMU 0
#define SEND_FLOW 1
#define SEND_GPS 2
#define SEND_ALT 3
extern float rate_gps_board;
void Send_IMU_TO_GPS(void);
#define SEND_BUF_SIZE1 64*2	//发送数据长度,最好等于sizeof(TEXT_TO_SEND)+2的整数倍.
extern u8 SendBuff1[SEND_BUF_SIZE1];	//发送数据缓冲区

void data_per_uart1(int16_t ax,int16_t ay, int16_t az, int16_t gx,int16_t  gy, int16_t gz,int16_t hx, int16_t hy, int16_t hz,
	int16_t yaw,int16_t pitch,int16_t roll,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec);


#define SEND_BUF_SIZE2 40	//发送数据长度,最好等于sizeof(TEXT_TO_SEND)+2的整数倍.
extern u8 SendBuff2[SEND_BUF_SIZE2];	//发送数据缓冲区
void data_per_uart2(void);

#define SEND_BUF_SIZE3 32	//发送数据长度,最好等于sizeof(TEXT_TO_SEND)+2的整数倍.
extern u8 SendBuff3[SEND_BUF_SIZE3];	//发送数据缓冲区
void data_per_uart3(void);

#define SEND_BUF_SIZE4 64	//发送数据长度,最好等于sizeof(TEXT_TO_SEND)+2的整数倍.
extern u8 SendBuff4[SEND_BUF_SIZE4];	//发送数据缓冲区
void data_per_uart4(u8 sel);

//nav_board
struct _IMU{
		float pitch;
		float roll;
		float yaw;
	  int gro_x;
		int gro_y;
		int gro_z;
		int acc_x;
		int acc_y;
		int acc_z;
		int q0;
		int q1;
		int q2;
		int q3;
		long J;
		long W;
		int H;
	      };

struct _SENSOR{
	int accx;
	int accy;
	int accz;
	int grox;
	int groy;
	int groz;
	int hmx;
	int hmy;
	int hmz;
	int hmx_c;
	int hmy_c;
	int hmz_c;
	int bmp;
	int temp;
	int sonar;
	      };

struct _FUSHION{
	int alt;
	int spd;
	int alt_acc_bmp;
	int alt_acc_sonar;
	int spd_acc_bmp;
	int spd_acc_sonar;
	      };
			
						
struct _SET{
float pitch;
float roll;
float yaw;
int alt;
int alt_bmp;
int alt_fushion;
int spd_x;
int spd_y;
int pos[2];
int pos_t[2];
int spd[2];
int spd_f[2];	
float nav[2];
float spd_alt;
int thr;
int alt_sonar;	
int alt_fushion_sonar;

		};

struct _STATE{
u8 alt_data ;
u8 gps ;
u8 flow;
u8 imu_nav;
u8 pid_fuzzy ;
		};	

struct _PID{
u16 pp_o;
u16 pi_o;
u16 pd_o;
u16 pp_i;
u16 pi_i;
u16 pd_i;

u16 rp_o;
u16 ri_o;
u16 rd_o;
u16 rp_i;
u16 ri_i;
u16 rd_i;

u16 yp_o;
u16 yi_o;
u16 yd_o;
u16 yp_i;
u16 yi_i;
u16 yd_i;

u16 hp_o;
u16 hi_o;
u16 hd_o;
u16 hp_i;
u16 hi_i;
u16 hd_i;

		};	

struct _PIDOUT{
int pitch;
int roll;
int yaw;
int alt;
int spd_x;
int spd_y;
		};	

struct _TRAJ{
float drone1[3];
float drone2[3];
float drone3[3];
float drone4[3];
		};	
		
struct _FLY{
				struct _IMU imu;    
				struct _SENSOR sensor;  
				struct _SET set; 
				struct _SET flow; 	
				struct _SET now;  
	      struct _STATE state;
	      struct _PID pid;
	      struct _PIDOUT pid_out;
				struct _POS_GPS_NAV gps;
	      struct _TRAJ traj;
	      int slam_sonar[5];
	      u8 fame;
            };

extern struct _FLY fly_controller;
						
#define SEND_NRF_RC 0
#define SEND_NRF_RC_PPMSBUS 1
#define SEND_NRF_PID 3						
void data_per_uart1_dma(u8 sel);
void clear_nrf_uart(void);
extern u16 nrf_uart_cnt;					
extern float mark[10][4];						
extern float sd_save[100];
void Send_RC_TO_FC(u8 sel);					

extern u8 cal_rc,mems_state,gps_state;						
#endif
