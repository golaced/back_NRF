
#include "include.h"
#include "usart_fc.h"
#include "ultrasonic.h"
#include "hml5833l.h"
#include "mpu6050.h"
#include "ms5611.h"
#include "rc.h"
#include "att.h"
#include "height_ctrl.h"
#include "alt_kf.h"
#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"
#include "flow.h"
#include "gps.h"
#include "circle.h"
#include "sd.h"
#include "sbus.h"
 u8 SendBuff1[SEND_BUF_SIZE1];	//发送数据缓冲区
 u8 SendBuff4[SEND_BUF_SIZE4];	//发送数据缓冲区
void Usart2_Init(u32 br_num)//--GOL-link
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
	
	//配置PD5作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
	//配置PD6作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = br_num;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART2, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART2, ENABLE);  //使能串口1 
	
	USART_ClearFlag(USART2, USART_FLAG_TC);

	//使能USART2接收中断
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	//使能USART2
	USART_Cmd(USART2, ENABLE); 
}

RC_GETDATA Rc_Get,Rc_Get_PPM,Rc_Get_SBUS;
struct _plane plane;
struct _slam slam;
struct _IMU_NAV imu_nav;
struct _MODE mode;
u8  KEY[8],KEY_SEL[4];

 void Data_IMU(u8 *data_buf,u8 num)
{
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
  if(*(data_buf+2)==0x01)//FLOW_MINE_frame
  {
	}		
	else if(*(data_buf+2)==0x11)//Att
  { 
	}		
}
u8 laser_sel=1;

u8 TxBuffer[256];
u8 TxCounter=0;
u8 count=0; 

u8 Rx_Buf[256];	//串口接收缓存
u8 RxBuffer[50];
u8 RxState = 0;
u8 RxBufferNum = 0;
u8 RxBufferCnt = 0;
u8 RxLen = 0;
u8 com_data ;
static u8 _data_len = 0,_data_cnt = 0;
void USART2_IRQHandler(void)
{ OSIntEnter(); 
	u8 com_data;
	
	if(USART2->SR & USART_SR_ORE)//ORE中断
	{
		com_data = USART2->DR;
	}

  //接收中断
	if( USART_GetITStatus(USART2,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);//清除中断标志

		com_data = USART2->DR;
				if(RxState==0&&com_data==0xAA)
		{
			RxState=1;
			RxBuffer[0]=com_data;
		}
		else if(RxState==1&&com_data==0xAF)
		{
			RxState=2;
			RxBuffer[1]=com_data;
		}
		else if(RxState==2&&com_data>0&&com_data<0XF1)
		{
			RxState=3;
			RxBuffer[2]=com_data;
		}
		else if(RxState==3&&com_data<50)
		{
			RxState = 4;
			RxBuffer[3]=com_data;
			_data_len = com_data;
			_data_cnt = 0;
		}
		else if(RxState==4&&_data_len>0)
		{
			_data_len--;
			RxBuffer[4+_data_cnt++]=com_data;
			if(_data_len==0)
				RxState = 5;
		}
		else if(RxState==5)
		{
			RxState = 0;
			RxBuffer[4+_data_cnt]=com_data;
			Data_IMU(RxBuffer,_data_cnt+5);
		}
		else
			RxState = 0;
	}
	//发送（进入移位）中断
	if( USART_GetITStatus(USART2,USART_IT_TXE ) )
	{
				
		USART2->DR = TxBuffer[TxCounter++]; //写DR清除中断标志          
		if(TxCounter == count)
		{
			USART2->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}
	}

   OSIntExit(); 
}


void UsartSend_GOL_LINK(uint8_t ch)
{

	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
USART_SendData(USART1, ch); 
}

void Send_Data_GOL_LINK(u8 *dataToSend , u8 length)
{
u16 i;
  for(i=0;i<length;i++)
     UsartSend_GOL_LINK(dataToSend[i]);
}

void UsartSend_M100(uint8_t ch)
{

	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
USART_SendData(USART1, ch); 
}

void Usart1_Init(u32 br_num)//-------UPload_board1
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	
	//配置PD5作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
	//配置PD6作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = br_num;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART1, ENABLE);  //使能串口1 
	
	USART_ClearFlag(USART1, USART_FLAG_TC);
	

	//使能USART2接收中断
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	//使能USART2
	USART_Cmd(USART1, ENABLE); 
}


#include "gps.h"

nmea_msg gpsx,gps_data; 		
u8  buf_GPRMC[100]={'G','P','R','M','C',','};//GPS信息
u8  buf_GPRMCt[100]={"GPRMC,144326.00,A,5107.0017737,N,11402.3291611,W,0.080,323.3,210307,0.0,E,A*20"};//GPS信息
u8  buf_GPGGA[100]={'G','P','G','G','A',','};//GPS信息
u8  buf_GPGGAt[100]={"GPGGA,134658.00,5106.9792,N,11402.3003,W,2,09,1.0,1048.47,M,-16.27,M,08,AAAA*60"};//GPS信息
u8  buf_imu_dj[20];
float angle_imu_dj[3];

u16 cnt_m100_data_refresh;
u8 m100_data_refresh;
float m100_update_time;
 void DATA_M100(u8 *data_buf,u8 num)
{double zen,xiao;
	vs16 rc_value_temp;
	static float m100_hr,m100_attr[3];
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
  if(*(data_buf+2)==0x51)//
  { dji_miss_cnt=0;
		DJI_CONNECT=1;
		m100_update_time = Get_Cycle_T(GET_T_M100);		
	  m100.Pit=(float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5))/10.;
		m100.Rol=(float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7))/10.;
		m100.Yaw=1*(float)((int16_t)(*(data_buf+8)<<8)|*(data_buf+9))/10.;
		
		m100.H=(float)((int16_t)(*(data_buf+10)<<8)|*(data_buf+11))/1000.;
		
		if(m100.H!=m100_hr||m100_attr[0]!=m100.Pit||m100_attr[1]!=m100.Rol||m100_attr[2]!=m100.Yaw)
		{cnt_m100_data_refresh=0;
		 m100_data_refresh=1;
		}
		m100_hr=m100.H;
		m100_attr[0]=m100.Pit;
		m100_attr[1]=m100.Rol;
		m100_attr[2]=m100.Yaw;
		
		m100.H_Spd=(float)((int16_t)(*(data_buf+12)<<8)|*(data_buf+13))/1000.;
		zen=(*(data_buf+14)<<8)|*(data_buf+15);
		xiao=(double)((u32)(*(data_buf+16)<<24)|(*(data_buf+17)<<16)|(*(data_buf+18)<<8)|*(data_buf+19))/1000000000.;
		m100.Lat=zen+xiao;
		zen=(*(data_buf+20)<<8)|*(data_buf+21);
		xiao=(double)((u32)(*(data_buf+22)<<24)|(*(data_buf+23)<<16)|(*(data_buf+24)<<8)|*(data_buf+25))/1000000000.;
		m100.Lon=zen+xiao;
		
		m100.Bat=(float)((int16_t)(*(data_buf+26)<<8)|*(data_buf+27))/100.;
		m100.Rc_pit=(float)((int16_t)(*(data_buf+28)<<8)|*(data_buf+29));
		m100.Rc_rol=(float)((int16_t)(*(data_buf+30)<<8)|*(data_buf+31));
		m100.Rc_yaw=(float)((int16_t)(*(data_buf+32)<<8)|*(data_buf+33));
		m100.Rc_thr=(float)((int16_t)(*(data_buf+34)<<8)|*(data_buf+45));
		m100.Rc_mode=(float)((int16_t)(*(data_buf+36)<<8)|*(data_buf+37));
		m100.Rc_gear=(float)((int16_t)(*(data_buf+38)<<8)|*(data_buf+39));
		m100.STATUS=*(data_buf+40);		
		m100.GPS_STATUS=*(data_buf+41);
	}			
}


void data_check_float(float *data,float *data_r,float in,float min,float max)
{
if(in<max&&in>min)
	{*data=in;*data_r=in;}
}

void data_check_int(int *data,int* data_r,int in,float min,float max)
{
if(in<max&&in>min)
	{*data=in;*data_r=in;}
}
//use now
float mark[10][4];
float sd_save[100];
float flow_matlab_data[4],baro_matlab_data[4],qr_matlab_data[4],qr_matlab_data_att[3];
struct _FLY fly_controller,fly_controller_r;
int k_scale_pix;
u8 en_save;
int16_t BLE_DEBUG[16];
 void Data_Receive_Anl_FC(u8 *data_buf,u8 num)//fly_board
{
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i,j;

	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		
	{
		j++;return;	
	}		//判断帧头
	if(*(data_buf+2)==0x05)								//判断功能字0x80 飞控IMU数据
	{
	     
			data_check_float(&fly_controller.imu.roll,&fly_controller_r.imu.roll,(float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/10,-180,180);
			data_check_float(&fly_controller.imu.pitch , &fly_controller_r.imu.pitch,(float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/10,-180,180);
			data_check_float(&fly_controller.imu.yaw, &fly_controller_r.imu.yaw,(float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/10,-180,180);
			data_check_int(&fly_controller.imu.gro_x, &fly_controller_r.imu.gro_x,((vs16)(*(data_buf+10)<<8)|*(data_buf+11)),-1024,1024);
			data_check_int(&fly_controller.imu.gro_y, &fly_controller_r.imu.gro_y,((vs16)(*(data_buf+12)<<8)|*(data_buf+13)),-1024,1024);
			data_check_int(&fly_controller.imu.gro_z, &fly_controller_r.imu.gro_z,((vs16)(*(data_buf+14)<<8)|*(data_buf+15)),-1024,1024);
			
			data_check_int(&fly_controller.imu.acc_x, &fly_controller_r.imu.acc_x,((vs16)(*(data_buf+16)<<8)|*(data_buf+17)),-6000,6000);
			data_check_int(&fly_controller.imu.acc_y, &fly_controller_r.imu.acc_y,((vs16)(*(data_buf+18)<<8)|*(data_buf+19)),-6000,6000);
			data_check_int(&fly_controller.imu.acc_z, &fly_controller_r.imu.acc_z,((vs16)(*(data_buf+20)<<8)|*(data_buf+21)),-6000,6000);
			
			data_check_int(&fly_controller.imu.q0,		&fly_controller_r.imu.q0,((vs16)(*(data_buf+22)<<8)|*(data_buf+23)),-2000,2000);
			data_check_int(&fly_controller.imu.q1, 		&fly_controller_r.imu.q1,((vs16)(*(data_buf+24)<<8)|*(data_buf+25)),-2000,2000);
			data_check_int(&fly_controller.imu.q2, 		&fly_controller_r.imu.q2,((vs16)(*(data_buf+26)<<8)|*(data_buf+27)),-2000,2000);
			data_check_int(&fly_controller.imu.q3, 		&fly_controller_r.imu.q3,((vs16)(*(data_buf+28)<<8)|*(data_buf+29)),-2000,2000);
		 //  en_save = *(data_buf+30);//sd_使能存储		

	}
	else if(*(data_buf+2)==0x01)								//判断功能字0x82 飞控姿态PID控制量
	{
	
			fly_controller.set.pitch = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/10;
			fly_controller.set.roll = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/10;
			fly_controller.set.yaw=(float) ((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/10;
	
	}
	else if(*(data_buf+2)==0x02)								//飞控光流PID控制
	{  fly_controller.set.pos[0]= ((vs16)(*(data_buf+4)<<8)|*(data_buf+5));
		 fly_controller.set.pos[1]= ((vs16)(*(data_buf+6)<<8)|*(data_buf+7));
		 fly_controller.now.pos[0]= ((vs16)(*(data_buf+8)<<8)|*(data_buf+9));
		 fly_controller.now.pos[1]= ((vs16)(*(data_buf+10)<<8)|*(data_buf+11));
		 fly_controller.set.spd[0]= ((vs16)(*(data_buf+12)<<8)|*(data_buf+13));
		 fly_controller.set.spd[1]= ((vs16)(*(data_buf+14)<<8)|*(data_buf+15));
		 fly_controller.now.spd[0]= ((vs16)(*(data_buf+16)<<8)|*(data_buf+17));
		 fly_controller.now.spd[1]= ((vs16)(*(data_buf+18)<<8)|*(data_buf+19));
		 fly_controller.now.nav[0]= (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/10;
		 fly_controller.now.nav[1]= (float)((vs16)(*(data_buf+22)<<8)|*(data_buf+23))/10;
	}
		else if(*(data_buf+2)==0x03)								//飞控高度PID控制
	{
	
			data_check_int(&fly_controller.set.alt ,&fly_controller_r.set.alt , ((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/10,-10*100,10*100);
		  data_check_int(&fly_controller.now.alt ,&fly_controller_r.now.alt , ((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/10,-10*100,10*100);
		  data_check_float(&fly_controller.set.spd_alt,&fly_controller_r.set.spd_alt , ((vs16)(*(data_buf+8)<<8)|*(data_buf+9)),-500,500);
	    fly_controller.now.spd_alt = ((vs16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.now.thr = ((vs16)(*(data_buf+12)<<8)|*(data_buf+13));
		  data_check_int(&fly_controller.now.alt_bmp ,&fly_controller_r.now.alt_bmp , ((vs16)(*(data_buf+14)<<8)|*(data_buf+15)),-10100,10100);
		  data_check_int(&fly_controller.now.alt_fushion ,&fly_controller_r.now.alt_fushion , ((vs16)(*(data_buf+16)<<8)|*(data_buf+17)),-10100,10100);
		
	}
			else if(*(data_buf+2)==0x04)								//飞控模式
	{
	
			en_save = *(data_buf+4);//sd_使能存储		
	}
			else if(*(data_buf+2)==0x06)								//飞控使用光流数据
	{
	
			 fly_controller.flow.spd_f[0] = ((vs16)(*(data_buf+4)<<8)|*(data_buf+5));
	     fly_controller.flow.spd_f[1] = ((vs16)(*(data_buf+6)<<8)|*(data_buf+7));
			 fly_controller.flow.spd[0] =  ((vs16)(*(data_buf+8)<<8)|*(data_buf+9));
	     fly_controller.flow.spd[1] =  ((vs16)(*(data_buf+10)<<8)|*(data_buf+11));
		   fly_controller.flow.pos_t[0] = ((vs16)(*(data_buf+12)<<8)|*(data_buf+13));
	     fly_controller.flow.pos_t[1] = ((vs16)(*(data_buf+14)<<8)|*(data_buf+15));
		   fly_controller.flow.pos[0] = ((vs16)(*(data_buf+16)<<8)|*(data_buf+17));
	     fly_controller.flow.pos[1] = ((vs16)(*(data_buf+18)<<8)|*(data_buf+19));
	
		
	}
				else if(*(data_buf+2)==0x07)								//飞控使用光流数据
	{
			
		  mode.en_sd_save= *(data_buf+28);
		  sd.en_save= mode.en_sd_save;
	}
				else if(*(data_buf+2)==0x08)								//飞控使用GPS数据
	{
	BLE_DEBUG[0]=((vs16)(*(data_buf+4)<<8)|*(data_buf+5));
	BLE_DEBUG[1]=((vs16)(*(data_buf+6)<<8)|*(data_buf+7));
	BLE_DEBUG[2]=((vs16)(*(data_buf+8)<<8)|*(data_buf+9));
	BLE_DEBUG[3]=((vs16)(*(data_buf+10)<<8)|*(data_buf+11));
	BLE_DEBUG[4]=((vs16)(*(data_buf+12)<<8)|*(data_buf+13));
	BLE_DEBUG[5]=((vs16)(*(data_buf+14)<<8)|*(data_buf+15));
	BLE_DEBUG[6]=((vs16)(*(data_buf+16)<<8)|*(data_buf+17));
	BLE_DEBUG[7]=((vs16)(*(data_buf+18)<<8)|*(data_buf+19));
	BLE_DEBUG[8]=((vs16)(*(data_buf+20)<<8)|*(data_buf+21));
	BLE_DEBUG[9]=((vs16)(*(data_buf+22)<<8)|*(data_buf+23));
	BLE_DEBUG[10]=((vs16)(*(data_buf+24)<<8)|*(data_buf+25));
	BLE_DEBUG[11]=((vs16)(*(data_buf+26)<<8)|*(data_buf+27));
	BLE_DEBUG[12]=((vs16)(*(data_buf+28)<<8)|*(data_buf+29));
	BLE_DEBUG[13]=((vs16)(*(data_buf+30)<<8)|*(data_buf+31));
	BLE_DEBUG[14]=((vs16)(*(data_buf+32)<<8)|*(data_buf+33));
	BLE_DEBUG[15]=((vs16)(*(data_buf+34)<<8)|*(data_buf+35));		
	}
				else if(*(data_buf+2)==0x09)								//飞控使用光流数据
	{
	
			 mark[0][0] = ((vs16)(*(data_buf+4)<<8)|*(data_buf+5));
	     mark[0][1] = ((vs16)(*(data_buf+6)<<8)|*(data_buf+7));
			 mark[0][2] =  ((vs16)(*(data_buf+8)<<8)|*(data_buf+9));
	     mark[1][0] =  ((vs16)(*(data_buf+10)<<8)|*(data_buf+11));
		   mark[1][1] = ((vs16)(*(data_buf+12)<<8)|*(data_buf+13));
	     mark[1][2] = ((vs16)(*(data_buf+14)<<8)|*(data_buf+15));
		   mark[2][0] = ((vs16)(*(data_buf+16)<<8)|*(data_buf+17));
	     mark[2][1] = ((vs16)(*(data_buf+18)<<8)|*(data_buf+19));
			 mark[2][2]= ((vs16)(*(data_buf+20)<<8)|*(data_buf+21));
			 mark[3][0]= ((vs16)(*(data_buf+22)<<8)|*(data_buf+23));
			 mark[3][1]= ((vs16)(*(data_buf+24)<<8)|*(data_buf+25));
			 mark[3][2]= ((vs16)(*(data_buf+26)<<8)|*(data_buf+27));
		 
		   mark[0][3]= *(data_buf+28);
		   mark[1][3]= *(data_buf+29);
		   mark[2][3]= *(data_buf+30);
		   mark[3][3]= *(data_buf+31);
	}
	//---------------------------------------------------------------------
	else if(*(data_buf+2)==0x71)								//判断功能字0x81 飞控Sensor数据
	{
			fly_controller.sensor.accx = ((vs16)(*(data_buf+4)<<8)|*(data_buf+5));
			fly_controller.sensor.accy = ((vs16)(*(data_buf+6)<<8)|*(data_buf+7));
			fly_controller.sensor.accz= ((vs16)(*(data_buf+8)<<8)|*(data_buf+9));
			fly_controller.sensor.grox = ((vs16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.sensor.groy = ((vs16)(*(data_buf+12)<<8)|*(data_buf+13));
			fly_controller.sensor.groz = ((vs16)(*(data_buf+14)<<8)|*(data_buf+15));
			fly_controller.sensor.hmx = ((vs16)(*(data_buf+16)<<8)|*(data_buf+17));
			fly_controller.sensor.hmy = ((vs16)(*(data_buf+18)<<8)|*(data_buf+19));
			fly_controller.sensor.hmz = ((vs16)(*(data_buf+20)<<8)|*(data_buf+21));
		  fly_controller.sensor.bmp = ((vs16)(*(data_buf+22)<<8)|*(data_buf+23));
		  fly_controller.sensor.temp = ((vs16)(*(data_buf+24)<<8)|*(data_buf+25));
		  fly_controller.sensor.sonar = ((vs16)(*(data_buf+26)<<8)|*(data_buf+27));
	}
		else if(*(data_buf+2)==0x74)								//判断功能字0x82 PID参数
	{
	
			fly_controller.pid.pp_o = ((u16)(*(data_buf+4)<<8)|*(data_buf+5));
			fly_controller.pid.pi_o = ((u16)(*(data_buf+6)<<8)|*(data_buf+7));
			fly_controller.pid.pd_o= ((u16)(*(data_buf+8)<<8)|*(data_buf+9));
			fly_controller.pid.pp_i = ((u16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.pid.pi_i =((u16)(*(data_buf+12)<<8)|*(data_buf+13));
			fly_controller.pid.pd_i =((u16)(*(data_buf+14)<<8)|*(data_buf+15));
		
		  fly_controller.pid.rp_o = ((u16)(*(data_buf+4)<<8)|*(data_buf+5));
			fly_controller.pid.ri_o = ((u16)(*(data_buf+6)<<8)|*(data_buf+7));
			fly_controller.pid.rd_o= ((u16)(*(data_buf+8)<<8)|*(data_buf+9));
			fly_controller.pid.rp_i = ((u16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.pid.ri_i =((u16)(*(data_buf+12)<<8)|*(data_buf+13));
			fly_controller.pid.rd_i =((u16)(*(data_buf+14)<<8)|*(data_buf+15));
		
		  fly_controller.pid.yp_o = ((u16)(*(data_buf+4)<<8)|*(data_buf+5));
			fly_controller.pid.yi_o = ((u16)(*(data_buf+6)<<8)|*(data_buf+7));
			fly_controller.pid.yd_o= ((u16)(*(data_buf+8)<<8)|*(data_buf+9));
			fly_controller.pid.yp_i = ((u16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.pid.yi_i =((u16)(*(data_buf+12)<<8)|*(data_buf+13));
			fly_controller.pid.yd_i =((u16)(*(data_buf+14)<<8)|*(data_buf+15));
			
			fly_controller.pid.hp_o = ((u16)(*(data_buf+4)<<8)|*(data_buf+5));
			fly_controller.pid.hi_o = ((u16)(*(data_buf+6)<<8)|*(data_buf+7));
			fly_controller.pid.hd_o= ((u16)(*(data_buf+8)<<8)|*(data_buf+9));
			fly_controller.pid.hp_i = ((u16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.pid.hi_i =((u16)(*(data_buf+12)<<8)|*(data_buf+13));
			fly_controller.pid.hd_i=((u16)(*(data_buf+14)<<8)|*(data_buf+15));
		
	}
		else if(*(data_buf+2)==0x74)								//判断功能字0x82 PID参数
	{
	
			fly_controller.pid.pp_o = ((u16)(*(data_buf+4)<<8)|*(data_buf+5));
			fly_controller.pid.pi_o = ((u16)(*(data_buf+6)<<8)|*(data_buf+7));
			fly_controller.pid.pd_o= ((u16)(*(data_buf+8)<<8)|*(data_buf+9));
			fly_controller.pid.pp_i = ((u16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.pid.pi_i =((u16)(*(data_buf+12)<<8)|*(data_buf+13));
			fly_controller.pid.pd_i =((u16)(*(data_buf+14)<<8)|*(data_buf+15));
		
		  fly_controller.pid.rp_o = ((u16)(*(data_buf+4)<<8)|*(data_buf+5));
			fly_controller.pid.ri_o = ((u16)(*(data_buf+6)<<8)|*(data_buf+7));
			fly_controller.pid.rd_o= ((u16)(*(data_buf+8)<<8)|*(data_buf+9));
			fly_controller.pid.rp_i = ((u16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.pid.ri_i =((u16)(*(data_buf+12)<<8)|*(data_buf+13));
			fly_controller.pid.rd_i =((u16)(*(data_buf+14)<<8)|*(data_buf+15));
		
		  fly_controller.pid.yp_o = ((u16)(*(data_buf+4)<<8)|*(data_buf+5));
			fly_controller.pid.yi_o = ((u16)(*(data_buf+6)<<8)|*(data_buf+7));
			fly_controller.pid.yd_o= ((u16)(*(data_buf+8)<<8)|*(data_buf+9));
			fly_controller.pid.yp_i = ((u16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.pid.yi_i =((u16)(*(data_buf+12)<<8)|*(data_buf+13));
			fly_controller.pid.yd_i =((u16)(*(data_buf+14)<<8)|*(data_buf+15));
			
			fly_controller.pid.hp_o = ((u16)(*(data_buf+4)<<8)|*(data_buf+5));
			fly_controller.pid.hi_o = ((u16)(*(data_buf+6)<<8)|*(data_buf+7));
			fly_controller.pid.hd_o= ((u16)(*(data_buf+8)<<8)|*(data_buf+9));
			fly_controller.pid.hp_i = ((u16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.pid.hi_i =((u16)(*(data_buf+12)<<8)|*(data_buf+13));
			fly_controller.pid.hd_i=((u16)(*(data_buf+14)<<8)|*(data_buf+15));
		
	}
		else if(*(data_buf+2)==0x81)								//sd save1
	{  u8 cnt2=4,i=0;
		 u16 temp1,temp2;
		 for(i=0;i<20;i++){
		  temp1=(*(data_buf+(cnt2++))<<8);temp2=*(data_buf+(cnt2++));
			sd_save[i] = (vs16)(temp1|temp2);
		 }
	}
			else if(*(data_buf+2)==0x82)								//sd save2
	{  u8 cnt2=4,i=0;
		 u16 temp1,temp2;
		 for(i=20;i<20*2;i++){
		  temp1=(*(data_buf+(cnt2++))<<8);temp2=*(data_buf+(cnt2++));
			sd_save[i] =  (vs16)(temp1|temp2);
		 }
	}
			else if(*(data_buf+2)==0x83)								//sd save3
	{  u8 cnt2=4,i=0;
		 u16 temp1,temp2;
		 for(i=20*2;i<20*3;i++){
		  temp1=(*(data_buf+(cnt2++))<<8);temp2=*(data_buf+(cnt2++));
			sd_save[i] =  (vs16)(temp1|temp2);
		 }
		 sd.en_save=*(data_buf+(cnt2++));
	}
		else if(*(data_buf+2)==0x61)								//qr_traj
	{
		fly_controller.traj.drone1[0]= ((vs16)(*(data_buf+4)<<8)|*(data_buf+5));
		fly_controller.traj.drone1[1]= ((vs16)(*(data_buf+6)<<8)|*(data_buf+7));
		fly_controller.traj.drone1[2]= ((vs16)(*(data_buf+8)<<8)|*(data_buf+9));
		fly_controller.traj.drone2[0]= ((vs16)(*(data_buf+10)<<8)|*(data_buf+11));
		fly_controller.traj.drone2[1]= ((vs16)(*(data_buf+12)<<8)|*(data_buf+13));
		fly_controller.traj.drone2[2]= ((vs16)(*(data_buf+14)<<8)|*(data_buf+15));
		fly_controller.traj.drone3[0]= ((vs16)(*(data_buf+16)<<8)|*(data_buf+17));
		fly_controller.traj.drone3[1]= ((vs16)(*(data_buf+18)<<8)|*(data_buf+19));
		fly_controller.traj.drone3[2]= ((vs16)(*(data_buf+20)<<8)|*(data_buf+21));
	}		
}

u8 TxBuffer4[256];
u8 TxCounter4=0;
u8 count4=0; 

u8 Rx_Buf4[256];	//串口接收缓存
u8 RxBuffer4[50];
u8 RxState4 = 0;
u8 RxBufferNum4 = 0;
u8 RxBufferCnt4 = 0;
u8 RxLen4= 0;
static u8 _data_len4 = 0,_data_cnt4 = 0;
void USART1_IRQHandler(void)
{ OSIntEnter(); 
	u8 com_data;
	
	if(USART1->SR & USART_SR_ORE)//ORE中断
	{
		com_data = USART1->DR;
	}

  //接收中断
	if( USART_GetITStatus(USART1,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);//清除中断标志

		com_data = USART1->DR;
	
			if(RxState4==0&&com_data==0xAA)
		{
			RxState4=1;
			RxBuffer4[0]=com_data;
		}
		else if(RxState4==1&&com_data==0xAF)
		{
			RxState4=2;
			RxBuffer4[1]=com_data;
		}
		else if(RxState4==2&&com_data>0&&com_data<0XF1)
		{
			RxState4=3;
			RxBuffer4[2]=com_data;
		}
		else if(RxState4==3&&com_data<50)
		{
			RxState4 = 4;
			RxBuffer4[3]=com_data;
			_data_len4 = com_data;
			_data_cnt4 = 0;
		}
		else if(RxState4==4&&_data_len4>0)
		{
			_data_len4--;
			RxBuffer4[4+_data_cnt4++]=com_data;
			if(_data_len4==0)
				RxState4 = 5;
		}
		else if(RxState4==5)
		{
			RxState4 = 0;
			RxBuffer4[4+_data_cnt4]=com_data;
			DATA_M100(RxBuffer4,_data_cnt4+5);
			Data_Receive_Anl_FC(RxBuffer4,_data_cnt4+5);
		}
		else
			RxState4 = 0;
		
	}
	//发送（进入移位）中断
	if( USART_GetITStatus(USART1,USART_IT_TXE ) )
	{
				
		USART1->DR = TxBuffer[TxCounter++]; //写DR清除中断标志          
		if(TxCounter == count)
		{
			USART1->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}
	}
  
   OSIntExit(); 

}


void Send_RC_TO_FC(u8 sel)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
	switch (sel){case 0:
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x03;//???
	data_to_send[_cnt++]=0;//???
	
	RC_GETDATA Rc_Get_Temp;
	if(Rc_Get_SBUS.update&&Rc_Get_SBUS.connect)
	{
	Rc_Get_Temp.PITCH=Rc_Get_SBUS.PITCH;
  Rc_Get_Temp.ROLL=Rc_Get_SBUS.ROLL;
	Rc_Get_Temp.THROTTLE=Rc_Get_SBUS.THROTTLE;
	Rc_Get_Temp.YAW=Rc_Get_SBUS.YAW;
	Rc_Get_Temp.AUX1=Rc_Get_SBUS.AUX1;
	Rc_Get_Temp.AUX2=Rc_Get_SBUS.AUX2;;
	Rc_Get_Temp.AUX3=Rc_Get_SBUS.AUX3;
	Rc_Get_Temp.AUX4=Rc_Get_SBUS.AUX4;
	Rc_Get_Temp.AUX5=Rc_Get_SBUS.AUX5;}
	else if(Rc_Get_PPM.update){
	Rc_Get_Temp.PITCH=Rc_Get_PPM.PITCH;
  Rc_Get_Temp.ROLL=Rc_Get_PPM.ROLL;
	Rc_Get_Temp.THROTTLE=Rc_Get_PPM.THROTTLE;
	Rc_Get_Temp.YAW=Rc_Get_PPM.YAW;
	Rc_Get_Temp.AUX1=Rc_Get_PPM.AUX1;
	Rc_Get_Temp.AUX2=Rc_Get_PPM.AUX2;;
	Rc_Get_Temp.AUX3=Rc_Get_PPM.AUX3;
	Rc_Get_Temp.AUX4=Rc_Get_PPM.AUX4;
	Rc_Get_Temp.AUX5=Rc_Get_PPM.AUX5;
	}
	_temp=Rc_Get_Temp.PITCH;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
  _temp=Rc_Get_Temp.ROLL;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=Rc_Get_Temp.THROTTLE;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=Rc_Get_Temp.YAW;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=Rc_Get_Temp.AUX1;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=Rc_Get_Temp.AUX2;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=Rc_Get_Temp.AUX3;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=Rc_Get_Temp.AUX4;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=Rc_Get_Temp.AUX5;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	if(Rc_Get_PPM.update)
	_temp=1;	
	else if(Rc_Get_SBUS.update&&Rc_Get_SBUS.connect)
	_temp=2;
	else
	_temp=0;
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	break;
	case 1:
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x01;//???
	data_to_send[_cnt++]=0;//???	
	_temp=Rc_Get.PITCH;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
  _temp=Rc_Get.ROLL;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=Rc_Get.THROTTLE;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=Rc_Get.YAW;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=Rc_Get.AUX1;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=Rc_Get.AUX2;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=Rc_Get.AUX3;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=Rc_Get.AUX4;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=Rc_Get.AUX5;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=KEY_SEL[0];
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=KEY_SEL[1];
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=KEY_SEL[2];
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=KEY_SEL[3];
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=KEY[0];
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=KEY[1];
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=KEY[2];
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=KEY[3];
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=KEY[4];
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=KEY[5];
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=KEY[6];
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=KEY[7];
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=Rc_Get.update;
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	break;
  }
	Send_Data_GOL_LINK(data_to_send, _cnt);
}  


u8 SendBuff1[SEND_BUF_SIZE1];	//发送数据缓冲区
u16 nrf_uart_cnt;
void data_per_uart1_dma(u8 sel)
{
	u8 i;	u8 sum = 0;
	u16 _cnt=0,cnt_reg;
	vs16 _temp;
  

switch(sel){
	case SEND_NRF_RC:
	cnt_reg=nrf_uart_cnt;
	SendBuff1[nrf_uart_cnt++]=0xAA;
	SendBuff1[nrf_uart_cnt++]=0xAF;
	SendBuff1[nrf_uart_cnt++]=0x01;//功能字
	SendBuff1[nrf_uart_cnt++]=0;//数据量
	_temp=Rc_Get.PITCH;
	SendBuff1[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
  _temp=Rc_Get.ROLL;
	SendBuff1[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=Rc_Get.THROTTLE;
	SendBuff1[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=Rc_Get.YAW;
	SendBuff1[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=Rc_Get.AUX1;
	SendBuff1[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=Rc_Get.AUX2;
	SendBuff1[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=Rc_Get.AUX3;
	SendBuff1[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=Rc_Get.AUX4;
	SendBuff1[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=Rc_Get.AUX5;
	SendBuff1[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=KEY_SEL[0];
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=KEY_SEL[1];
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=KEY_SEL[2];
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=KEY_SEL[3];
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=KEY[0];
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=KEY[1];
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=KEY[2];
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=KEY[3];
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=KEY[4];
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=KEY[5];
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=KEY[6];
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=KEY[7];
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=Rc_Get.update;
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	
	SendBuff1[cnt_reg+3] =(nrf_uart_cnt-cnt_reg)-4;
		for( i=cnt_reg;i<nrf_uart_cnt;i++)
	sum += SendBuff1[i];
	SendBuff1[nrf_uart_cnt++] = sum;
	break;

	case SEND_NRF_PID:
	cnt_reg=nrf_uart_cnt;
	SendBuff1[nrf_uart_cnt++]=0xAA;
	SendBuff1[nrf_uart_cnt++]=0xAF;
	SendBuff1[nrf_uart_cnt++]=0x02;//功能字
	SendBuff1[nrf_uart_cnt++]=0;//数据量
	
	
	SendBuff1[cnt_reg+3] =(nrf_uart_cnt-cnt_reg)-4;
		for( i=cnt_reg;i<nrf_uart_cnt;i++)
	sum += SendBuff1[i];
	SendBuff1[nrf_uart_cnt++] = sum;
	break;
	case SEND_NRF_RC_PPMSBUS:
	cnt_reg=nrf_uart_cnt;
	SendBuff1[nrf_uart_cnt++]=0xAA;
	SendBuff1[nrf_uart_cnt++]=0xAF;
	SendBuff1[nrf_uart_cnt++]=0x03;//功能字
	SendBuff1[nrf_uart_cnt++]=0;//数据量
	RC_GETDATA Rc_Get_Temp;
	if(Rc_Get_SBUS.update&&Rc_Get_SBUS.connect)
	{
	Rc_Get_Temp.PITCH=Rc_Get_SBUS.PITCH;
  Rc_Get_Temp.ROLL=Rc_Get_SBUS.ROLL;
	Rc_Get_Temp.THROTTLE=Rc_Get_SBUS.THROTTLE;
	Rc_Get_Temp.YAW=Rc_Get_SBUS.YAW;
	Rc_Get_Temp.AUX1=Rc_Get_SBUS.AUX1;
	Rc_Get_Temp.AUX2=Rc_Get_SBUS.AUX2;;
	Rc_Get_Temp.AUX3=Rc_Get_SBUS.AUX3;
	Rc_Get_Temp.AUX4=Rc_Get_SBUS.AUX4;
	Rc_Get_Temp.AUX5=Rc_Get_SBUS.AUX5;}
	else if(Rc_Get_PPM.update){
	Rc_Get_Temp.PITCH=Rc_Get_PPM.PITCH;
  Rc_Get_Temp.ROLL=Rc_Get_PPM.ROLL;
	Rc_Get_Temp.THROTTLE=Rc_Get_PPM.THROTTLE;
	Rc_Get_Temp.YAW=Rc_Get_PPM.YAW;
	Rc_Get_Temp.AUX1=Rc_Get_PPM.AUX1;
	Rc_Get_Temp.AUX2=Rc_Get_PPM.AUX2;;
	Rc_Get_Temp.AUX3=Rc_Get_PPM.AUX3;
	Rc_Get_Temp.AUX4=Rc_Get_PPM.AUX4;
	Rc_Get_Temp.AUX5=Rc_Get_PPM.AUX5;
	}
	_temp=Rc_Get_Temp.PITCH;
	SendBuff1[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
  _temp=Rc_Get_Temp.ROLL;
	SendBuff1[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=Rc_Get_Temp.THROTTLE;
	SendBuff1[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=Rc_Get_Temp.YAW;
	SendBuff1[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=Rc_Get_Temp.AUX1;
	SendBuff1[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=Rc_Get_Temp.AUX2;
	SendBuff1[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=Rc_Get_Temp.AUX3;
	SendBuff1[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=Rc_Get_Temp.AUX4;
	SendBuff1[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=Rc_Get_Temp.AUX5;
	SendBuff1[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	if(Rc_Get_PPM.update)
	_temp=1;	
	else if(Rc_Get_SBUS.update&&Rc_Get_SBUS.connect)
	_temp=2;
	else
	_temp=0;
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	
	SendBuff1[cnt_reg+3] =(nrf_uart_cnt-cnt_reg)-4;
		for( i=cnt_reg;i<nrf_uart_cnt;i++)
	sum += SendBuff1[i];
	SendBuff1[nrf_uart_cnt++] = sum;	
	break;
	
	default:break;
}
}

void clear_nrf_uart(void)
{u16 i;
for(i=0;i<SEND_BUF_SIZE1;i++)
SendBuff1[i]=0;
 nrf_uart_cnt=0;
}

void Usart3_Init(u32 br_num)//-------SBUS
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
	
	//配置PD5作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
	//配置PD6作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = br_num;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_9b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_2;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_Even;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART3, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART3, ENABLE);  //使能串口1 
	
	USART_ClearFlag(USART3, USART_FLAG_TC);

	//使能USART2接收中断
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	//使能USART2
	USART_Cmd(USART3, ENABLE); 
}

u8 sbus_data_i_check[50];
void USART3_IRQHandler(void)
{ OSIntEnter(); 
	u8 com_data,i;
	static u8 state,state1;
	static uint8_t byteCNT = 0,byteCNT1;

	static uint32_t lastTime = 0;
	uint32_t curTime;
	uint32_t interval = 0;
	if(USART3->SR & USART_SR_ORE)//ORE中断
	{
		com_data = USART3->DR;
		Rc_Get_SBUS.lose_cnt=0;
		Rc_Get_SBUS.connect=1;
		oldx_sbus_rx(com_data);
		if(channels[16]==500||channels[16]==503){
		Rc_Get_SBUS.update=1;Rc_Get_SBUS.lose_cnt_rx=0; }
		if(Rc_Get_SBUS.lose_cnt_rx++>100){
		Rc_Get_SBUS.update=0;}
	}

  //接收中断
	if( USART_GetITStatus(USART3,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);//清除中断标志
		com_data = USART3->DR;
		Rc_Get_SBUS.lose_cnt=0;
		Rc_Get_SBUS.connect=1;
		oldx_sbus_rx(com_data);
		//for check input
		switch(state1)
		{
			case 0:
			if(com_data==0x0f)	
			{
			for(i=0;i<50;i++)
				sbus_data_i_check[i++]=0;
			byteCNT1=0;sbus_data_i_check[byteCNT1++]=com_data;state1=1;}
				break;
		  case 1:
				if(byteCNT1>49||com_data==0x0f)
				{
				state1=0;
				byteCNT1=0;
				}else{		
				sbus_data_i_check[byteCNT1++]=com_data;   
		  	}
			  break;
		}
		
    if(channels[16]==500||channels[16]==503){
		Rc_Get_SBUS.update=1;Rc_Get_SBUS.lose_cnt_rx=0; }
		if(Rc_Get_SBUS.lose_cnt_rx++>100){
		Rc_Get_SBUS.update=0;}
				
	}
	//发送（进入移位）中断
	if( USART_GetITStatus(USART3,USART_IT_TXE ) )
	{
				
		USART3->DR = TxBuffer[TxCounter++]; //写DR清除中断标志          
		if(TxCounter == count)
		{
			USART3->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}
	} 
   OSIntExit(); 
}


/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

