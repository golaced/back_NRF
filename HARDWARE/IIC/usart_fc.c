
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
 u8 SendBuff1[SEND_BUF_SIZE1];	//�������ݻ�����
 u8 SendBuff4[SEND_BUF_SIZE4];	//�������ݻ�����
void Usart2_Init(u32 br_num)//--GOL-link
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //����USART2ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);	
	
	//�����ж����ȼ�
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
	
	//����PD5��ΪUSART2��Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
	//����PD6��ΪUSART2��Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

	
   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = br_num;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART2, &USART_InitStructure); //��ʼ������1
	
  USART_Cmd(USART2, ENABLE);  //ʹ�ܴ���1 
	
	USART_ClearFlag(USART2, USART_FLAG_TC);
	

	//ʹ��USART2�����ж�
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	//ʹ��USART2
	USART_Cmd(USART2, ENABLE); 
//	//ʹ�ܷ��ͣ�������λ���ж�
//	if(!(USART2->CR1 & USART_CR1_TXEIE))
//	{
//		USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 
//	}


}
RESULT color;
float dt_flow_rx;
u16 data_rate_gol_link;
PID_STA HPID,SPID,FIX_PID,NAV_PID;
struct _PID_SET pid;
RC_GETDATA Rc_Get,Rc_Get_PPM,Rc_Get_SBUS;
struct _plane plane;
struct _slam slam;
struct _IMU_NAV imu_nav;
struct _MODE mode;
u8 LOCK, KEY[8],KEY_SEL[4];

u8 NAV_BOARD_CONNECT=0;
u8 SONAR_HEAD_CHECK[2];
u32 imu_loss_cnt;
u16 S_head;
float Pitch_DJ,Roll_DJ;
float ALT_POS_SONAR_HEAD,ALT_POS_SONAR_HEAD_LASER_SCANER;
 void Data_IMU(u8 *data_buf,u8 num)
{
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//�ж�sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//�ж�֡ͷ
  if(*(data_buf+2)==0x01)//FLOW_MINE_frame
  {
	  imu_loss_cnt=0;
    NAV_BOARD_CONNECT=1;
		imu_nav.flow.rate=*(data_buf+4);
		imu_nav.flow.speed.y_f=(float)((int16_t)((*(data_buf+5)<<8)|*(data_buf+6)))/1000.;
		imu_nav.flow.speed.x_f=(float)((int16_t)((*(data_buf+7)<<8)|*(data_buf+8)))/1000.;
		imu_nav.flow.speed.y = (float)(int16_t)((*(data_buf+9)<<8)|*(data_buf+10))/1000.;
		imu_nav.flow.speed.x = (float)(int16_t)((*(data_buf+11)<<8)|*(data_buf+12))/1000.;
		now_position[LON]=(float)(int16_t)((*(data_buf+13)<<8)|*(data_buf+14))/100.;//m
		now_position[LAT]=(float)(int16_t)((*(data_buf+15)<<8)|*(data_buf+16))/100.;//m
		//ALT_VEL_SONAR=(float)(int16_t)((*(data_buf+17)<<8)|*(data_buf+18))/1000.;//m
		float temp=(float)(int16_t)((*(data_buf+19)<<8)|*(data_buf+20))/1000.;//m
			if(temp<8.888)
	  ALT_POS_SONAR_HEAD = temp;
		SONAR_HEAD_CHECK[0]=*(data_buf+4);
    S_head=(float)(int16_t)((*(data_buf+33)<<8)|*(data_buf+34));
			
		//ALT_VEL_BMP=(float)(int16_t)((*(data_buf+21)<<8)|*(data_buf+22))/1000.;//m
		//ALT_POS_BMP=(float)(int32_t)((*(data_buf+23)<<24)|(*(data_buf+24)<<16)|(*(data_buf+25)<<8)|*(data_buf+26))/1000.;//m
		
	}		
 	/* else if(*(data_buf+2)==0x12)//debug
  {
	flow_debug.en_ble_debug= *(data_buf+4);	
	flow_debug.ax=((int16_t)(*(data_buf+5)<<8)|*(data_buf+6));
	flow_debug.ay=((int16_t)(*(data_buf+7)<<8)|*(data_buf+8));
	flow_debug.az=((int16_t)(*(data_buf+9)<<8)|*(data_buf+10));
	flow_debug.gx=((int16_t)(*(data_buf+11)<<8)|*(data_buf+12));
	flow_debug.gy=((int16_t)(*(data_buf+13)<<8)|*(data_buf+14));
	flow_debug.gz=((int16_t)(*(data_buf+15)<<8)|*(data_buf+16));
	flow_debug.hx=((int16_t)(*(data_buf+17)<<8)|*(data_buf+18));
	flow_debug.hy=((int16_t)(*(data_buf+19)<<8)|*(data_buf+20));
  flow_debug.hz=((int16_t)(*(data_buf+21)<<8)|*(data_buf+22));

	}		
	else if(*(data_buf+2)==0x02)//CAL
  {
	  mpu6050.Acc_Offset.x=(int16_t)(*(data_buf+4)<<8)|*(data_buf+5);
    mpu6050.Acc_Offset.y=(int16_t)(*(data_buf+6)<<8)|*(data_buf+7);
		mpu6050.Acc_Offset.z=(int16_t)(*(data_buf+8)<<8)|*(data_buf+9);
		mpu6050.Gyro_Offset.x=(int16_t)(*(data_buf+10)<<8)|*(data_buf+11);
		mpu6050.Gyro_Offset.y=(int16_t)(*(data_buf+12)<<8)|*(data_buf+13);
		mpu6050.Gyro_Offset.z=(int16_t)(*(data_buf+14)<<8)|*(data_buf+15);
		ak8975.Mag_Offset.x=(int16_t)(*(data_buf+16)<<8)|*(data_buf+17);
		ak8975.Mag_Offset.y=(int16_t)(*(data_buf+18)<<8)|*(data_buf+19);
		ak8975.Mag_Offset.z=(int16_t)(*(data_buf+20)<<8)|*(data_buf+21);
		ak8975.Mag_Gain.x=(float)((int16_t)(*(data_buf+22)<<8)|*(data_buf+23))/1000.;
		ak8975.Mag_Gain.y=(float)((int16_t)(*(data_buf+24)<<8)|*(data_buf+25))/1000.;
		ak8975.Mag_Gain.z=(float)((int16_t)(*(data_buf+26)<<8)|*(data_buf+27))/1000.;
	}		
else if(*(data_buf+2)==0x10)//Mems
  {
	  mpu6050.Acc.x=(int16_t)(*(data_buf+4)<<8)|*(data_buf+5);
    mpu6050.Acc.y=(int16_t)(*(data_buf+6)<<8)|*(data_buf+7);
		mpu6050.Acc.z=(int16_t)(*(data_buf+8)<<8)|*(data_buf+9);
		mpu6050.Gyro.x=(int16_t)(*(data_buf+10)<<8)|*(data_buf+11);
		mpu6050.Gyro.y=(int16_t)(*(data_buf+12)<<8)|*(data_buf+13);
		mpu6050.Gyro.z=(int16_t)(*(data_buf+14)<<8)|*(data_buf+15);
		mpu6050.Gyro_deg.x = mpu6050.Gyro.x *TO_ANGLE;
		mpu6050.Gyro_deg.y = mpu6050.Gyro.y *TO_ANGLE;
		mpu6050.Gyro_deg.z = mpu6050.Gyro.z *TO_ANGLE;
		ak8975.Mag_Val.x=(int16_t)(*(data_buf+16)<<8)|*(data_buf+17);
		ak8975.Mag_Val.y=(int16_t)(*(data_buf+18)<<8)|*(data_buf+19);
		ak8975.Mag_Val.z=(int16_t)(*(data_buf+20)<<8)|*(data_buf+21);
		int temp=(int16_t)(*(data_buf+22)<<8)|*(data_buf+23);
		if(temp<3200)
	  ultra_distance = temp;
	  baroAlt=(int32_t)((*(data_buf+24)<<24)|(*(data_buf+25)<<16)|(*(data_buf+26)<<8)|*(data_buf+27));
	}	*/	
	else if(*(data_buf+2)==0x11)//Att
  { //dt_flow_rx= Get_Cycle_T(GET_T_FLOW_UART);	
	  Pitch=(float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5))/10.;
    Roll=(float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7))/10.;
		Yaw=(float)((int16_t)(*(data_buf+8)<<8)|*(data_buf+9))/10.;
		q_nav[0]=(float)((int16_t)(*(data_buf+10)<<8)|*(data_buf+11))/1000.;
		q_nav[1]=(float)((int16_t)(*(data_buf+12)<<8)|*(data_buf+13))/1000.;
		q_nav[2]=(float)((int16_t)(*(data_buf+14)<<8)|*(data_buf+15))/1000.;
		q_nav[3]=(float)((int16_t)(*(data_buf+16)<<8)|*(data_buf+17))/1000.;
		Pitch_DJ=(float)((int16_t)(*(data_buf+18)<<8)|*(data_buf+19))/10.;
    Roll_DJ= (float)((int16_t)(*(data_buf+20)<<8)|*(data_buf+21))/10.;
		//Pitch_mid_down=(float)((int16_t)(*(data_buf+18)<<8)|*(data_buf+19))/10.;
    //Roll_mid_down=(float)((int16_t)(*(data_buf+20)<<8)|*(data_buf+21))/10.;
		//Yaw_mid_down=(float)((int16_t)(*(data_buf+22)<<8)|*(data_buf+23))/10.;
		//ref_q_imd_down[0]=(float)((int16_t)(*(data_buf+24)<<8)|*(data_buf+25))/1000.;
		//ref_q_imd_down[1]=(float)((int16_t)(*(data_buf+26)<<8)|*(data_buf+27))/1000.;
		//ref_q_imd_down[2]=(float)((int16_t)(*(data_buf+28)<<8)|*(data_buf+29))/1000.;
		//ref_q_imd_down[3]=(float)((int16_t)(*(data_buf+30)<<8)|*(data_buf+31))/1000.;
		ALT_VEL_BMP_EKF=(float)(int16_t)((*(data_buf+32)<<8)|*(data_buf+33))/1000.;//m
		
		
  	//reference_vr[0]=reference_v.x = 2*(q_nav[1]*q_nav[3] - q_nav[0]*q_nav[2]);
  	//reference_vr[1]=reference_v.y = 2*(q_nav[0]*q_nav[1] + q_nav[2]*q_nav[3]);
	  //reference_vr[2]=reference_v.z = 1 - 2*(q_nav[1]*q_nav[1] + q_nav[2]*q_nav[2]);
	  //reference_vr_imd_down[0] = 2*(ref_q_imd_down[1]*ref_q_imd_down[3] - ref_q_imd_down[0]*ref_q_imd_down[2]);
	  //reference_vr_imd_down[1] = 2*(ref_q_imd_down[0]*ref_q_imd_down[1] + ref_q_imd_down[2]*ref_q_imd_down[3]);
	  //reference_vr_imd_down[2] = 1 - 2*(ref_q_imd_down[1]*ref_q_imd_down[1] + ref_q_imd_down[2]*ref_q_imd_down[2]);
	
	}		
}
u8 laser_sel=1;

u8 TxBuffer[256];
u8 TxCounter=0;
u8 count=0; 

u8 Rx_Buf[256];	//���ڽ��ջ���
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
	
	if(USART2->SR & USART_SR_ORE)//ORE�ж�
	{
		com_data = USART2->DR;
	}

  //�����ж�
	if( USART_GetITStatus(USART2,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);//����жϱ�־

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
	//���ͣ�������λ���ж�
	if( USART_GetITStatus(USART2,USART_IT_TXE ) )
	{
				
		USART2->DR = TxBuffer[TxCounter++]; //дDR����жϱ�־          
		if(TxCounter == count)
		{
			USART2->CR1 &= ~USART_CR1_TXEIE;		//�ر�TXE�������жϣ��ж�
		}


		//USART_ClearITPendingBit(USART2,USART_IT_TXE);
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
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //����USART2ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);	
	
	//�����ж����ȼ�
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	
	//����PD5��ΪUSART2��Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
	//����PD6��ΪUSART2��Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = br_num;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART1, &USART_InitStructure); //��ʼ������1
	
  USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���1 
	
	USART_ClearFlag(USART1, USART_FLAG_TC);
	

	//ʹ��USART2�����ж�
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	//ʹ��USART2
	USART_Cmd(USART1, ENABLE); 
//	//ʹ�ܷ��ͣ�������λ���ж�
//	if(!(USART2->CR1 & USART_CR1_TXEIE))
//	{
//		USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 
//	}


}


#include "gps.h"

nmea_msg gpsx,gps_data; 		
u8  buf_GPRMC[100]={'G','P','R','M','C',','};//GPS��Ϣ
u8  buf_GPRMCt[100]={"GPRMC,144326.00,A,5107.0017737,N,11402.3291611,W,0.080,323.3,210307,0.0,E,A*20"};//GPS��Ϣ
u8  buf_GPGGA[100]={'G','P','G','G','A',','};//GPS��Ϣ
u8  buf_GPGGAt[100]={"GPGGA,134658.00,5106.9792,N,11402.3003,W,2,09,1.0,1048.47,M,-16.27,M,08,AAAA*60"};//GPS��Ϣ
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
	if(!(sum==*(data_buf+num-1)))		return;		//�ж�sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//�ж�֡ͷ
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
	if(!(sum==*(data_buf+num-1)))		return;		//�ж�sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		
	{
		j++;return;	
	}		//�ж�֡ͷ
	if(*(data_buf+2)==0x05)								//�жϹ�����0x80 �ɿ�IMU����
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
		 //  en_save = *(data_buf+30);//sd_ʹ�ܴ洢		

	}
	else if(*(data_buf+2)==0x01)								//�жϹ�����0x82 �ɿ���̬PID������
	{
	
			fly_controller.set.pitch = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/10;
			fly_controller.set.roll = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/10;
			fly_controller.set.yaw=(float) ((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/10;
	
	}
	else if(*(data_buf+2)==0x02)								//�ɿع���PID����
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
		else if(*(data_buf+2)==0x03)								//�ɿظ߶�PID����
	{
	
			data_check_int(&fly_controller.set.alt ,&fly_controller_r.set.alt , ((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/10,-10*100,10*100);
		  data_check_int(&fly_controller.now.alt ,&fly_controller_r.now.alt , ((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/10,-10*100,10*100);
		  data_check_float(&fly_controller.set.spd_alt,&fly_controller_r.set.spd_alt , ((vs16)(*(data_buf+8)<<8)|*(data_buf+9)),-500,500);
	    fly_controller.now.spd_alt = ((vs16)(*(data_buf+10)<<8)|*(data_buf+11));
			fly_controller.now.thr = ((vs16)(*(data_buf+12)<<8)|*(data_buf+13));
		  data_check_int(&fly_controller.now.alt_bmp ,&fly_controller_r.now.alt_bmp , ((vs16)(*(data_buf+14)<<8)|*(data_buf+15)),-10100,10100);
		  data_check_int(&fly_controller.now.alt_fushion ,&fly_controller_r.now.alt_fushion , ((vs16)(*(data_buf+16)<<8)|*(data_buf+17)),-10100,10100);
		
	}
			else if(*(data_buf+2)==0x04)								//�ɿ�ģʽ
	{
	
			//en_save = *(data_buf+4);//sd_ʹ�ܴ洢		
	}
			else if(*(data_buf+2)==0x06)								//�ɿ�ʹ�ù�������
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
				else if(*(data_buf+2)==0x07)								//�ɿ�ʹ�ù�������
	{
			SPID.OP= ((vs16)(*(data_buf+4)<<8)|*(data_buf+5));
			SPID.OI= ((vs16)(*(data_buf+6)<<8)|*(data_buf+7));
			SPID.OD= ((vs16)(*(data_buf+8)<<8)|*(data_buf+9));
			SPID.IP= ((vs16)(*(data_buf+10)<<8)|*(data_buf+11));
			SPID.II= ((vs16)(*(data_buf+12)<<8)|*(data_buf+13));
			SPID.ID= ((vs16)(*(data_buf+14)<<8)|*(data_buf+15));
			SPID.YP= ((vs16)(*(data_buf+16)<<8)|*(data_buf+17));
			SPID.YI= ((vs16)(*(data_buf+18)<<8)|*(data_buf+19));
			SPID.YD= ((vs16)(*(data_buf+20)<<8)|*(data_buf+21));
			HPID.OP= ((vs16)(*(data_buf+22)<<8)|*(data_buf+23));
			HPID.OI= ((vs16)(*(data_buf+24)<<8)|*(data_buf+25));
			HPID.OD= ((vs16)(*(data_buf+26)<<8)|*(data_buf+27));
		
	}
				else if(*(data_buf+2)==0x08)								//�ɿ�ʹ��GPS����
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
		
	/*
	fly_controller.gps.J = 			  ((long)(*(data_buf+4)<<24)|(*(data_buf+5)<<16)|(*(data_buf+6)<<8)|*(data_buf+7));//W
	fly_controller.gps.W = 			  ((long)(*(data_buf+8)<<24)|(*(data_buf+9)<<16)|(*(data_buf+10)<<8)|*(data_buf+11));//J
	fly_controller.gps.gps_mode =  *(data_buf+12);//W
	fly_controller.gps.star_num =  *(data_buf+13);//J
	fly_controller.gps.X_O = 			((long)(*(data_buf+14)<<24)|(*(data_buf+15)<<16)|(*(data_buf+16)<<8)|*(data_buf+17));//W
	fly_controller.gps.Y_O = 			((long)(*(data_buf+18)<<24)|(*(data_buf+19)<<16)|(*(data_buf+20)<<8)|*(data_buf+21));//J
	fly_controller.gps.X_UKF = 		((long)(*(data_buf+22)<<24)|(*(data_buf+23)<<16)|(*(data_buf+24)<<8)|*(data_buf+25));//W
	fly_controller.gps.Y_UKF = 		((long)(*(data_buf+26)<<24)|(*(data_buf+27)<<16)|(*(data_buf+28)<<8)|*(data_buf+29));//J
		*/
	}
				else if(*(data_buf+2)==0x09)								//�ɿ�ʹ�ù�������
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
//		   mark[0][0]= ((vs16)(*(data_buf+28)<<8)|*(data_buf+29));
//			 baro_matlab_data[1]= ((vs16)(*(data_buf+30)<<8)|*(data_buf+31));
//		   qr_matlab_data[0]= *(data_buf+32);
//			 qr_matlab_data[1]= ((vs16)(*(data_buf+33)<<8)|*(data_buf+34));
//			 qr_matlab_data[2]= ((vs16)(*(data_buf+35)<<8)|*(data_buf+36));
//			 qr_matlab_data[3]= ((vs16)(*(data_buf+35)<<8)|*(data_buf+36));
//			 qr_matlab_data_att[0]= ((vs16)(*(data_buf+37)<<8)|*(data_buf+38));
//			 qr_matlab_data_att[1]= ((vs16)(*(data_buf+39)<<8)|*(data_buf+40));
//			 qr_matlab_data_att[2]= ((vs16)(*(data_buf+41)<<8)|*(data_buf+42));
//       k_scale_pix= ((vs16)(*(data_buf+43)<<8)|*(data_buf+44));
		
	}
	//---------------------------------------------------------------------
	else if(*(data_buf+2)==0x71)								//�жϹ�����0x81 �ɿ�Sensor����
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
		else if(*(data_buf+2)==0x74)								//�жϹ�����0x82 PID����
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
		else if(*(data_buf+2)==0x74)								//�жϹ�����0x82 PID����
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

u8 Rx_Buf4[256];	//���ڽ��ջ���
u8 RxBuffer4[50];
u8 RxState4 = 0;
u8 RxBufferNum4 = 0;
u8 RxBufferCnt4 = 0;
u8 RxLen4= 0;
static u8 _data_len4 = 0,_data_cnt4 = 0;
void USART1_IRQHandler(void)
{ OSIntEnter(); 
	u8 com_data;
	
	if(USART1->SR & USART_SR_ORE)//ORE�ж�
	{
		com_data = USART1->DR;
	}

  //�����ж�
	if( USART_GetITStatus(USART1,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);//����жϱ�־

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
	//���ͣ�������λ���ж�
	if( USART_GetITStatus(USART1,USART_IT_TXE ) )
	{
				
		USART1->DR = TxBuffer[TxCounter++]; //дDR����жϱ�־          
		if(TxCounter == count)
		{
			USART1->CR1 &= ~USART_CR1_TXEIE;		//�ر�TXE�������жϣ��ж�
		}


		//USART_ClearITPendingBit(USART2,USART_IT_TXE);
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


u8 SendBuff1[SEND_BUF_SIZE1];	//�������ݻ�����
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
	SendBuff1[nrf_uart_cnt++]=0x01;//������
	SendBuff1[nrf_uart_cnt++]=0;//������
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
	SendBuff1[nrf_uart_cnt++]=0x02;//������
	SendBuff1[nrf_uart_cnt++]=0;//������
	_temp=SPID.OP;
	SendBuff1[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
  _temp=SPID.OI;
	SendBuff1[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=SPID.OD;
	SendBuff1[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	
	_temp=SPID.IP;
	SendBuff1[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=SPID.II;
	SendBuff1[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=SPID.ID;
	SendBuff1[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	
	_temp=SPID.YP;
	SendBuff1[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=SPID.YI;
	SendBuff1[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=SPID.YD;
	SendBuff1[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	
	_temp=HPID.OP;
	SendBuff1[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=HPID.OI;
	SendBuff1[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=HPID.OD;
	SendBuff1[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	
	_temp=HPID.IP;
	SendBuff1[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=HPID.II;
	SendBuff1[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	_temp=HPID.ID;
	SendBuff1[nrf_uart_cnt++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt++]=BYTE0(_temp);
	
	SendBuff1[cnt_reg+3] =(nrf_uart_cnt-cnt_reg)-4;
		for( i=cnt_reg;i<nrf_uart_cnt;i++)
	sum += SendBuff1[i];
	SendBuff1[nrf_uart_cnt++] = sum;
	break;
	case SEND_NRF_RC_PPMSBUS:
	cnt_reg=nrf_uart_cnt;
	SendBuff1[nrf_uart_cnt++]=0xAA;
	SendBuff1[nrf_uart_cnt++]=0xAF;
	SendBuff1[nrf_uart_cnt++]=0x03;//������
	SendBuff1[nrf_uart_cnt++]=0;//������
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

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //����USART2ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);	
	
	//�����ж����ȼ�
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
	
	//����PD5��ΪUSART2��Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
	//����PD6��ΪUSART2��Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
	
	
   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = br_num;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_9b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_2;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_Even;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART3, &USART_InitStructure); //��ʼ������1
	
  USART_Cmd(USART3, ENABLE);  //ʹ�ܴ���1 
	
	USART_ClearFlag(USART3, USART_FLAG_TC);

	//ʹ��USART2�����ж�
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	//ʹ��USART2
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
	if(USART3->SR & USART_SR_ORE)//ORE�ж�
	{
		com_data = USART3->DR;
	}

  //�����ж�
	if( USART_GetITStatus(USART3,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);//����жϱ�־
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
	//���ͣ�������λ���ж�
	if( USART_GetITStatus(USART3,USART_IT_TXE ) )
	{
				
		USART3->DR = TxBuffer[TxCounter++]; //дDR����жϱ�־          
		if(TxCounter == count)
		{
			USART3->CR1 &= ~USART_CR1_TXEIE;		//�ر�TXE�������жϣ��ж�
		}
	}
  
   OSIntExit(); 

}


/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

