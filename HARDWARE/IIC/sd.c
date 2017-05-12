#include "include.h"
#include "sdio.h"
#include "sd.h"
struct _SD sd;
u8 res=0,sd_had_init,sdcard_force_out;	
FIL fsrc, fdst;      /* file objects */
char path0[512]="0:";
char buffer[1000];   /* file copy buffer */
uint8_t textFileBuffer[] = "Test\r\n";
uint8_t textFileBuffer2[] ="Done\r\n";

void sd_write_9channal(float ch1,float ch2,float ch3,float ch4,float ch5,float ch6,float ch7,float ch8,float ch9);
void sd_write_sensor_test(float new_data,u8 ndigit);

u16 creat_floder(void);
char *my_ftoa(double number,int ndigit,char *buf);


void SD_INIT(void)
{
	res = 	f_mount(fs[0],"0:",1); 					//挂载SD卡 
	if(res != FR_OK){
		;//printf("mount filesystem 0 failed : %d\r\n",res);
	}
}


void Uart_zero_fix(char* buf ,u32 num)
{u32 i;
	for(i=0;i<num-1;i++)
			{
			 if(buf[i]==0x0)
				   buf[i]=' ';
		 }

}	


volatile uint32_t lastUpdater, nowr; // 采样周期计数 单位 us
#include "led_fc.h"
char data_conv_test[20];
u8 nrf_right;
void sd_task_init(void)//---------------------------------------------------------------------------------------
{   static u8 sd_insert_reg=0,state;
	  static u8 init;
	 
 	  //sd_check();
	if(sd_insert&&!init)
	{
	init=1;
//       if(SD_Init())
//				{state=1;
//				//exfuns_init();							//为fatfs相关变量申请内存				 
//				//SD_INIT();
//				sd_had_init=1;
//				}
			}
}

void sd_task_init1(void)//---------------------------------------------------------------------------------------
{   static u8 sd_insert_reg=0,state;
	  static u8 init;
	 
 	  sd_check();
	if(sd_insert&&!init)
	{
	init=1;
       if(SD_Init())
				{state=1;
				exfuns_init();							//为fatfs相关变量申请内存				 
				SD_INIT();
				sd_had_init=1;
				}
	}
	switch (state)
	{
		case 0:
			if(sd_insert==1&&sd_insert_reg==0&&sd_had_init==0)
			{ power_sd(1);
			
				if(!SD_Init())
				{state=1;
				exfuns_init();							//为fatfs相关变量申请内存				 
				SD_INIT();
				sd_had_init=1;
				}
			}
				
		break;
		case 1:
			if(sd_insert==0){
		  sd_had_init=0;
			state=0;
			power_sd(0);	
			}
	
	  break;
	}		 
	sd_insert_reg=sd_insert;
}
char* my_itoa(int value,char *str,int radix)
{
	int sign = 0;
	//char *s = str;
	char ps[32];int i=0,j=0;
	memset(ps,0,32);
	
	if(value < 0)
	{
		sign = -1;
		value = -value;
	}
	do
	{
		if(value%radix>9)
			ps[i] = value%radix +'0'+7;
		else 
			ps[i] = value%radix +'0';
		i++;
	}while((value/=radix)>0);
	if(sign<0)
		ps[i] = '-';
	else
		i--;
	for(j=i;j>=0;j--)
	{
		str[i-j] = ps[j];
	}
	return str;
}

char *my_ftoa(double number,int ndigit,char *buf)
{
  long int_part;int i=0;
	double float_part;
	char str_int[32];
	char str_float[32];
	memset(str_int,0,32);
	memset(str_float,0,32);
	int_part = (long)number;
	float_part = number - int_part;
	my_itoa(int_part,str_int,10);
	if(ndigit>0)
	{
		float_part = pow(10.0,(double)ndigit)*fabs(float_part);
		my_itoa((long)float_part,str_float,10);
	}
	 i= strlen(str_int);
	str_int[i] = '.';
	strcat(str_int,str_float);
	strcpy(buf,str_int);
	return buf;
}

#define MAX_FILE_SCAN 25

u8 BUF_TIME[14];
u16 creat_floder(void)
{	char file_name[20]={"0:/DATA_OUT"};
	u8 res,i,j,k;					 
	u16 index=0;
   res=f_mkdir(file_name);//!=FR_EXIST);		//创建PHOTO文件夹
for (k=0;k<20;k++)
	file_name[k]=0;
 for(index=0;index<MAX_FILE_SCAN;index++)
{ 
										 
	sprintf(file_name,"0:/DATA_OUT/ODS_%d",index);
  res=f_mkdir(file_name);//!=FR_EXIST);		//创建PHOTO文件夹
 	for (k=0;k<20;k++)
	file_name[k]=0;
	
	if(res==FR_OK)
		break;
	
	}

return 	index;
} 
u8 init_sd_write=0,sdcard_force_out=0,sd_had_init=0;

void sd_write(float *ch,char *file_name,u16 index, u32 t_now,u8 max_num,u8 point)
{	
char folder[40]={0};
u16 i=0,j=0,size=0,k=0,l=0;
char data_conv_time[20]={0};
char data_conv_data[20]={0};
char data_write[200]={0};	
float halfT;
static u8 init;

	my_ftoa(t_now,1,data_conv_time);
	
	while(data_conv_time[i]!= '.')
	{data_write[i]=data_conv_time[i];i++;}
	data_write[i++]='m';
	data_write[i++]='s';
	data_write[i++]=' ';
	data_write[i++]=' ';
	
for(l=0;l<max_num;l++){//--ch write task
	my_ftoa(*ch++,point,data_conv_data);
	while(data_conv_data[j]!= '\0')
	data_write[i++]=data_conv_data[j++];	j=0;
	data_write[i++]=' ';
	data_write[i++]=' ';
}
  data_write[i++]='\r';
	data_write[i++]='\n';

	while(data_write[size]!= '\0')
	size++;
	//--写入文件夹
	//sprintf(folder,"0:/DATA_OUT/X%d_%s",index,BUF_TIME);			
		sprintf(folder,"0:/DATA_OUT/ODS_%d",index);
	for(i=0;i<40;i++)
	 if(folder[i]==0)
		   break;
	 
	 folder[i++]='/';
	 for(j=0;j<20;j++)//最大检索文件
	 { folder[i+j]=file_name[j];
	 }
	 //--
	  res = f_open(&fdst, folder, FA_OPEN_ALWAYS | FA_WRITE);//以只写的方式打开，如果文件不存在的话则创建这个文件
	if(res != FR_OK){
		//printf("open file error : %d\r\n",res);
		//en_save=0;
	}else{
		  
		   f_lseek(&fdst,fdst.fsize);
	    res = f_write(&fdst, data_write,size, &bw);               /* Write it to the dst file */
		if(res == FR_OK){
		//	printf("write data ok! write num=%d\r\n",bw);
		}else{
			//printf("write data error : %d\r\n",res);
		}
	
		/*close file */
		f_close(&fdst);
	}
}

long t_sd_save;
void SD_Save_Task(float delay){
u8 i;
static u8 state=0;
static u16 index=0;
float ch[25]={0};
float halfT;
static float test_data[9];
u16 delta_t;
static u32 t_now;
 nowr = micros();  //读取时间
  if(nowr<lastUpdater){ //定时器溢出过了。
  halfT =  ((float)(nowr + (0xffff- lastUpdater)) / 1000000.0f);
  }
  else	{
  halfT =  ((float)(nowr - lastUpdater) / 1000000.0f);
  }
  lastUpdater = nowr;	//更新时间
	delta_t=delay*1000;
	
switch (state)
{
	case 0:
	if(en_save)
  {		
		index=creat_floder();
	  t_now=0;
	  state=1;
	}
	break;
	case 1:
	if(!en_save)
  {	init_sd_write=0;t_now=0;state=0;}
	else
	{u8 cnt_;
	 cnt_=0;	
	

//ALT UKF		 
//	 ch[cnt_++]=fly_controller.imu.roll;
//	 ch[cnt_++]=fly_controller.imu.pitch;
//	 ch[cnt_++]=fly_controller.imu.yaw;
//	 ch[cnt_++]=qr_matlab_data_att[0];
//	 ch[cnt_++]=qr_matlab_data_att[1];
//	 ch[cnt_++]=qr_matlab_data_att[2];
//	 ch[cnt_++]=fly_controller.imu.acc_x;
//	 ch[cnt_++]=fly_controller.imu.acc_y;
//	 ch[cnt_++]=fly_controller.imu.acc_z;
//		 
//	ch[cnt_++]=fly_controller.now.alt_sonar ;//z sonar
//	ch[cnt_++]=fly_controller.now.alt_fushion_sonar ;//z sonar acc
//	ch[cnt_++]=qr_matlab_data[3];//z qr
//	ch[cnt_++]=k_scale_pix;
//	 sd_write(ch,"imu_fly_board.txt", index, t_now,cnt_,1);cnt_=0;
//  HML CAL	
//	 ch[cnt_++]=fly_controller.imu.roll;
//	 ch[cnt_++]=fly_controller.imu.pitch;
//	 ch[cnt_++]=fly_controller.imu.yaw;
//	 ch[cnt_++]=fly_controller.sensor.hmx;
//	 ch[cnt_++]=fly_controller.sensor.hmy;
//	 ch[cnt_++]=fly_controller.sensor.hmz;
//	 ch[cnt_++]=fly_controller.sensor.hmx_c;
//	 ch[cnt_++]=fly_controller.sensor.hmy_c;
//	 ch[cnt_++]=fly_controller.sensor.hmz_c;
//	 sd_write(ch,"hml.txt", index, t_now,cnt_,1);cnt_=0;
		
//  FLOW QR		
//		ch[cnt_++]=fly_controller.flow.spd_f[0];
//		ch[cnt_++]=fly_controller.flow.spd_f[1];
//		ch[cnt_++]=fly_controller.flow.pos_t[0];
//		ch[cnt_++]=fly_controller.flow.pos_t[1];
//		ch[cnt_++]=fly_controller.flow.pos[0];
//		ch[cnt_++]=fly_controller.flow.pos[1];
//		ch[cnt_++]=fly_controller.flow.spd[0];
//		ch[cnt_++]=fly_controller.flow.spd[1];
//	  ch[cnt_++]=flow_matlab_data[0];
//		ch[cnt_++]=flow_matlab_data[1];
//		ch[cnt_++]=flow_matlab_data[2];
//		ch[cnt_++]=flow_matlab_data[3];	
//	 	ch[cnt_++]=baro_matlab_data[0];
//		ch[cnt_++]=baro_matlab_data[1];
//		ch[cnt_++]=baro_matlab_data[2];
//		ch[cnt_++]=baro_matlab_data[3];
//		ch[cnt_++]=qr_matlab_data[0];
//		ch[cnt_++]=qr_matlab_data[1];
//		ch[cnt_++]=qr_matlab_data[2];

//		
//	 sd_write(ch,"flow_sensor.txt", index, t_now,cnt_,3);cnt_=0;
//GPS
	 
//	  ch[cnt_++]=flow_matlab_data[0];
//		ch[cnt_++]=flow_matlab_data[1];
//		ch[cnt_++]=flow_matlab_data[2];
//		ch[cnt_++]=flow_matlab_data[3];
//		ch[cnt_++]=(float)fly_controller.gps.J/10000000.;	
//		ch[cnt_++]=(float)fly_controller.gps.W/10000000.;			
//		ch[cnt_++]=fly_controller.gps.gps_mode;
//		ch[cnt_++]=fly_controller.gps.spd;
//		ch[cnt_++]=fly_controller.gps.angle;
//		ch[cnt_++]=fly_controller.gps.yaw;
//		ch[cnt_++]=fly_controller.gps.star_num;
//		ch[cnt_++]=fly_controller.gps.svnum;
//		sd_write(ch,"gps.txt", index, t_now,cnt_,7);cnt_=0;
//		
//		
//		ch[cnt_++]=qr_matlab_data[0];
//		ch[cnt_++]=qr_matlab_data[1];
//		ch[cnt_++]=qr_matlab_data[2];
//		ch[cnt_++]=fly_controller.now.alt_fushion_sonar;
//		sd_write(ch,"bai.txt", index, t_now,cnt_,7);cnt_=0;
// HML YAW_CAL
//	ch[cnt_++]=fly_controller.imu.roll;
//	ch[cnt_++]=fly_controller.imu.pitch;
//	ch[cnt_++]=fly_controller.imu.yaw;
//	
//	ch[cnt_++]=fly_controller.sensor.hmx_c;
//	ch[cnt_++]=fly_controller.sensor.hmy_c;
//	ch[cnt_++]=fly_controller.sensor.hmz_c;
//	ch[cnt_++]=fly_controller.imu.acc_x;
//	ch[cnt_++]=fly_controller.imu.acc_y;
//	ch[cnt_++]=fly_controller.imu.acc_z;
//	sd_write(ch,"hml_yaw.txt", index, t_now,cnt_,2);cnt_=0;
		
//QR
//	  ch[cnt_++]=mark[0][0];
//		ch[cnt_++]=mark[0][1];
//		ch[cnt_++]=mark[0][2];
//		ch[cnt_++]=mark[0][3];
//		ch[cnt_++]=mark[1][0];
//		ch[cnt_++]=mark[1][1];
//		ch[cnt_++]=mark[1][2];
//		ch[cnt_++]=mark[1][3];
//		ch[cnt_++]=mark[2][0];
//		ch[cnt_++]=mark[2][1];
//		ch[cnt_++]=mark[2][2];
//		ch[cnt_++]=mark[2][3];
//		ch[cnt_++]=mark[3][0];
//		ch[cnt_++]=mark[3][1];
//		ch[cnt_++]=mark[3][2];
//		ch[cnt_++]=mark[3][3];
//		sd_write(ch,"mark.txt", index, t_now,cnt_,2);cnt_=0;
//		ch[cnt_++]=qr_matlab_data[0];
//		ch[cnt_++]=qr_matlab_data[1];
//		ch[cnt_++]=qr_matlab_data[2];
//		ch[cnt_++]=fly_controller.now.alt_fushion_sonar;
//		sd_write(ch,"bai.txt", index, t_now,cnt_,7);cnt_=0;
//		
//	 sd_write(ch,"baro_sensor.txt", index, t_now,4,3);
	 
	 //ch[0]=1;ch[1]=2;ch[2]=3;ch[3]=4;ch[4]=5;ch[5]=6;ch[6]=7;ch[7]=8;ch[8]=9;
	 //sd_write(ch,"imu_nav_board.txt", index, t_now);
		
	 //ch[0]=1;ch[1]=2;ch[2]=3;ch[3]=4;ch[4]=5;ch[5]=6;ch[6]=7;ch[7]=8;ch[8]=9;	
	 //sd_write(ch,"sensor_fly_board.txt", index, t_now);	
		
	 //ch[0]=1;ch[1]=2;ch[2]=3;ch[3]=4;ch[4]=5;ch[5]=6;ch[6]=7;ch[7]=8;ch[8]=9;	
	 //sd_write(ch,"sensor_nav_board.txt", index, t_now);		
		

//		ch[0]=fly_controller.set.roll ;
//		ch[1]=fly_controller.set.pitch ;
//		ch[2]=fly_controller.set.yaw ;
//		ch[3]=fly_controller.imu.roll;
//		ch[4]=fly_controller.imu.pitch;
//		ch[5]=fly_controller.imu.yaw;
//		sd_write(ch,"pid_att.txt", index, t_now,6,1);	

//		ch[0]=fly_controller.set.alt;ch[1]=fly_controller.now.alt;
//		ch[2]=fly_controller.set.spd_alt;ch[3]=fly_controller.now.spd_alt;
//		ch[4]=fly_controller.now.thr;ch[5]=fly_controller.now.alt_bmp;
//		ch[6]=fly_controller.now.alt_fushion;
//		sd_write(ch,"pid_alt.txt", index, t_now,7,1);			

//		ch[0]=fly_controller.set.pos[0];ch[1]=fly_controller.set.pos[1];
//		ch[2]=fly_controller.now.pos[0];ch[3]=fly_controller.now.pos[1];
//		ch[4]=fly_controller.set.spd[0];ch[5]=fly_controller.set.spd[1];
//		ch[6]=fly_controller.now.spd[0];ch[7]=fly_controller.now.spd[1];
//		ch[8]=fly_controller.now.nav[0];ch[9]=fly_controller.now.nav[1];
//		sd_write(ch,"pid_flow.txt", index, t_now,10,1);	

//		ch[0]=fly_controller.flow.spd[0];ch[1]=fly_controller.flow.spd[1];
//		ch[2]= fly_controller.flow.spd_x;ch[3]= fly_controller.flow.spd_y;
//		ch[4]=fly_controller.flow.pos[0];ch[5]=fly_controller.flow.pos[1];
//		sd_write(ch,"fly_flow_data.txt", index, t_now,6,1);


//		ch[0]=fly_controller.slam_sonar[0];ch[1]=fly_controller.slam_sonar[1];
//		ch[2]=fly_controller.slam_sonar[2];ch[3]=fly_controller.slam_sonar[3];
//		ch[4]=fly_controller.slam_sonar[4];
//		sd_write(ch,"slam_sonar.txt", index, t_now,5,1);
		
//		ch[0]=fly_controller.gps.W/100000;ch[1]=fly_controller.gps.J/100000;
//		ch[2]=fly_controller.gps.Y_O/100;ch[3]=fly_controller.gps.X_O/100;
//		ch[4]=fly_controller.gps.Y_UKF/100;ch[5]=fly_controller.gps.X_UKF/100;
//		sd_write(ch,"gps_data.txt", index, t_now,6,6);	


//		ch[0]=fly_controller.imu.gro_x;ch[1]=fly_controller.imu.gro_y;ch[2]=fly_controller.imu.gro_z;
//		ch[3]=fly_controller.imu.acc_x;ch[4]=fly_controller.imu.acc_y;ch[5]=fly_controller.imu.acc_z;
//		ch[6]=fly_controller.imu.q0;ch[7]=fly_controller.imu.q1;ch[8]=fly_controller.imu.q2;ch[9]=fly_controller.imu.q3;
//		sd_write(ch,"imu_sensor_data.txt", index, t_now,10,1);	


//sd1
		for(i=0;i<20;i++)
	  ch[cnt_++]=sd_save[i];
		sd_write(ch,"sd1.txt", index, t_now,cnt_,1);cnt_=0;
//sd2
		for(i=20;i<20*2;i++)
	  ch[cnt_++]=sd_save[i];
		sd_write(ch,"sd2.txt", index, t_now,cnt_,1);cnt_=0;
//sd3
		for(i=20*2;i<20*3;i++)
	  ch[cnt_++]=sd_save[i];
		sd_write(ch,"sd3.txt", index, t_now,cnt_,1);cnt_=0;		
		//--------
	  t_now=t_now+delta_t;
		sd.save_time=t_sd_save=(float)t_now/1000.;
	}
	
	break;
	default:state=0;en_save=0;init_sd_write=0;t_now=0;break;

}	
}