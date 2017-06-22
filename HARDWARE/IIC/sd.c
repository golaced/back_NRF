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
u8 delta_t;
static u32 t_now;
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