#include "include.h" 
#include "flash.h"
#include "led_fc.h"
#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"
#include "ucos_task.h"
#include "stm32f4xx_dma.h"
#include "circle.h"
#include "sd.h"
#include "sdio.h"
 /////////////////////////UCOSII启动任务设置///////////////////////////////////
//START 任务
//设置任务优先级
#define START_TASK_PRIO      			20 //开始任务的优先级设置为最低
//设置任务堆栈大小
#define START_STK_SIZE  				64
//任务堆栈	
OS_STK START_TASK_STK[START_STK_SIZE];
//任务函数
void start_task(void *pdata);	
//////////////////////////////////////////////////////////////////////////////
    
OS_EVENT * msg_key;			//按键邮箱事件块	  
OS_EVENT * q_msg;			//消息队列

OS_FLAG_GRP * flags_key;	//按键信号量集
void * MsgGrp[256];			//消息队列存储地址,最大支持256个消息
u8 en_read=1;
int main(void)
{ 
	NVIC_PriorityGroupConfig(NVIC_GROUP);//设置系统中断优先级分组2
	delay_init(168);  //初始化延时函数
	TIM3_Cap_Init(0XFFFF,168/2-1);	//PPM
	Delay_ms(100);
 
  //-------------------------Para Init------------------------------------	
	Para_Init();							//参数初始化
	if(en_read)
	READ_PARM();//读取参数
	Initial_Timer_SYS();
	LED_Init();								//LED功能初始化
	Delay_ms(100);
	my_mem_init(SRAMIN);		//初始化内部内存池 
	my_mem_init(SRAMCCM);		//初始化CCM内存池
	Delay_ms(100);
//	sd_check();
//	if(sd_insert)
//	{
//       if(!SD_Init())
//				{
//				exfuns_init();							//为fatfs相关变量申请内存				 
//				SD_INIT();
//				sd_had_init=1;
//				}
//	}
//------------------------Uart Init-------------------------------------
  Usart1_Init(576000L);			//NRF M100
	#if EN_DMA_UART1 
	MYDMA_Config(DMA2_Stream7,DMA_Channel_4,(u32)&USART1->DR,(u32)SendBuff1,SEND_BUF_SIZE1+2,1);//DMA2,STEAM7,CH4,外设为串口1,存储器为SendBuff,长度为:SEND_BUF_SIZE.
	#endif
	//Usart2_Init(921600L);			//IMU 
	#if EN_DMA_UART2
	MYDMA_Config(DMA1_Stream6,DMA_Channel_4,(u32)&USART2->DR,(u32)SendBuff4,SEND_BUF_SIZE4+2,1);//DMA2,STEAM7,CH4,外设为串口1,存储器为SendBuff,长度为:SEND_BUF_SIZE.
	#endif
	Delay_ms(100);
	Spi1_Init();		
	Nrf24l01_Init(MODEL_RX2,40);//射频频道初始化
  Nrf24l01_Check();
	Usart3_Init(100000);//-------SBUS
//-----------------------Mode &  Flag init--------------------	
//system
#define AUX_SEL 1
#if AUX_SEL //big
  track.control_k=0.325;//云台控制增益3.8;//1.325;//2;//---------------------circle_K
	track.control_k_miss=0.325;//云台控制增益2.8;//;
#else  //small
  track.control_k=0.45;//云台控制增益3.8;//1.325;//2;//---------------------circle_K
	track.control_k_miss=0.45;//云台控制增益2.8;//;
#endif
	track.control_yaw  =0.3;//云台对准左右增益
#if USE_M100
  circle.control_k=-0.12;//45/4;//云台对准上下增益1.325;
#else
	circle.control_k=-0.0525;//45/4;//云台对准上下增益1.325;
#endif
	track.forward=58;//循环侧飞遥控量
	track.forward_end_dj_pwm=1.6;//航向控制增益
	
	//--------------------
	fly_ready=0;
	mode.cal_sel=0;
	mode.height_upload=0;
	mode.dj_lock=0;
	mode.en_h_mode_switch=1;
	mode.en_dj_cal=0;	
	mode.no_head=0;
//pid
	mode.height_safe=0;	
	mode.hunman_pid=0;
	mode.yaw_imu_control=0;
	mode.en_pid_sb_set=0;
//flow
	mode.hold_use_flow=1;
	mode.flow_hold_position=0;//6  
	mode.flow_hold_position_use_global=0;
	mode.flow_hold_position_high_fix=0;
//thr
	mode.thr_fix=0;
	mode.thr_add_pos=1;
	mode.en_moto_smooth=0;
	mode.en_h_mode_switch=1;
//--sb
	mode.sb_smooth=1;
	mode.en_sensor_equal_flp=1;
	mode.mpu6050_bw_42=1;
	mode.sonar_avoid=0;
	mode.en_fuzzy_angle_pid=0;
	mode.en_sd_save=0;//sd_save_flag
	mode.en_imu_q_update_mine=0;
	mode.en_fuzzy_angle_pid=0;
	mode.flow_hold_position_high_fix=0;
	mode.yaw_sel=1;
	mode.pid_mode=0;
	mode.spid=1;
	mode.en_pid_out_pit=1;
	mode.en_pid_out_rol=1;
	mode.en_pid_out_yaw=1;
	mode.en_pid_fuzzy_p=0;
	mode.en_pid_yaw_angle_control=1;//yaw 角度外环
	mode.pit_rol_pid_out_flp=0;
	mode.en_pid_yaw_control_by_pit_rol=1;
	mode.test1=0;
//---circle
  mode.circle_miss_fly=0;
//--
	RX_CH[PITr]=RX_CH[ROLr]=RX_CH[YAWr]=1500;
	RX_CH[THRr]=1000;
	//--遥控器偏执
	RX_CH_FIX[ROLr]=4;//-15;
	RX_CH_FIX[PITr]=-13;//-1;
	RX_CH_FIX[THRr]=0;//7;
	RX_CH_FIX[YAWr]=1;//11;
	//--
	RX_CH[AUX1r]=500;
	RX_CH[AUX2r]=500;
	RX_CH[AUX3r]=500;
	RX_CH[AUX4r]=500;
  m100.Rc_gear=-4545;
	m100.Rc_mode=-8000;
  TIM2_Int_Init(20000-1,84-1);	//定时器时钟84M，分频系数8400，所以84M/8400=10Khz的计数频率，计数5000次为500ms      TIM2_Config(

	//-----------------DMA Init--------------------------
#if EN_DMA_UART2
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);  //使能串口1的DMA发送     
	MYDMA_Enable(DMA1_Stream6,SEND_BUF_SIZE4+2);     //开始一次DMA传输！	
#endif
#if EN_DMA_UART1 
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  //使能串口1的DMA发送     
	MYDMA_Enable(DMA2_Stream7,SEND_BUF_SIZE1+2);     //开始一次DMA传输！	 
#endif	

	OSInit();  	 				
	OSTaskCreate(start_task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );//创建起始任务
	OSStart();	    
}
 

//开始任务
void start_task(void *pdata)
{
  OS_CPU_SR cpu_sr=0;
	u8 err;	    	    
	pdata = pdata; 	
	msg_key=OSMboxCreate((void*)0);		//创建消息邮箱
	q_msg=OSQCreate(&MsgGrp[0],256);	//创建消息队列
 	flags_key=OSFlagCreate(0,&err); 	//创建信号量集		  
	  
	OSStatInit();					//初始化统计任务.这里会延时1秒钟左右
	//注册软件定时器
	tmr1=OSTmrCreate(10,10,OS_TMR_OPT_PERIODIC,(OS_TMR_CALLBACK)tmr1_callback,0,"tmr1",&err);		//100ms执行一次  cpu使用率
	OSTmrStart(tmr1,&err);//启动软件定时器1				 	
	tmr2=OSTmrCreate(10,5,OS_TMR_OPT_PERIODIC,(OS_TMR_CALLBACK)tmr2_callback,0,"tmr2",&err);		//50ms执行一次  LED&&MODE
	OSTmrStart(tmr2,&err);//启动软件定时器1				 	
 	OS_ENTER_CRITICAL();			//进入临界区(无法被中断打断)    
 	//注册线程 	
  //OSTaskCreate(mems_task,(void *)0,(OS_STK*)&MEMS_TASK_STK[MEMS_STK_SIZE-1],MEMS_TASK_PRIO);		
 	OSTaskCreate(inner_task,(void *)0,(OS_STK*)&INNER_TASK_STK[INNER_STK_SIZE-1],INNER_TASK_PRIO);	 
	OSTaskCreate(outer_task,(void *)0,(OS_STK*)&OUTER_TASK_STK[OUTER_STK_SIZE-1],OUTER_TASK_PRIO);
	//OSTaskCreate(baro_task,(void *)0,(OS_STK*)&BARO_TASK_STK[BARO_STK_SIZE-1],BARO_TASK_PRIO);
	//OSTaskCreate(sonar_task,(void *)0,(OS_STK*)&SONAR_TASK_STK[SONAR_STK_SIZE-1],SONAR_TASK_PRIO);	
	OSTaskCreate(nrf_task,(void *)0,(OS_STK*)&NRF_TASK_STK[NRF_STK_SIZE-1],NRF_TASK_PRIO);
	#if !USE_M100
	OSTaskCreate(uart_task,(void *)0,(OS_STK*)&UART_TASK_STK[UART_STK_SIZE-1],UART_TASK_PRIO);
	#endif
	OSTaskCreate(flow_task,(void *)0,(OS_STK*)&FLOW_TASK_STK[FLOW_STK_SIZE-1],FLOW_TASK_PRIO);
	#if USE_M100
	OSTaskCreate(m100_task,(void *)0,(OS_STK*)&M100_TASK_STK[M100_STK_SIZE-1],M100_TASK_PRIO);
	#endif
	//--
 	OSTaskSuspend(START_TASK_PRIO);	//挂起起始任务.
	OS_EXIT_CRITICAL();				//退出临界区(可以被中断打断)
}
   

//信号量集处理任务
void flags_task(void *pdata)
{	
	u16 flags;	
	u8 err;	    						 
	while(1)
	{
		flags=OSFlagPend(flags_key,0X001F,OS_FLAG_WAIT_SET_ANY,0,&err);//等待信号量
 		
		OSFlagPost(flags_key,0X001F,OS_FLAG_CLR,&err);//全部信号量清零
 	}
}
   		    


