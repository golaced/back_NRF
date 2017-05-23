
#include "time.h"
#include "include.h"

volatile uint32_t sysTickUptime = 0;

#define TICK_PER_SECOND 1000 
#define TICK_US	(1000000/TICK_PER_SECOND)

void  SysTick_Configuration(void)
{
	RCC_ClocksTypeDef  rcc_clocks;
	uint32_t         cnts;

	RCC_GetClocksFreq(&rcc_clocks);

	cnts = (uint32_t)rcc_clocks.HCLK_Frequency / TICK_PER_SECOND;
	cnts = cnts / 8;

	SysTick_Config(cnts);
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
}

uint32_t GetSysTime_us(void) 
{
//	register uint32_t ms;
//	u32 value;
//	ms = sysTickUptime;
//	value = ms * TICK_US + (SysTick->LOAD - SysTick->VAL) * TICK_US / SysTick->LOAD;
	return micros();
}

void Delay_us(uint32_t us)
{
 delay_us(1);
}

void Delay_ms(uint32_t ms)
{
    while (ms--)
        Delay_us(1000);
}

int time_1h,time_1m,time_1s,time_1ms;


volatile float Cycle_T[GET_TIME_NUM][3];

enum
{
	NOW = 0,
	OLD,
	NEW,
};

float Get_Cycle_T(u8 item)	
{
	Cycle_T[item][OLD] = Cycle_T[item][NOW];	//上一次的时间
	Cycle_T[item][NOW] = (float)micros()/1000000.0f; //GetSysTime_us()/1000000.0f; //本次的时间
	Cycle_T[item][NEW] = ( ( Cycle_T[item][NOW] - Cycle_T[item][OLD] ) );//间隔的时间（周期）
	return Cycle_T[item][NEW];
}

void Cycle_Time_Init()
{
	u8 i;
	for(i=0;i<GET_TIME_NUM;i++)
	{
		Get_Cycle_T(i);
	}

}


#define TIME_SYS TIM2
#define TIME_SYS_RCC RCC_APB1Periph_TIM2
/**************************实现函数********************************************
*函数原型:		void Initial_Timer3(void)
*功　　能:	  初始化Tim2  Tim3 将两个定时器级联，以产生一个32位的定时器来提供系统us 级的计时	
输入参数：无
输出参数：没有	
*******************************************************************************/
void Initial_Timer_SYS(void)
{ RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
 // RCC->APB1ENR |= 0x0008;	//使能TIM5时钟
	TIME_SYS->CR1 = 0x0080; //TIMx_ARR buffered  upcounter
	TIME_SYS->CR2 = 0x0000;
	TIME_SYS->CNT = 0x0000;
	TIME_SYS->ARR = 0xFFFFFFFF;
	TIME_SYS->PSC = 84 - 1;	//分出 1M 的时钟 保证每个周期为1us
	TIME_SYS->EGR = 0x0001;
	TIME_SYS->CR1 |= 0x0001; //启动定时器           
}


/**************************实现函数********************************************
*函数原型:		uint32_t micros(void)
*功　　能:	  读取系统运行的时间 ，返回单位为us 的时间数。	
输入参数：无
输出参数：处理器当前时间，从上电开始计时  单位 us
*******************************************************************************/
uint32_t micros(void)
{
 	uint32_t temp=0 ;
 	temp = TIME_SYS->CNT;
 	return temp;
}



extern u8  TIM3CH1_CAPTURE_STA;		//输入捕获状态		    				
extern u16	TIM3CH1_CAPTURE_VAL;	//输入捕获值	
extern u16 ppm_rx[];

TIM_ICInitTypeDef  TIM3_ICInitStructure;

void TIM3_Cap_Init(u16 arr,u16 psc)
{	 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
   	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  	//TIM5时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//使能PORTA时钟	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; //GPIOA0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //下拉
	GPIO_Init(GPIOB,&GPIO_InitStructure); //初始化PA0

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_TIM3); //PA0复用位定时器5
  
	TIM_TimeBaseStructure.TIM_Period = arr; //设定计数器自动重装值 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 	//预分频器   
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	

	//初始化TIM5输入捕获参数
	TIM3_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	选择输入端 IC1映射到TI1上
  TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  TIM3_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
  TIM_ICInit(TIM3, &TIM3_ICInitStructure);
		
	TIM_ITConfig(TIM3,TIM_IT_Update|TIM_IT_CC1,ENABLE);//允许更新中断 ,允许CC1IE捕获中断	
	
  TIM_Cmd(TIM3,ENABLE ); 	//使能定时器5

 
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
   


}

u32 temp=0;
u8  TIM3CH1_CAPTURE_STA=0,ppm_rx_sta=0,ppm_rx_num=0;	//输入捕获状态		    				
u16	TIM3CH1_CAPTURE_VAL;	//输入捕获值
u16 ppm_rx[10];//ppm_rx[0]   1   接收到ppm数据
//定时器5中断服务程序	 
void TIM3_IRQHandler(void)
{ 

 	if((TIM3CH1_CAPTURE_STA&0X80)==0)//还未成功捕获	
	{	  
		if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
		 
		{	    
			if(TIM3CH1_CAPTURE_STA&0X40)//已经捕获到高电平了
			{
				if((TIM3CH1_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
				{
					TIM3CH1_CAPTURE_STA|=0X80;//标记成功捕获了一次
					TIM3CH1_CAPTURE_VAL=0XFFFF;
				}else TIM3CH1_CAPTURE_STA++;
			}	 
		}
	if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)//捕获1发生捕获事件
		{	
			if(TIM3CH1_CAPTURE_STA&0X40)		//捕获到一个下降沿 		
			{	  			
				TIM3CH1_CAPTURE_STA|=0X80;		//标记成功捕获到一次高电平脉宽
				TIM3CH1_CAPTURE_VAL=TIM_GetCapture1(TIM3);
		   		TIM_OC1PolarityConfig(TIM3,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
			}else  								//还未开始,第一次捕获上升沿
			{
				TIM3CH1_CAPTURE_STA=0;			//清空
				TIM3CH1_CAPTURE_VAL=0;
	 			TIM_SetCounter(TIM3,0);
				TIM3CH1_CAPTURE_STA|=0X40;		//标记捕获到了上升沿
		   		TIM_OC1PolarityConfig(TIM3,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
			}		    
		}			     	    					   
 	}
	
	//处理帧数据
		if(TIM3CH1_CAPTURE_STA&0X80)//成功捕获到了一次上升沿
		{
		
			if(ppm_rx_sta==1) {
				if((ppm_rx[3]>1019+2||ppm_rx[3]<1019-2)&&ppm_rx[3]>1000){
				Rc_Get_PPM.update=1;Rc_Get_PPM.lose_cnt=0;
				}
				if(TIM3CH1_CAPTURE_VAL+400>0&&TIM3CH1_CAPTURE_VAL+400<2000){
			ppm_rx[ppm_rx_num+1]=TIM3CH1_CAPTURE_VAL+400;ppm_rx_num++;}
			}//printf("TIM3CH1_CAPTURE_VAL:%d\r\n",TIM3CH1_CAPTURE_VAL);
			if(4>TIM3CH1_CAPTURE_STA&0X3F>0||TIM3CH1_CAPTURE_VAL>3000) ppm_rx_sta++;//低电平时间大于3000us为起始帧
			if(ppm_rx_sta==2) {ppm_rx_sta=0;ppm_rx[0]=1;ppm_rx_num=0;}//printf("receive\r\n");//ppm_rx_sta   1 表示接收到同步帧/ 2接收到到下一起始帧 ppm数据接收完毕
			
			TIM3CH1_CAPTURE_STA=0;//开启下一次捕获
			
		}
				
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC1|TIM_IT_Update); //清除中断标志位
 
}


void TIM5_Int_Init(u16 arr,u16 psc)
{ GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);  ///使能TIM3时钟
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStructure);//初始化TIM3
	
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE); //允许定时器3更新中断
	TIM_Cmd(TIM5,ENABLE); //使能定时器3
	
	    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;            //TIM2中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2 ;  //先占优先级0级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;         //从优先级3级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            //IRQ通道被使能
    NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
}

float time5;
 void TIM5_IRQHandler(void)//2ms_intrupt
{ static u8 flag_int=0,state,cnt;
	static u16 ms8=0,ms40=0,ms6=0,ms_sd=0,ms1000;	//中断次数计数器
	static u8 sel;
	if(TIM_GetITStatus(TIM5,TIM_IT_Update)==SET) //溢出中断
	{		time5 = Get_Cycle_T(15);		
	TIM_ClearITPendingBit(TIM5,TIM_IT_Update);  //清除中断标志位
		
		
		
//		switch(sel){
//			case 0:Send_RC_TO_FC(0);
//			sel=1;
//		  break;
//			case 1:Send_RC_TO_FC(1);
//			sel=0;
//		  break;
//		}
		
						if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)!=RESET)//等待DMA2_Steam7传输完成
							{ 	
						  DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);//清除DMA2_Steam7传输完成标志
							clear_nrf_uart();
							switch(sel){
								case 0:;data_per_uart1_dma(SEND_NRF_RC_PPMSBUS);sel=1;	
									break;
								case 1: 
									  sel=0;
										data_per_uart1_dma(SEND_NRF_RC);
								  break;
							}
							
							USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  //使能串口1的DMA发送     
							MYDMA_Enable(DMA2_Stream7,nrf_uart_cnt+2);     //开始一次DMA传输！	  
							}	
							
	}
}