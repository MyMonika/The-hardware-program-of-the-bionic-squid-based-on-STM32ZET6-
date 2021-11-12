#include "encoder.h"
#define	ENCODER_PPR 390 
int Encoder_Timer_Overflow;                                      //编码器溢出次数（每389*4溢出一次）
u16 Previous_Count;                                              //上次TIM2->CNT的值
//初始化编码器
//这里要设定编码器的A相与B相
void encoder_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	
	
	//首先对于A B相所在的GPIO口进行设置，所用到的GPIO是PA0与PA1
//	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM2, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	

	
	
	//对于时基单元的设定部分
	//Specifies the prescaler value used to divide the TIM clock.
	//也就是说，这里的TIM2的时钟信号其实是由A/B相的频率来决定的，类似于外部时钟，然后分频就是对这个脉冲频率分频，比如二分频就是把两个脉冲记为一个脉冲。
	TIM_TimeBaseStructure.TIM_Prescaler = 0;					//不分频
	TIM_TimeBaseStructure.TIM_Period = 65535;					//每来一个脉冲信号的上升沿（下面有设置）计数值就累加（或累减），65535则为最大计数值，就溢出了
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //这里按理来说应该不起作用，因为计数方向是受TI1和TI2信号的影响的
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	
	
	
	//用默认值来初始化输入捕获单元结构体，目的是懒得自己写
	TIM_ICStructInit(&TIM_ICInitStructure);						//Fills each TIM_ICInitStruct member with its default value
	
	//对第一个相位（通道？）的初始化
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_1|TIM_Channel_2;                 //选择输入端IC1映射到TI1上
  TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;	      //上升沿捕获
  TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;   //映射到TI1上
  TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;	            //配置输入分频,不分频 
  TIM_ICInitStructure.TIM_ICFilter =6;                            //配置输入滤波器
  TIM_ICInit(TIM2,&TIM_ICInitStructure);
	
	//对第2个相位（通道？）的初始化
//	TIM_ICInitStructure.TIM_Channel=TIM_Channel_2;                  //选择输入端IC2映射到TI2上
//  TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;	      //上升沿捕获
//  TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;   //映射到TI2上
//  TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;	            //配置输入分频,不分频 
//  TIM_ICInitStructure.TIM_ICFilter=6;                             //配置输入滤波器
//  TIM_ICInit(TIM2,&TIM_ICInitStructure);
	
	//配置为编码器模式
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising ,TIM_ICPolarity_Rising); //配置为编码器模式，计数器在TI1和TI2上升沿处均计数
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;                   //定时器3中断分组配置
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;                   //使能
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01;      //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0x01;            //响应优先级1
	NVIC_Init(&NVIC_InitStructure);                                 //配置定时器3
		
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);                        //允许定时器3更新中断
	
	TIM_SetCounter(TIM2, 0);		//将脉冲计数值设为零
	TIM_Cmd(TIM2, ENABLE);			//使能TIM2

}
//定时器2的中断
void TIM2_IRQHandler(void)                                        //定时器3中断服务函数
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET)                    //溢出中断
	{   
		Encoder_Timer_Overflow++;     		
	}
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);                      //清除中断标志位
}
// 读取定时器计数值
uint32_t read_cnt(void)
{
//	uint32_t encoder_cnt;
//	encoder_cnt = TIM2->CNT;		//读取计数器CNT的值，CNT系uint32_t型的变量
//	TIM_SetCounter(TIM2, 0);		//每一次读取完计数值后将计数值清零，重新开始累加脉冲，方便下一次计数
//	return encoder_cnt;				//返回的值就是本次读到的计数值
	u32 Count;                                                      //一段时间内转过的脉冲数
  u16 Current_Count;                                              //当前TIM2->CNT的值
	u16 Enc_Timer_Overflow_one;	                                   

  Enc_Timer_Overflow_one=Encoder_Timer_Overflow;                  
  Current_Count = TIM2->CNT;                                      //获得当前TIM2->CNT的值
  Encoder_Timer_Overflow=0;                                       //清零，方便下次计算
	if((TIM2->CR1&0x0010) == 0x0010)                                //如果反转
    Count = (u32)((Enc_Timer_Overflow_one)* -1*(4*ENCODER_PPR) - (Current_Count - Previous_Count));   //计算出一个时间转过的脉冲数
	else                                                            //如果正转
		Count = (u32)(Current_Count - Previous_Count + (Enc_Timer_Overflow_one) * (4*ENCODER_PPR));       //计算出一个时间转过的脉冲数
  Previous_Count = Current_Count;  
  return(Count);
}



