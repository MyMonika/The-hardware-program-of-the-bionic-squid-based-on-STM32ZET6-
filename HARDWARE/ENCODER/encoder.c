#include "encoder.h"
#define	ENCODER_PPR 390 
int Encoder_Timer_Overflow;                                      //���������������ÿ389*4���һ�Σ�
u16 Previous_Count;                                              //�ϴ�TIM2->CNT��ֵ
//��ʼ��������
//����Ҫ�趨��������A����B��
void encoder_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	
	
	//���ȶ���A B�����ڵ�GPIO�ڽ������ã����õ���GPIO��PA0��PA1
//	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM2, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	

	
	
	//����ʱ����Ԫ���趨����
	//Specifies the prescaler value used to divide the TIM clock.
	//Ҳ����˵�������TIM2��ʱ���ź���ʵ����A/B���Ƶ���������ģ��������ⲿʱ�ӣ�Ȼ���Ƶ���Ƕ��������Ƶ�ʷ�Ƶ���������Ƶ���ǰ����������Ϊһ�����塣
	TIM_TimeBaseStructure.TIM_Prescaler = 0;					//����Ƶ
	TIM_TimeBaseStructure.TIM_Period = 65535;					//ÿ��һ�������źŵ������أ����������ã�����ֵ���ۼӣ����ۼ�����65535��Ϊ������ֵ���������
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ﰴ����˵Ӧ�ò������ã���Ϊ������������TI1��TI2�źŵ�Ӱ���
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	
	
	
	//��Ĭ��ֵ����ʼ�����벶��Ԫ�ṹ�壬Ŀ���������Լ�д
	TIM_ICStructInit(&TIM_ICInitStructure);						//Fills each TIM_ICInitStruct member with its default value
	
	//�Ե�һ����λ��ͨ�������ĳ�ʼ��
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_1|TIM_Channel_2;                 //ѡ�������IC1ӳ�䵽TI1��
  TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;	      //�����ز���
  TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;   //ӳ�䵽TI1��
  TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;	            //���������Ƶ,����Ƶ 
  TIM_ICInitStructure.TIM_ICFilter =6;                            //���������˲���
  TIM_ICInit(TIM2,&TIM_ICInitStructure);
	
	//�Ե�2����λ��ͨ�������ĳ�ʼ��
//	TIM_ICInitStructure.TIM_Channel=TIM_Channel_2;                  //ѡ�������IC2ӳ�䵽TI2��
//  TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;	      //�����ز���
//  TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;   //ӳ�䵽TI2��
//  TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;	            //���������Ƶ,����Ƶ 
//  TIM_ICInitStructure.TIM_ICFilter=6;                             //���������˲���
//  TIM_ICInit(TIM2,&TIM_ICInitStructure);
	
	//����Ϊ������ģʽ
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising ,TIM_ICPolarity_Rising); //����Ϊ������ģʽ����������TI1��TI2�����ش�������
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;                   //��ʱ��3�жϷ�������
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;                   //ʹ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01;      //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0x01;            //��Ӧ���ȼ�1
	NVIC_Init(&NVIC_InitStructure);                                 //���ö�ʱ��3
		
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);                        //����ʱ��3�����ж�
	
	TIM_SetCounter(TIM2, 0);		//���������ֵ��Ϊ��
	TIM_Cmd(TIM2, ENABLE);			//ʹ��TIM2

}
//��ʱ��2���ж�
void TIM2_IRQHandler(void)                                        //��ʱ��3�жϷ�����
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET)                    //����ж�
	{   
		Encoder_Timer_Overflow++;     		
	}
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);                      //����жϱ�־λ
}
// ��ȡ��ʱ������ֵ
uint32_t read_cnt(void)
{
//	uint32_t encoder_cnt;
//	encoder_cnt = TIM2->CNT;		//��ȡ������CNT��ֵ��CNTϵuint32_t�͵ı���
//	TIM_SetCounter(TIM2, 0);		//ÿһ�ζ�ȡ�����ֵ�󽫼���ֵ���㣬���¿�ʼ�ۼ����壬������һ�μ���
//	return encoder_cnt;				//���ص�ֵ���Ǳ��ζ����ļ���ֵ
	u32 Count;                                                      //һ��ʱ����ת����������
  u16 Current_Count;                                              //��ǰTIM2->CNT��ֵ
	u16 Enc_Timer_Overflow_one;	                                   

  Enc_Timer_Overflow_one=Encoder_Timer_Overflow;                  
  Current_Count = TIM2->CNT;                                      //��õ�ǰTIM2->CNT��ֵ
  Encoder_Timer_Overflow=0;                                       //���㣬�����´μ���
	if((TIM2->CR1&0x0010) == 0x0010)                                //�����ת
    Count = (u32)((Enc_Timer_Overflow_one)* -1*(4*ENCODER_PPR) - (Current_Count - Previous_Count));   //�����һ��ʱ��ת����������
	else                                                            //�����ת
		Count = (u32)(Current_Count - Previous_Count + (Enc_Timer_Overflow_one) * (4*ENCODER_PPR));       //�����һ��ʱ��ת����������
  Previous_Count = Current_Count;  
  return(Count);
}



