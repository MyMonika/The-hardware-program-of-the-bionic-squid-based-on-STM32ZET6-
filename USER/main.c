/*д����������˵Ļ���
@attention
1.�ⲿ��ʵ�ֵ���������˶������ݷ��� �����㽫���ݷ��͵����ں���ͨ��SI4463��Ƶ�źŷ��͵�����
Ҳ����˵�����δ���������źŵĽ��ܲ���ʼ�ն�����ᱻ���õ�

2.ͬʱ������ԭ�ӵ�USART�Ⲣδ�ã� ���Դ��ڲ��ִ���ʹ�õ�����ħ�Ĺ���drv-usart�ļ�

3.���У�ʹ����2��IICЭ������䣬һ����MPU����һ����16·�����Ի���һЩ�ظ�������
�����Ҳ�û�кϲ��������ļ�����ΪMPU�Ǹ�����̫�˽⣬ǣһ������Ҫ��MPU��ȫ�����Բ�������
ʹ���ǲ�����ʹ�õģ����Ǵ��뿴�������˵�
*/

#include "sys.h"	
#include "delay.h"	
#include "mpu6050.h"   
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "encoder.h"
#include "dht11.h" 
#include "timer.h"
#include "main.h"				//ϣ��ע�ͣ�main.h �к���TX/RX�����SPI/Ӳ��SPIѡ������ѡ��
#include "drv_uart.h"
#include "drv_button.h"
#include "drv_delay.h"
#include "drv_led.h"
#include "drv_spi.h"
#include "drv_SI446x.h"
#include "stm32f10x.h"
#include "pca9685.h"
#include <stdlib.h>

//ϣ��ע�������������������Ҫ�ڴ˴������ı����Լ��������õ����м���ֵ
//�м���ֵ���ǲ��ᱻ���͵���ֵ������ת����ʱ������Ҫ��
//���ת����õ���
	u32 count;
	u8 flag;
	u8 frequency;
	uint16_t i = 0;
	uint8_t g_UartRxFlag = 0; //����Ҫ���͵��ֽ�
  uint8_t g_UartRxBuffer[ 64 ] = { 0 };//ϣ��ע��Uart�Ľ��ջ���,Ҳ�������н������ǵĻ���
  uint8_t g_SI4463ItStatus[ 9 ] = { 0 };
  uint8_t g_SI4463RxBuffer[64] = { 0 }; //ϣ��ע��SI4463�Ľ��ջ���
	//ע�ͣ�temp�ǽ��ջ���Ķ�����commandʹ����ָ��
	uint8_t temp=48;
	uint8_t* command=&temp;
	u8 isRun=0; //Ĭ�������ǲ����˶���
	
	//��������ʹ�õ�������
	//�������õ����м�����
	uint8_t t=0,report=1;			//Ĭ�Ͽ����ϱ�
	uint8_t key;
	float pitch,roll,yaw; 		//ŷ����
	short aacx,aacy,aacz;		  //���ٶȴ�����ԭʼ����
	short gyrox,gyroy,gyroz;	//������ԭʼ����
	
//����ı䷽����*****************************************************
void changeDirection(u8 result){
	//���ȼ���pwm��
	TIM_SetCompare3(TIM4,100);//PB8 
	if(result==1){
		GPIO_ResetBits(GPIOE,IN2_PIN); //�͵�ƽ
		GPIO_SetBits(GPIOE,IN1_PIN);  //�ߵ�ƽ
		}
	else if(result==0) {
		GPIO_SetBits(GPIOE,IN2_PIN); //�ߵ�ƽ
		GPIO_ResetBits(GPIOE,IN1_PIN);  //�͵�ƽ
	}
}





//SI4463оƬ�������ݺ���*****************************************************
uint16_t ReceiveCommand(){
	//��һ�ε�����ʼ��Ϊ0��ʱ����Ҫ�ȴ�
	//�ڶ�������Ҫ������Ϣ����0 �������������Ϣ������Ҫ
	while(1)
	{
		SI446x_Interrupt_Status( g_SI4463ItStatus );		//���ж�״̬
//		printf("��ʼ�ж϶�ȡ \n");

		if( g_SI4463ItStatus[ 3 ] & ( 0x01 << 4 ))
      {
			i = SI446x_Read_Packet( g_SI4463RxBuffer );		//��FIFO����		
				//������ݳ��Ȳ�����0
			temp=g_SI4463RxBuffer[0];
			printf("temp= %d\r\n",temp);
//				printf("command= %d\r\n",*command);
			if( i != 0 )
			{
				//�򴮿�������յ����ֽ�
				drv_uart_tx_bytes( g_SI4463RxBuffer,i );	//������յ����ֽ�
			}
			//SI4463����д��
			SI446x_Change_Status( 6 );
			while( 6 != SI446x_Get_Device_Status( ));
			SI446x_Start_Rx(  0, 0, PACKET_LENGTH,0,0,3 );
		}
		else
		{
			if( 3000 == i++ )
			{
				i = 0;
				SI446x_Init( );
			}
			drv_delay_ms( 1 );	
			
		}
		//��һ��������û��������Ҫ������temp=0ʱ������������
		if(isRun==0&&*command!=48)  {
			//��������״̬Ϊ�˶�
			isRun=1;
			return temp;
		}
		//��һ�������㶯����������Ҫ������temp=0ʱ��������ֹ
		if(isRun==1&&*command==48)  {
				 //��������״̬Ϊ��ֹ
			while(count!=0) printf("count== %d\r\n",count);
			isRun=0;
			frequency=0;
			delay_us(800);
			GPIO_ResetBits(GPIOE,IN2_PIN); //�͵�ƽ
			GPIO_ResetBits(GPIOE,IN1_PIN);  //�׵�ƽ
			return temp;
		}
	}
}



//usart��������Ҫ�ĳ�ʼ��*******************************************************
void baseInit(){
	drv_uart_init( 9600 );
	drv_delay_init();
	delay_init();	 
	drv_led_init( );
	
	//�ж����ȼ��ĳ�ʼ��
	//����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}



//ͨѶ���ֵĳ�ʼ��***********************************
void newsReportInit(){
	drv_spi_init( );
	SI446x_Init( );

}



//pca9685��ʼ��*************************************
void pca9685Init(){
	IIC_Init();//IIC��ʼ��
	IIC_Start();
  pca_write(pca_mode1,0x00);//д��ַ
}

//��ʱ���趨ʱ��****************
void setTIM_PWM(){
	TIM4_PWM_Init(200,7200-1);//����ж�
	TIM5_Int_Init(500-1,8400-1);  //DHT��MPU�ж�                   //84M/8400=10Khz,10Khz/500 = 20hz 1��    
}

//�����豸�ĳ�ʼ��****************************
void OtherDevice(){
	MPU_Init();					//��ʼ��MPU6050
	DHT11_Init();        ////DHT11��ʼ��
}




//��ʼ���ṹ��ı���**************************
void Send_Data_stru_Init(Send_Data_stru *sd){
	uint8_t i1 ;
//	sd->mpuString="MPUsending";
//	sd->DHTString="DHTsending";
//	sd->MotorString="Motor";
	//DHT11�õ���
	sd->temperature=0;  //�¶�	
	sd->humidity=0;	  //ʪ��
//	//����ͱ������õ��ı���
//	sd->encode=0;//��¼�Ƕ�
//	sd->speed=0;//��¼�ٶ�
	//mpu�ĳ�ʼ��
	for(i1=0;i1<9;i1++) sd->tbuf[i1]=0;
}


//mpu���ݵĽ���**************************************************
void usart1_report_imu(Send_Data_stru *sd,short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw)
{
	//��ʵ&0xffû��ʲô�ô�������ʵ��������ɾ�ˣ����Ϳ�����
	uint8_t i;
	for(i=0;i<9;i++){;//?0
		sd->tbuf[0]=aacx&0XFF;
		sd->tbuf[1]=aacy&0XFF;
		sd->tbuf[2]=aacz&0XFF; 
		sd->tbuf[3]=gyrox&0XFF;
		sd->tbuf[4]=gyroy&0XFF;
		sd->tbuf[5]=gyroz&0XFF;	
		sd->tbuf[6]=roll&0XFF;
		sd->tbuf[7]=pitch&0XFF;
		sd->tbuf[8]=yaw&0XFF;
	}   
}



///*��ʱ����ʹ�ã�
//���б�����ʹ�õĶ�ʱ��ΪTIM2������TIM2�ṩ���ж�ȥ�õ��������������ռ1����Ӧ2�����Ǳ�����ֻ��д���ˣ���û�н�������
//���ʹ�õĶ�ʱ����PWM��ΪTIM4����ռ1����Ӧ1
//��������ȡ���ж���TIM5 ȥ��¼������Ҫ��ֵ ��ռ1����Ӧ3
//USART�� PA9 PA10
//LED��PB5 PE5
//Button��PE4
//DHT11��PG11
//MPU6050��PC11 PC12
//IIC��MPU��PB11��PB12
//spi��PA4 PA5 PA6 PA7
//SI4463:PF11 PF12 PF13 PF14 PB1 PB2
//IIC(���):PC10 PC11
//Encoder��PA0,PA1
//Motor��PB8,PE10,PE11
//*/
 u8 res=0;
int main(void)
 { 
	 //��ʱȡ���˵�������д���
	baseInit();
	  printf("�����豸��ʼ���ɹ�\r\n");
	newsReportInit();
	pca9685Init();
	printf("�����豸��ʼ���ɹ�\r\n");
	flag=1;
	count=0;
	
	//��ʼ��������
	sds=(Send_Data_stru*)malloc(sizeof(Send_Data_stru));
	Send_Data_stru_Init(sds);	 
	 
	
	//ȷ�������豸��ʼ���ɹ�
	if(DHT11_Init()==0) printf("ʪ�ȴ�������ʼ���ɹ�\r\n"); 
	 res=mpu_dmp_init();
	//if(mpu_dmp_init()==0&&MPU_Init()==0) printf("MPU��ʼ���ɹ�");
	printf("res=%d\r\n",res);
	 //���������֣����м���
	 //�ȴ���������
	 while(1){
		ReceiveCommand();	
	//		printf("�ȴ����");
		//��ʼ��ʱ���������
		printf("�ı��˷���\r\n");
		switch (*command)
		{
				case 49: changeDirection(flag); break;
				case 50: changeDirection(flag+1); break;
			default: break;
		}
		printf("�ɹ�\r\n");
		//Ӧ�ô�ʱ�Ű��жϴ򿪣��ÿ��Զ�
		TIM4_PWM_Init(200,7200-1);
		printf("�ٴμ���\r\n");
		//�ٴμ����¼�
		//ReceiveCommand();
	}

}

//��ʱ��4���ж�
void TIM4_IRQHandler(void)                                        //��ʱ��4�жϷ�����
{
	if(TIM_GetITStatus(TIM4,TIM_IT_Update)==SET)                    //����ж�
	{   
		count++;
			//��¼count�Ĵ���
		if(*command!=48){
				printf("count= %d\r\n",count);   	
				if(count>=100){
					delay_us(8000);
					count=0;
					frequency++;
					flag++;
					changeDirection(flag%2);
				}
			}
		else {
			if(count>=100){
					delay_us(8000);
					count=0;
			}
		}
	}
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);                      //����жϱ�־λ
}

//��ʱ��5�жϷ������
void TIM5_IRQHandler(void)   //TIM5�ж�
{

	//��ȡʪ�ȴ�������ֵ
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)  //���TIM5�����жϷ������
	{
		
		DHT11_Read_Data(&(sds->temperature),&(sds->humidity));	//��ȡ��ʪ��ֵ
		printf("ʪ����ֵΪ%d��%d\r\n",sds->temperature,sds->humidity);
		printf("�ı��˷���\r\n");
		
		//��ȡ��������ֵ
	if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
		{ 
			u8  x=0;
			short temp=MPU_Get_Temperature();
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������			
			if(report)
			{
				usart1_report_imu(sds,aacx,aacy,aacz,gyrox,gyroy,gyroz,(int)(roll*100),(int)(pitch*100),(int)(yaw*10));
		
			}
			for(x=0;x<=8;x++) printf("MPU��ֵΪ%d\r\n",sds->tbuf[x]);
			delay_ms(2);
		}

		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);  //���TIMx�����жϱ�־ 
	}
}
