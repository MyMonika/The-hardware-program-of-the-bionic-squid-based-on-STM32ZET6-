/*写给看代码的人的话：
@attention
1.这部分实现的是鱿鱼的运动与数据发送 ，鱿鱼将数据发送到串口后，再通过SI4463射频信号发送到电脑
也就是说，本段代码里面的信号的接受部分始终都不会会被调用的

2.同时，正电原子的USART库并未用， 所以串口部分处理使用的是我魔改过的drv-usart文件

3.还有，使用了2个IIC协议的运输，一个是MPU，另一个是16路，所以会有一些重复的轮子
但是我并没有合并这两个文件，因为MPU那个不算太了解，牵一发可能要动MPU的全身，所以不好下手
使用是不妨碍使用的，就是代码看起来多了点
*/

#include "sys.h"	
#include "delay.h"	
#include "mpu6050.h"   
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "encoder.h"
#include "dht11.h" 
#include "timer.h"
#include "main.h"				//希辰注释：main.h 中含有TX/RX、软件SPI/硬件SPI选择配置选项
#include "drv_uart.h"
#include "drv_button.h"
#include "drv_delay.h"
#include "drv_led.h"
#include "drv_spi.h"
#include "drv_SI446x.h"
#include "stm32f10x.h"
#include "pca9685.h"
#include <stdlib.h>

//希辰注：这里用来存放所有需要在此处命名的变量以及发送所用到的中间数值
//中间数值就是不会被发送的数值，但是转化的时候是需要的
//电机转向会用到的
	u32 count;
	u8 flag;
	u8 frequency;
	uint16_t i = 0;
	uint8_t g_UartRxFlag = 0; //我们要发送的字节
  uint8_t g_UartRxBuffer[ 64 ] = { 0 };//希辰注：Uart的接收缓冲,也会用来承接陀螺仪的缓存
  uint8_t g_SI4463ItStatus[ 9 ] = { 0 };
  uint8_t g_SI4463RxBuffer[64] = { 0 }; //希辰注：SI4463的接收缓存
	//注释，temp是接收缓存的东西，command使他的指针
	uint8_t temp=48;
	uint8_t* command=&temp;
	u8 isRun=0; //默认鱿鱼是不会运动的
	
	//陀螺仪所使用到的数据
	//陀螺仪用到的中间数据
	uint8_t t=0,report=1;			//默认开启上报
	uint8_t key;
	float pitch,roll,yaw; 		//欧拉角
	short aacx,aacy,aacz;		  //加速度传感器原始数据
	short gyrox,gyroy,gyroz;	//陀螺仪原始数据
	
//电机改变方向函数*****************************************************
void changeDirection(u8 result){
	//首先激活pwm波
	TIM_SetCompare3(TIM4,100);//PB8 
	if(result==1){
		GPIO_ResetBits(GPIOE,IN2_PIN); //低电平
		GPIO_SetBits(GPIOE,IN1_PIN);  //高电平
		}
	else if(result==0) {
		GPIO_SetBits(GPIOE,IN2_PIN); //高电平
		GPIO_ResetBits(GPIOE,IN1_PIN);  //低电平
	}
}





//SI4463芯片传输数据函数*****************************************************
uint16_t ReceiveCommand(){
	//第一次当它初始化为0的时候需要等待
	//第二次则需要接收信息等于0 ，但如果接收信息，还需要
	while(1)
	{
		SI446x_Interrupt_Status( g_SI4463ItStatus );		//读中断状态
//		printf("开始中断读取 \n");

		if( g_SI4463ItStatus[ 3 ] & ( 0x01 << 4 ))
      {
			i = SI446x_Read_Packet( g_SI4463RxBuffer );		//读FIFO数据		
				//如果数据长度不等于0
			temp=g_SI4463RxBuffer[0];
			printf("temp= %d\r\n",temp);
//				printf("command= %d\r\n",*command);
			if( i != 0 )
			{
				//向串口输出接收到的字节
				drv_uart_tx_bytes( g_SI4463RxBuffer,i );	//输出接收到的字节
			}
			//SI4463重新写入
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
		//第一次是鱿鱼没动，我们要求当遇到temp=0时候让他动起来
		if(isRun==0&&*command!=48)  {
			//标记鱿鱼的状态为运动
			isRun=1;
			return temp;
		}
		//第一次是鱿鱼动起来，我们要求当遇到temp=0时候让他静止
		if(isRun==1&&*command==48)  {
				 //标记鱿鱼的状态为静止
			while(count!=0) printf("count== %d\r\n",count);
			isRun=0;
			frequency=0;
			delay_us(800);
			GPIO_ResetBits(GPIOE,IN2_PIN); //低电平
			GPIO_ResetBits(GPIOE,IN1_PIN);  //底电平
			return temp;
		}
	}
}



//usart和其他必要的初始化*******************************************************
void baseInit(){
	drv_uart_init( 9600 );
	drv_delay_init();
	delay_init();	 
	drv_led_init( );
	
	//中断优先级的初始化
	//设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}



//通讯部分的初始化***********************************
void newsReportInit(){
	drv_spi_init( );
	SI446x_Init( );

}



//pca9685初始化*************************************
void pca9685Init(){
	IIC_Init();//IIC初始化
	IIC_Start();
  pca_write(pca_mode1,0x00);//写地址
}

//定时器设定时间****************
void setTIM_PWM(){
	TIM4_PWM_Init(200,7200-1);//电机中断
	TIM5_Int_Init(500-1,8400-1);  //DHT与MPU中断                   //84M/8400=10Khz,10Khz/500 = 20hz 1秒    
}

//其他设备的初始化****************************
void OtherDevice(){
	MPU_Init();					//初始化MPU6050
	DHT11_Init();        ////DHT11初始化
}




//初始化结构体的变量**************************
void Send_Data_stru_Init(Send_Data_stru *sd){
	uint8_t i1 ;
//	sd->mpuString="MPUsending";
//	sd->DHTString="DHTsending";
//	sd->MotorString="Motor";
	//DHT11用到的
	sd->temperature=0;  //温度	
	sd->humidity=0;	  //湿度
//	//电机和编码器用到的变量
//	sd->encode=0;//记录角度
//	sd->speed=0;//记录速度
	//mpu的初始化
	for(i1=0;i1<9;i1++) sd->tbuf[i1]=0;
}


//mpu数据的接收**************************************************
void usart1_report_imu(Send_Data_stru *sd,short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw)
{
	//其实&0xff没有什么用处，但我实在是懒得删了，将就看看吧
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



///*定时器的使用：
//其中编码器使用的定时器为TIM2（其中TIM2提供了中断去得到溢出次数），抢占1，响应2，但是编码器只是写好了，并没有进行运行
//电机使用的定时器（PWM）为TIM4，抢占1，响应1
//编码器读取的中断是TIM5 去记录我们需要的值 抢占1，响应3
//USART： PA9 PA10
//LED：PB5 PE5
//Button：PE4
//DHT11：PG11
//MPU6050：PC11 PC12
//IIC（MPU）PB11，PB12
//spi：PA4 PA5 PA6 PA7
//SI4463:PF11 PF12 PF13 PF14 PB1 PB2
//IIC(舵机):PC10 PC11
//Encoder：PA0,PA1
//Motor：PB8,PE10,PE11
//*/
 u8 res=0;
int main(void)
 { 
	 //暂时取消了电机的运行代码
	baseInit();
	  printf("基础设备初始化成功\r\n");
	newsReportInit();
	pca9685Init();
	printf("两个设备初始化成功\r\n");
	flag=1;
	count=0;
	
	//初始化发送体
	sds=(Send_Data_stru*)malloc(sizeof(Send_Data_stru));
	Send_Data_stru_Init(sds);	 
	 
	
	//确保所有设备初始化成功
	if(DHT11_Init()==0) printf("湿度传感器初始化成功\r\n"); 
	 res=mpu_dmp_init();
	//if(mpu_dmp_init()==0&&MPU_Init()==0) printf("MPU初始化成功");
	printf("res=%d\r\n",res);
	 //主函数部分，进行监听
	 //等待输入命令
	 while(1){
		ReceiveCommand();	
	//		printf("等待完成");
		//开始的时候决定方向
		printf("改变了方向\r\n");
		switch (*command)
		{
				case 49: changeDirection(flag); break;
				case 50: changeDirection(flag+1); break;
			default: break;
		}
		printf("成功\r\n");
		//应该此时才把中断打开，让可以动
		TIM4_PWM_Init(200,7200-1);
		printf("再次监听\r\n");
		//再次监听事件
		//ReceiveCommand();
	}

}

//定时器4的中断
void TIM4_IRQHandler(void)                                        //定时器4中断服务函数
{
	if(TIM_GetITStatus(TIM4,TIM_IT_Update)==SET)                    //溢出中断
	{   
		count++;
			//记录count的次数
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
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);                      //清除中断标志位
}

//定时器5中断服务程序
void TIM5_IRQHandler(void)   //TIM5中断
{

	//读取湿度传感器数值
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)  //检查TIM5更新中断发生与否
	{
		
		DHT11_Read_Data(&(sds->temperature),&(sds->humidity));	//读取温湿度值
		printf("湿度数值为%d，%d\r\n",sds->temperature,sds->humidity);
		printf("改变了方向\r\n");
		
		//读取陀螺仪数值
	if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
		{ 
			u8  x=0;
			short temp=MPU_Get_Temperature();
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据			
			if(report)
			{
				usart1_report_imu(sds,aacx,aacy,aacz,gyrox,gyroy,gyroz,(int)(roll*100),(int)(pitch*100),(int)(yaw*10));
		
			}
			for(x=0;x<=8;x++) printf("MPU数值为%d\r\n",sds->tbuf[x]);
			delay_ms(2);
		}

		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);  //清除TIMx更新中断标志 
	}
}
