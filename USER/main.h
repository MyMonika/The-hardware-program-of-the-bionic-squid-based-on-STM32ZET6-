
#ifndef __MAIN_H__
#define __MAIN_H__


#include "drv_uart.h"
#include "drv_button.h"
#include "drv_delay.h"
#include "drv_led.h"
#include "drv_spi.h"
#include "drv_SI446x.h"


#define 	__SI446X_TX_TEST__ 1						//**@@ 如果测试发送功能则需要定义该宏，如果测试接收则需要屏蔽该宏 **@@//
#define 	__USE_SOFT_SPI_INTERFACE__					//**@@ 如果使用软件SPI则需要定义该宏，反之如果使用硬件SPI则需要屏蔽该宏 **@@//


/** 发送模式定义 */
enum
{
	TX_MODE_Fixed = 0,		//发送模式1，发送固定的字符串
	TX_MODE_USART			//发送模式2，发送串口接收到的数据
};
/*发送的字符串定义*/
enum
{
	Motornum = 0,		//发送模式1，发送固定的字符串
	MPUnum,	//发送模式2，发送串口接收到的数据
	DHTnum,
};
//这里我们定义了最后的发送结构体
typedef struct
{
	//发送用到的原始字符串和缓存
//	const char *mpuString;//希辰注：要发送的字符串
//	const char *DHTString;
//	const char *MotorString;
	//DHT11用到的
	uint8_t temperature;  //温度	
	uint8_t humidity;	  //湿度
	//电机和编码器用到的变量
//	u32 encode;//记录角度
//	float speed;//记录速度
	//mpu的值太多，用一个数组存储
	uint8_t tbuf[9];

}Send_Data_stru;

//定义一个发送结构体指针
Send_Data_stru *sds;

void setTIM_PWM();
void pca9685Init();
void baseInit();
void newsReportInit();
void Send_Data_stru_Init(Send_Data_stru *sd);
void SendAndReceive(uint8_t num,uint8_t TxMode,Send_Data_stru* sd,u8 typeString);
void mpu6050_send_data(Send_Data_stru *sd,short aacx, short aacy, short aacz, short gyrox, short gyroy, short gyroz);
void usart1_report_imu(Send_Data_stru *sd,short aacx, short aacy, short aacz, short gyrox, short gyroy, short gyroz, short roll, short pitch, short yaw);
void left_fin(uint8_t degrees);
void right_fin(uint8_t degrees);
void RunningException(void);
void changeDirection(u8 result);
void Send_Struct(Send_Data_stru *st);

#endif
