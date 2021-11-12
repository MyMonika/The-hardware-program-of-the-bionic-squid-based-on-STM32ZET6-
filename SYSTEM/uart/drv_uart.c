
#include "drv_uart.h"
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif
//加入对于print的定义
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//__use_no_semihosting was requested, but _ttywrch was 
_ttywrch(int ch)
{
    ch = ch;
}
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif

/**
  * @brief :串口初始化
  * @param :
  *			@UartBaudRate:串口波特率
  * @note  :无
  * @retval:无
  */
	//希辰注：初始化思路是：
	//首先将引脚初始化，陪置输入输出模式
void drv_uart_init( uint32_t UartBaudRate )
{
	GPIO_InitTypeDef	UartGpioInitStructer;
	USART_InitTypeDef	UartinitStructer;
	
	//在配置过程中，为防止TX RX不再同一个端口上，增强可移植性，固分开配置
	//初始化串口TX RX 引脚 
	RCC_APB2PeriphClockCmd( UART_TX_GPIO_CLK | UART_RX_GPIO_CLK, ENABLE );	//打开TX RX 端口时钟 
	//希辰注 	以上使能时钟是RCC_APB2Periph_GPIOA 和 RCC_APB2Periph_GPIOA
	//代码思路：先是使能，再对一个GPIO的指针进行各种操作后，再将他赋值入GPIOA
	
	UartGpioInitStructer.GPIO_Mode = GPIO_Mode_AF_PP;//配置为复用推挽输出
	UartGpioInitStructer.GPIO_Speed = GPIO_Speed_2MHz;//配置IO口速度设置
	
	//TX 希辰注：transport 输出，TX的推挽就是全双工
	UartGpioInitStructer.GPIO_Pin = UART_TX_GPIO_PIN;//希辰注：打开的是引脚IO 9，设为Tx的输出端，已经审核原理图确保正确
	GPIO_Init( UART_TX_GPIO_PORT, &UartGpioInitStructer );		//初始化TX引脚  配置为复用功能 希辰注：此时还是GPIOA的9引脚，而且是TX的引脚
	
	//RX 希辰注：receive 输入
	UartGpioInitStructer.GPIO_Mode = GPIO_Mode_IN_FLOATING;//GPIO_Mode_AF_PP;//GPIO_Mode_AF_OD;//GPIO_Mode_AIN;//GPIO_Mode_IN_FLOATING;
	//希辰注： 以上暂定为为浮空输入，也就是恢复复位后的状态，Rx的浮空就是全双工
	UartGpioInitStructer.GPIO_Pin = UART_RX_GPIO_PIN; //希辰注：打开的是IO 10 此时是RX的引脚 已经审核原理图确保正确
	GPIO_Init( UART_RX_GPIO_PORT, &UartGpioInitStructer );		//初始化RX引脚  配置为输入
	
	//配置USART外设
	USART_DeInit( UART_PORT );		//外设复位,希辰注：这里是USART1的配置
	
	if( USART1 == UART_PORT )		//使能外设时钟，希辰注：USART1时钟的使能，USART1将意味着配置上面引脚的配置的归属是正确的
	{
		RCC_APB2PeriphClockCmd( UART_PORT_CLK, ENABLE );		//希辰注： USART部分的时钟使能	
	}																	//不同的USART外设可能在不同的APB时钟上														
	else																//STM32F103单片机只有USART1在APB2上，如平台有差异做相应改变即可
	{
		RCC_APB1PeriphClockCmd( UART_PORT_CLK, ENABLE );//希辰注：这是为了适应平台的差异的判断语句，主要是为了判断时钟的使能问题 
	}
	
	UartinitStructer.USART_BaudRate = UartBaudRate;						//设置波特率，希辰注： 波特率这里是用是传入的参数
	UartinitStructer.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//不使用流控制，希辰注：流控制是啥玩意
	UartinitStructer.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;		//发送和接收	希辰注：初始化为双工传输
	UartinitStructer.USART_Parity = USART_Parity_No;					//不带校验
	UartinitStructer.USART_StopBits = USART_StopBits_1;					//一个停止位
	UartinitStructer.USART_WordLength = USART_WordLength_8b;			//8个数据位，希辰注：初始化为8位，也有16位可供选择
	
	USART_Cmd( UART_PORT, DISABLE );									//失能外设
	USART_Init( UART_PORT, &UartinitStructer );							//初始化外设
	USART_Cmd( UART_PORT, ENABLE );										//使能外设	
}

/**
  * @brief :串口发送数据
  * @param :
  *			@TxBuffer:发送数据首地址
  *			@Length:数据长度
  * @note  :无
  * @retval:无
  */
void drv_uart_tx_bytes( uint8_t* TxBuffer, uint8_t Length )
{
	while( Length-- )
	{
		while( RESET == USART_GetFlagStatus( UART_PORT, USART_FLAG_TXE ));
		UART_PORT->DR = *TxBuffer;
		TxBuffer++;
	}
}
/**
  * @brief :串口发送char
  * @param :
  *			@c:待发送的字符
  * @note  :无
  * @retval:无
  */
void usart1_send_char(uint8_t c)
{
	while((USART1->SR&0X40)==0);//等待上一次发送完毕
	UART_PORT->DR=c;   	
} 

/**
  * @brief :串口接收数据
  * @param :
  *			@RxBuffer:发送数据首地址
  * @note  :无
  * @retval:接收到的字节个数
  */
uint8_t drv_uart_rx_bytes( uint8_t* RxBuffer )
{
	uint8_t l_RxLength = 0;
	uint16_t l_UartRxTimOut = 0x7FFF;
	
	while( l_UartRxTimOut-- )			//等待查询串口数据
	{
		if( RESET != USART_GetFlagStatus( UART_PORT, USART_FLAG_RXNE ))
		{
			*RxBuffer = (uint8_t)UART_PORT->DR;
			RxBuffer++;//希辰注：这里是加了一个缓存
			l_RxLength++;//这个是用来计算接收的字符数
			l_UartRxTimOut = 0x7FFF;	//接收到一个字符，回复等待时间
		}
		if( 64 == l_RxLength )
		{
			break;		//不能超过64个字节
		}
	}
	
	return l_RxLength;					//等待超时，数据接收完成
}

