/**
  ******************************************************************************
  * @author  泽耀科技 ASHINING
  * @version V3.0
  * @date    2016-10-08
  * @brief   SPI配置C文件
  ******************************************************************************
  * @attention
  *
  * 官网	:	http://www.ashining.com
  * 淘宝	:	https://shop105912646.taobao.com
  * 阿里巴巴:	https://cdzeyao.1688.com
  ******************************************************************************
  */



#include "drv_spi.h"


#ifndef __USE_SOFT_SPI_INTERFACE__

/** 硬件SPI */
#define SPI_WAIT_TIMEOUT			((uint16_t)0xFFFF)

/**
  * @brief :SPI初始化(硬件)
  * @param :无
  * @note  :无
  * @retval:无
  */ 
void drv_spi_init( void )
{
	GPIO_InitTypeDef	SpiGpioInitStructer;
	SPI_InitTypeDef		SpiInitStructer;
	
	/** SPI引脚配置 */
	RCC_APB2PeriphClockCmd( SPI_CLK_GPIO_CLK | SPI_MISO_GPIO_CLK | SPI_MOSI_GPIO_CLK | SPI_NSS_GPIO_CLK, ENABLE );	//打开端口时钟。
	//希辰注：上面这里全是关于引脚A，可能后期要改的，CLK对应的是SCK，而且这里NSS放在了GPIOG的7引脚
	//我在这里经过查验资料发现NSS就是CS
	//最小系统版不提供NSS的引脚，NSS代表是软件控制还是硬件控制
	//NSS：从设备选择。这是一个可选的引脚，用来选择主/从设备。它的功能是用来作为“片选引脚”，
//	让主设备可以单独地与特定从设备通讯，避免数据线上的冲突。
//	从设备的NSS引脚可以由主设备的一个标准I/O引脚来驱动。
//	一旦被使能(SSOE位)，NSS引脚也可以作为输出引脚，并在SPI处于主模式时拉低；
//	此时，所有的SPI设备，如果它们的NSS引脚连接到主设备的NSS引脚，则会检测到低电平，
//	如果它们被设置为NSS硬件模式，就会自动进入从设备状态。
//	当配置为主设备、NSS配置为输入引脚(MSTR=1，SSOE=0)时，如果NSS被拉低，则这个SPI设备进入主模式失败状态：
//	即MSTR位被自动清除，此设备进入从模式
	
	//SCK MOSI MISO 配置为复用  希辰注（推挽复用）--其实还是只对引脚做更改，不涉及SPI
	SpiGpioInitStructer.GPIO_Speed = GPIO_Speed_10MHz;//速度频率为10兆赫
	SpiGpioInitStructer.GPIO_Mode = GPIO_Mode_AF_PP;//希辰注：复用为推挽输出
	
	SpiGpioInitStructer.GPIO_Pin = SPI_CLK_GPIO_PIN;//希辰注：对应的是引脚5
	GPIO_Init( SPI_CLK_GPIO_PORT, &SpiGpioInitStructer );		//初始化SCK
	
	SpiGpioInitStructer.GPIO_Pin = SPI_MOSI_GPIO_PIN;////希辰注：对应的是引脚7
	GPIO_Init( SPI_MOSI_GPIO_PORT, &SpiGpioInitStructer );		//初始化MOSI
	GPIO_SetBits( SPI_MOSI_GPIO_PORT, SPI_MOSI_GPIO_PIN );//希辰注：输出高位

	SpiGpioInitStructer.GPIO_Pin = SPI_MISO_GPIO_PIN; //希辰注：对应的是引脚6
	GPIO_Init( SPI_MISO_GPIO_PORT, &SpiGpioInitStructer );		//初始化MISO
	GPIO_SetBits( SPI_MISO_GPIO_PORT, SPI_MISO_GPIO_PIN ); //希辰注：输出高位



	//NSS配置为推挽输出，用一个独立的IO口的引脚来进行NSS的位设置
	SpiGpioInitStructer.GPIO_Mode = GPIO_Mode_Out_PP; //希辰注：复用为推挽输出
	SpiGpioInitStructer.GPIO_Pin = SPI_NSS_GPIO_PIN; //希辰注：对应的是引脚7
	GPIO_Init( SPI_NSS_GPIO_PORT, &SpiGpioInitStructer );		//初始化NSS
	GPIO_SetBits( SPI_NSS_GPIO_PORT, SPI_NSS_GPIO_PIN );		//置高，//希辰注：此处是对GPIOG进行了置高


//希辰注： 以上全都是对于引脚的配置，对于spi并无任何的配置，NSS可选了GPIOG的引脚7，其他的选择了SPI1
	/** SPI配置 */
	SPI_I2S_DeInit( SPI_PORT );			//复位SPI
	
	if( SPI1 == SPI_PORT )				//希辰注：定义SPI1为SPI_PORT
	{
		RCC_APB2PeriphClockCmd( SPI_PORT_CLK, ENABLE );			//SPI1在APB2上，希辰注：打开SPI1的时钟，进行使能
	}
	else
	{
		RCC_APB1PeriphClockCmd( SPI_PORT_CLK, ENABLE );			//SPI2，3在APB1上。希辰注：对应的引脚全在PB上，这段代码一般不会执行
	}
	
	SPI_Cmd( SPI_PORT, DISABLE );		//关闭SPI外设，配置前关闭
	
	SpiInitStructer.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	//双线全双工
	SpiInitStructer.SPI_Mode = SPI_Mode_Master;							//主机模式
	SpiInitStructer.SPI_CPOL = SPI_CPOL_Low;							//空闲状态为低电平 
	SpiInitStructer.SPI_CPHA = SPI_CPHA_1Edge;							//第一个边沿采集数据
	SpiInitStructer.SPI_DataSize = SPI_DataSize_8b;						//8位数据
	SpiInitStructer.SPI_NSS = SPI_NSS_Soft;								//从机软件管理
	SpiInitStructer.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;	//64分频
	SpiInitStructer.SPI_FirstBit = SPI_FirstBit_MSB;					//最高位先发送
	SpiInitStructer.SPI_CRCPolynomial = 7;								//CRC多项式,默认不使用SPI自带CRC	 
	
	//希辰注：首先设定好SPI1的各个模式，然后初始化SPI
	SPI_Init( SPI_PORT, &SpiInitStructer );
	SPI_Cmd( SPI_PORT, ENABLE ); //希辰注：使能spi的外设
}

/**
  * @brief :SPI收发一个字节
  * @param :
  *			@TxByte: 发送的数据字节
  * @note  :非堵塞式，一旦等待超时，函数会自动退出
  * @retval:接收到的字节
  */
uint8_t drv_spi_read_write_byte( uint8_t TxByte )
{
	uint8_t l_Data = 0;
	uint16_t l_WaitTime = 0;
	
	while( RESET == SPI_I2S_GetFlagStatus( SPI_PORT, SPI_I2S_FLAG_TXE ) )		//等待发送缓冲区为空
		//希辰注：这是为了确保发生前Buffer为空，也就是说上一次已经发生完成
	{
		if( SPI_WAIT_TIMEOUT == ++l_WaitTime )
		{
			break;			//如果等待超时则退出
		}
	}
	l_WaitTime = SPI_WAIT_TIMEOUT / 2;		//重新设置接收等待时间(因为SPI的速度很快，正常情况下在发送完成之后会立即收到数据，等待时间不需要过长)
	SPI_PORT->DR = TxByte;	//发送数据
	
	while( RESET == SPI_I2S_GetFlagStatus( SPI_PORT, SPI_I2S_FLAG_RXNE ) )		//等待接收缓冲区非空
	{
		if( SPI_WAIT_TIMEOUT == ++l_WaitTime )
		{
			break;			//如果等待超时则退出
		}
	}
	
	l_Data = (uint8_t)SPI_PORT->DR;		//读取接收数据
	
	return l_Data;		//返回
}

/**
  * @brief :SPI收发字符串
  * @param :
  *			@ReadBuffer: 接收数据缓冲区地址
  *			@WriteBuffer:发送字节缓冲区地址
  *			@Length:字节长度
  * @note  :非堵塞式，一旦等待超时，函数会自动退出
  * @retval:无
  */
void drv_spi_read_write_string( uint8_t* ReadBuffer, uint8_t* WriteBuffer, uint16_t Length )
{
	GPIO_ResetBits( SPI_NSS_GPIO_PORT, SPI_NSS_GPIO_PIN);			//拉低片选
	while( Length-- )
	{
		*ReadBuffer = drv_spi_read_write_byte( *WriteBuffer );		//收发数据
		ReadBuffer++;
		WriteBuffer++;				//读写地址加1
	}
	GPIO_SetBits( SPI_NSS_GPIO_PORT, SPI_NSS_GPIO_PIN);				//拉高片选
}

//希辰注：之上我们NSS的配置是软件



/** 硬件SPI */
#endif






#ifdef __USE_SOFT_SPI_INTERFACE__

/** 软件SPI */


/**
  * @brief :SPI初始化(软件)
  * @param :无
  * @note  :无
  * @retval:无
  */
void drv_spi_init( void )
{
	GPIO_InitTypeDef	SpiGpioInitStructer;
	
	/** SPI引脚配置 */
	RCC_APB2PeriphClockCmd( SPI_CLK_GPIO_CLK | SPI_MISO_GPIO_CLK | SPI_MOSI_GPIO_CLK | SPI_NSS_GPIO_CLK, ENABLE );	//打开端口时钟
	
	//SCK MOSI NSS配置为推挽输出
	SpiGpioInitStructer.GPIO_Speed = GPIO_Speed_10MHz;
	SpiGpioInitStructer.GPIO_Mode = GPIO_Mode_Out_PP;
	
	SpiGpioInitStructer.GPIO_Pin = SPI_CLK_GPIO_PIN;
	GPIO_Init( SPI_CLK_GPIO_PORT, &SpiGpioInitStructer );		//初始化SCK
	GPIO_ResetBits( SPI_CLK_GPIO_PORT, SPI_CLK_GPIO_PIN);		//初始化状态设置为低
	
	SpiGpioInitStructer.GPIO_Pin = SPI_MOSI_GPIO_PIN;
	GPIO_Init( SPI_MOSI_GPIO_PORT, &SpiGpioInitStructer );		//初始化MOSI
	GPIO_SetBits( SPI_MOSI_GPIO_PORT, SPI_MOSI_GPIO_PIN);		//初始化状态设置为高
	
	SpiGpioInitStructer.GPIO_Pin = SPI_NSS_GPIO_PIN;
	GPIO_Init( SPI_NSS_GPIO_PORT, &SpiGpioInitStructer );		//初始化NSS
	GPIO_SetBits( SPI_NSS_GPIO_PORT, SPI_NSS_GPIO_PIN);			//初始化状态设置为高
	
	//初始化MISO 上拉输入
	SpiGpioInitStructer.GPIO_Mode = GPIO_Mode_IPU;
	SpiGpioInitStructer.GPIO_Pin = SPI_MISO_GPIO_PIN;
	GPIO_Init( SPI_MISO_GPIO_PORT, &SpiGpioInitStructer );		
	GPIO_SetBits( SPI_MISO_GPIO_PORT, SPI_MISO_GPIO_PIN);		//初始化状态设置为高
}

/**
  * @brief :SPI收发一个字节
  * @param :
  *			@TxByte: 发送的数据字节
  * @note  :非堵塞式，一旦等待超时，函数会自动退出
  * @retval:接收到的字节
  */
uint8_t drv_spi_read_write_byte( uint8_t TxByte )
{
	uint8_t i = 0, Data = 0;
	
	spi_set_clk_low( );
	
	for( i = 0; i < 8; i++ )			//一个字节8byte需要循环8次
	{
		/** 发送 */
		if( 0x80 == ( TxByte & 0x80 ))
		{
			spi_set_mosi_hight( );		//如果即将要发送的位为 1 则置高IO引脚
		}
		else
		{
			spi_set_mosi_low( );		//如果即将要发送的位为 0 则置低IO引脚
		}
		TxByte <<= 1;					//数据左移一位，先发送的是最高位
		
		spi_set_clk_high( );			//时钟线置高
		__nop( );
		__nop( );
		
		/** 接收 */
		Data <<= 1;						//接收数据左移一位，先接收到的是最高位
		if( 1 == spi_get_miso( ))
		{
			Data |= 0x01;				//如果接收时IO引脚为高则认为接收到 1
		}
		
		spi_set_clk_low( );				//时钟线置低
		__nop( );
		__nop( );
	}
	
	return Data;		//返回接收到的字节
}

/**
  * @brief :SPI收发字符串
  * @param :
  *			@ReadBuffer: 接收数据缓冲区地址
  *			@WriteBuffer:发送字节缓冲区地址
  *			@Length:字节长度
  * @note  :非堵塞式，一旦等待超时，函数会自动退出
  * @retval:无
  */
void drv_spi_read_write_string( uint8_t* ReadBuffer, uint8_t* WriteBuffer, uint16_t Length )
{
	spi_set_nss_low( );			//片选拉低
	
	while( Length-- )
	{
		*ReadBuffer = drv_spi_read_write_byte( *WriteBuffer );		//收发数据
		ReadBuffer++;
		WriteBuffer++;			//读写地址加1
	}
	
	spi_set_nss_high( );		//片选拉高
}


/** 软件SPI */
#endif


