#include "myiic.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "stm32f10x.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//IIC���� ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/9
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
 
//��ʼ��IIC
void IIC_Init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );	//ʹ��GPIOBʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //������� ��ԭ��
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;  	/* ��©��� */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB,GPIO_Pin_10|GPIO_Pin_11); 	//PB10,PB11 �����
	//GPIO_SetBits(GPIOB,GPIO_Pin_6|GPIO_Pin_7);
	I2C_Cmd(I2C2,ENABLE);
}
//����IIC��ʼ�ź�
void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_us(4);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL=1; 
	IIC_SDA=1;//����I2C���߽����ź�
	delay_us(4);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
	IIC_SDA=1;delay_us(1);	   
	IIC_SCL=1;delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}
//������ACKӦ��		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        //IIC_SDA=(txd&0x80)>>7;
		if((txd&0x80)>>7)
			IIC_SDA=1;
		else
			IIC_SDA=0;
		IIC_SCL=1;
		delay_us(2);   //��TEA5767��������ʱ���Ǳ����
		txd<<=1; 
		delay_us(2); 
		IIC_SCL=0;	
		delay_us(2);
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        delay_us(2);
				IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
				delay_us(1); 
  }					 
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}



uint8_t i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
{
	  IIC_Init();	
    IIC_Start(); 
		
	  printf("begin\n");
    IIC_Send_Byte(addr | 0x00);
    if (IIC_Wait_Ack()==1) 
			{
        IIC_Stop();
				printf("��ַӦ��no�ɹ�\n");
				return 0;
      }
			else
				printf("��ַ�ɹ�Ӧ��\n");
		 IIC_Send_Byte(reg);
     if (IIC_Wait_Ack()==1) 
			{
//        i2c_Stop();
				printf("�Ĵ�����ַӦ��no�ɹ�\n");
    }
				else
				printf("�Ĵ�����ַӦ��ɹ�\n");
        IIC_Send_Byte(data);
        if (IIC_Wait_Ack()==1) 
			{
//        i2c_Stop();
				printf("����Ӧ��no�ɹ�\n");
    }
				else
				printf("����Ӧ��ɹ�\n");
    IIC_Stop();
    return 1;
}



u8 i2c_CheckDevice(uint8_t _Address)
{
	uint8_t ucAck;

	IIC_Init();		/* ����GPIO */

	IIC_Start();		/* ���������ź� */
  
	/* �����豸��ַ+��д����bit��0 = w�� 1 = r) bit7 �ȴ� */
        ///////////////////////��Ȼ����λ������������������������
	IIC_Send_Byte(_Address<<1 | 0x00);
	ucAck = IIC_Wait_Ack();	/* ����豸��ACKӦ�� */
	IIC_Stop();			/* ����ֹͣ�ź� */
        if(!ucAck){
          printf("ͬ��ַΪ%d��PCA9685ͨѶ�ɹ�\r\n",_Address);
					return 1;
				}
				else
				{
					printf("ͨѶʧ��");
					return 0;
				}
}









































