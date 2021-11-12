#include "pca9685.h"
#include "myiic.h"
#include "delay.h"
#include "math.h"

void pca_write(u8 adrr,u8 data)//��PCAд����,adrrd��ַ,data����
{ 
		IIC_Start();
		delay_ms(5);
		IIC_Send_Byte(pca_adrr);
		IIC_Wait_Ack();
		
		IIC_Send_Byte(adrr);
		IIC_Wait_Ack();
		
		IIC_Send_Byte(data);
		IIC_Wait_Ack();
		
		IIC_Stop();
}

u8 pca_read(u8 adrr)//��PCA������
{
		u8 data;
		IIC_Start();
		delay_ms(5);
		IIC_Send_Byte(pca_adrr);
		IIC_Wait_Ack();//���ͺ�ȴ���Ƭ����Ӧ��
	  
		IIC_Send_Byte(adrr);
		IIC_Wait_Ack();
		IIC_Stop();
	
		delay_ms(5);
	
		IIC_Start(); //Ϊʲô��������������
		delay_ms(5);
		IIC_Send_Byte(pca_adrr|0x01);
		IIC_Wait_Ack();
		
	//data�Ƕ�ȡ�����ݵ�λ��
		data=IIC_Read_Byte(0);//����nack
		IIC_Stop();
		
		return data;
}


void pca_setfreq(float freq)//����PWMƵ��
{
		u8 prescale,oldmode,newmode;
		double prescaleval;
		freq *= 0.915; 
		prescaleval = 25000000;
		prescaleval /= 4096;
		prescaleval /= freq;
		prescaleval -= 1;
		prescale =floor(prescaleval + 0.5f);
		//prescale = ((prescaleval/(freq*4096))+0.5)-1;
		oldmode = pca_read(pca_mode1);//pca_mode1 0x0

		newmode = (oldmode & 0x7f) | 0x10; // sleep  0x0&0x7f=0x0  0x0|0x10=0x10
		//newmode = (oldmode & ~0x80) | 0x10; 
		pca_write(pca_mode1, newmode); // go to sleep  D4sleepд1

		pca_write(pca_pre, prescale); // set the prescaler 
		//pca_pre 0xFE�������ڵļĴ��������������Ԥװ��ֵ����

		pca_write(pca_mode1, oldmode);//oldmode 0x0,D4д0�˳�sleepģʽ
		delay_ms(5);

		pca_write(pca_mode1, oldmode | 0xa1); 
		//pca_write(pca_mode1, oldmode | 0x80 | 0x20); 
		//0x0|0xa1=10100001 D6ʹ���ڲ�ʱ�ӣ�D5�ڲ���ַ��д���Զ����ӣ�D0��Ӧ0x70ͨ��i2c��ַ
}

/*num:���PWM�������0~15��on:PWM��������ֵ0~4096,off:PWM�½�����ֵ0~4096
һ��PWM���ڷֳ�4096�ݣ���0��ʼ+1�������Ƶ�onʱ����Ϊ�ߵ�ƽ������������offʱ
����Ϊ�͵�ƽ��ֱ������4096���¿�ʼ�����Ե�on������0ʱ������ʱ,��on����0ʱ��
off/4096��ֵ����PWM��ռ�ձȡ�*/

void pca_setpwm(u8 num, u32 on, u32 off)
{
		pca_write(LED0_ON_L+4*num,on);
		pca_write(LED0_ON_H+4*num,on>>8);
		pca_write(LED0_OFF_L+4*num,off);
		pca_write(LED0_OFF_H+4*num,off>>8);
		/*IIC_Start();
	  
		IIC_Send_Byte(pca_adrr);
		
		IIC_Send_Byte(LED0_ON_L+4*num);
		IIC_Send_Byte(on);
		IIC_Send_Byte(on>>8);
		IIC_Send_Byte(off);
		IIC_Send_Byte(off>>8);
		IIC_Stop();*/


}

u16 calculate_PWM(u8 angle){
    return (int)(204.8*(0.5+angle*1.0/90));
}

/*
��ֵ���㲽�裺
		����ʱ��0.5ms-2.5ms(ռ�ձ�5%-12.5%),Ƶ��330HZ������3.03ms�����ֱ���4096
		0�㣬0.5/3.03*4096=675
		90�㣬1.5/3.03*4096=2017
		180�㣬2.5/3.03*4096=3379
*/
//void Rotate1(u8 num, u32 off)
//{
//	u16 i=103;
//	for(i=103;i<=off;i++)
//	{
//		pca_setpwm(num,0,i);
//		delay_ms(500);
//	}
//}

//void Rotate2(void)//ʳָ��չ
//	{
//	u16 i=0;
//	for(i=675;i<=2200;i++)
//	{
//		pca_setpwm(2,0,i);
//		delay_ms(3);
//	}
//}
//	
//	void Rotate3(void)//��ָ��չ
//{
//	u16 i=0;
//	for(i=675;i<=2800;i++)
//	{
//		pca_setpwm(15,0,i);
//		delay_ms(3);
//	}
//}