/*PB6是IIC_SCL
PB7是IIC_SDA
OE默认接地*/
#ifndef __PCA9685_H
#define __PCA9685_H
#include "myiic.h"


#define uint8 unsigned char
#define uint16 unsigned int
#define uint32 unsigned long
	
#define PCA9685_SUBADR1 0x02 
#define PCA9685_SUBADR2 0x03 
#define PCA9685_SUBADR3 0x04 
#define pca_adrr 0x80
#define pca_mode1 0x00 
#define pca_pre 0xFE //控制周期的寄存器
#define LED0_ON_L 0x06 //第0路
#define LED0_ON_H 0x07 
#define LED0_OFF_L 0x08 
#define LED0_OFF_H 0x09 
#define ALLLED_ON_L 0xFA 
#define ALLLED_ON_H 0xFB 
#define ALLLED_OFF_L 0xFC 
#define ALLLED_OFF_H 0xFD 

void pca_write(u8 adrr,u8 data);

u8 pca_read(u8 adrr);

void pca_setfreq(float freq);

void pca_setpwm(u8 num, u32 on, u32 off);

//void Rotate1(void);

//void Rotate2(void);

//void Rotate3(void);

#endif