#include <stm32f4xx.h>
#include"defines.h"
#include"LSM303DLHC.h"
#include"I2C.h"
//odblokowanie akcelerometru
void LSM303DLHC_acc_enable(MEMS *mems){
	
	uint8_t data[2];
	
	delay();
	data[0]=0x20;//control reg 1
	data[1]=0x67;//enable axis x y z and odr
	I2C_WRITE(LSM303DLHC,data,2);
	delay();
	data[0]=0x23;//control reg 4
	data[1]=0x08;//BIT HR normal mofe high resolution and BDU block updtate
	I2C_WRITE(LSM303DLHC,data,2);
	delay();
	//data[0]=0x21;
	//data[1]=0x00;
	//I2C_WRITE(LSM303DLHC,data,2);
	delay();
	(*mems).konwersja=A2;
	(*mems).zakres=2;
}

void LSM303DLHC_magnet_enable(MEMS *mems){
	
	uint8_t data[2];
	
	delay();
	data[0]=0x00;//control reg data outpu
	data[1]=0x1C;//data outpu 220Hz
	I2C_WRITE(LSM303DLHC,data,2);
	delay();
	data[0]=0x01;//control reg m
	data[1]=0x20;//+- 1.3 gauss
	I2C_WRITE(LSM303DLHC,data,2);
	delay();
	data[0]=0x02;
	data[1]=0x00;//continous mode
	I2C_WRITE(LSM303DLHC,data,2);
	delay();
	(*mems).konwersja=M1;
	(*mems).zakres=2;
	
}

//odczyt danych z akcelerometru
void LSM303DLHC_READ_AXIS(MEMS *mems,uint8_t TYP){
	uint8_t data[6];
	uint8_t data0 = 0;
	I2C_READ(LSM303DLHC,TYP,data,6);
	(*mems).x=((data[1]<<8)|data[0]);
	(*mems).y=((data[3]<<8)|data[2]);
	(*mems).z=((data[5]<<8)|data[4]);
	
}
