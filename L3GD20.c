#include <stm32f4xx.h>
#include"defines.h"
#include"L3GD20.h"
#include"SPI.h"

//
void L3GD20_gyro_enable(MEMS *mems){
CS_LOW;
	spi1_transfer_byte(0x20);
	spi1_transfer_byte(0x7F);//0x0F odblokownaie osi
	//delay();
	CS_HIGH;
	//delay();
	CS_LOW;
	//ustawianie filtru gorno-przepostowego
	spi1_transfer_byte(0x21);
	spi1_transfer_byte(0x20);//nor
	//delay();
	CS_HIGH;
	//delay();
	CS_LOW;
	//ustawienie pelnej skali
	spi1_transfer_byte(0x23);
	spi1_transfer_byte(0x00);//
	//delay();
CS_HIGH;
		CS_LOW;
	//ustawienie pelnej skali
	spi1_transfer_byte(0x24);
	spi1_transfer_byte(0x12);//
	//delay();
CS_HIGH;
	(*mems).konwersja=Z250;
	(*mems).zakres=250;
}
//
void L3GD20_READ_AXIS(MEMS *gyro){
	uint8_t i;
	uint8_t data[6];
		CS_LOW;
	spi1_transfer_byte(0xC0|L3GD20_READ);
	for(i=0;i<6;i++){
	data[i]=spi1_transfer_byte(0xFF);
	}	
	
	(*gyro).x= ((data[1]<<8)|data[0]);
	(*gyro).y= ((data[3]<<8)|data[2]);
	(*gyro).z=	((data[5]<<8)|data[4]);
	
}

