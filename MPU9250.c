#include <stm32f4xx.h>
#include"defines.h"
#include"MPU9250.h"

#include"SPI.h"

void MPU9250_ACC_ENABLE(MEMS *mems){
	MPU_CS_LOW;
	delay();
	spi4_transfer_byte(0x1D);//ACC CONF 2 DLPF
	spi4_transfer_byte(0x04);
	delay();
	MPU_CS_HIGH;
	
//	MPU_CS_LOW;
//	delay();
//	spi4_transfer_byte(108);//power moder
	//spi4_transfer_byte(0x00); // odblokwoanie
//	delay();
//	MPU_CS_HIGH;

	(*mems).zakres=2;
	(*mems).konwersja=A2;
	
}

void MPU9250_ACC_READ(MEMS *mems){
	uint8_t i;
	uint8_t data[6];
		MPU_CS_LOW;
	spi4_transfer_byte(0xBB);
	for(i=0;i<6;i++){
		data[i]=spi4_transfer_byte(0xFF);
	}
	
	(*mems).x= ((data[1]<<8)|data[0]);
	(*mems).y= ((data[3]<<8)|data[2]);
	(*mems).z=	((data[5]<<8)|data[4]);
	
	
	MPU_CS_HIGH;
}




