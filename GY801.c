#include <stm32f4xx.h>
#include"I2C.h"
#include"defines.h"
#include"GY801.h"
#include"math.h"


void L3GD40_ENABLE(MEMS *mems){
	uint8_t data[2];
	
	data[0]=0x20;//control reg 1
	data[1]=0x4F;//odblokowanie osi + normal mode +200Hz ODR + cut-off 12Hz
	I2C3_WRITE(L3GD40,data,2);
	delay();
	data[0]=0x21;//control reg 2
	data[1]=0x20;//normal mode 15Hz cut off
	I2C3_WRITE(L3GD40,data,2);
	delay();
	data[0]=0x24;//control reg 5
	data[1]=0x12;//odblokowanie filtru gornoprzepustowego oraz uruchomienie filtrow gorno i folno przepustowych
	I2C3_WRITE(L3GD40,data,2);
	delay();
	
	(*mems).zakres=250;
	(*mems).konwersja=Z250;
	(*mems).dryft=10;
	
}

void L3GD40_READ_AXIS(MEMS *mems){
	uint8_t data[6];
	I2C3_READ(L3GD40,L3GD40_AXIS,data,6);
	(*mems).x=((data[1]<<8)|data[0]);
	(*mems).y=((data[3]<<8)|data[2]);
	(*mems).z=((data[5]<<8)|data[4]);
	
}

void ADXL345_ENABLE(MEMS *mems){
	//rate code 0x0B
	//addr 0x2C
	uint8_t data[2];
	data[0]=0x27;
	data[1]=0x00;
	I2C3_WRITE(ADXL345_W,data,2);
	delay();
	data[0]=0x2D;//power ctl
	data[1]=0x08;
	I2C3_WRITE(ADXL345_W,data,2);
	delay();
	data[0]=0x38;
	data[1]=0x00;
	I2C3_WRITE(ADXL345_W,data,2);
	delay();
	data[0]=0x31;
	data[1]=0x08;
	I2C3_WRITE(ADXL345_W,data,2);
	
	//0x38
	//0x00bypass fifo
	(*mems).konwersja = A2;
	(*mems).zakres=2;
	
	
}

void ADXL345_READ_AXIS(MEMS *mems){
	
	uint8_t data[6];
	I2C3_READ(ADXL345_W,ADXL345_AXIS,data,6);
	(*mems).x=((data[0]<<8)|data[1]);
	(*mems).y=((data[2]<<8)|data[3]);
	(*mems).z=((data[4]<<8)|data[5]);
	
}

void HMC5883L_ENABLE(MEMS *mems){
	uint8_t data[2];
	//1.3Gauss + -
	data[0]=0x00;
	data[1]=0x18;//75Hz odr
	I2C3_WRITE(HMC5883L_W,data,2);
	delay();
	data[0]=0x02;
	data[1]=0x00;//continous mode
	I2C3_WRITE(HMC5883L_W,data,2);
	delay();
	data[0]=0x01;
	data[1]=0xE0;//+- 8.1 Gaussa
	I2C3_WRITE(HMC5883L_W,data,2);
	delay();
	(*mems).zakres=81; 
	(*mems).konwersja=G81;
	
}

void HMC5883L_READ_AXIS(MEMS *mems){
	uint8_t data[6];
	I2C3_READ(HMC5883L_W,0x03,data,6);
		(*mems).x=((data[1]<<8)|data[0]);
	(*mems).y=((data[3]<<8)|data[2]);
	(*mems).z=((data[5]<<8)|data[4]);
}


void BMP180_ENABLE(BAROMETR *baro){
	uint8_t data[22];
	I2C3_READ(BMP180_W,0xAA,data,22);
	(*baro).AC1=((data[0]<<8)|data[1]);
	(*baro).AC2= ((data[2]<<8)|data[3]);
	(*baro).AC3= ((data[4]<<8)|data[5]);
	(*baro).AC4= ((data[6]<<8)|data[7]);
	(*baro).AC5= ((data[8]<<8)|data[9]);
	(*baro).AC6= ((data[10]<<8)|data[11]);
	(*baro).B1= ((data[12]<<8)|data[13]);
	(*baro).B2= ((data[14]<<8)|data[15]);
	(*baro).MB= ((data[16]<<8)|data[17]);
	(*baro).MC= ((data[18]<<8)|data[19]);
	(*baro).MD= ((data[20]<<8)|data[21]);
	
}

void BMP180_READ(BAROMETR *baro){
	uint8_t data[3];
	data[0]=0xF4;
	data[1]=0x2E;
	I2C3_WRITE(BMP180_W,data,2);
	I2C3_READ(BMP180_W,0xF6,data,2);
	(*baro).t_reg= ((data[0]<<8)|data[1]);
	data[0]=0xF4;
	data[1]=0x32+(3<<6);
	I2C3_WRITE(BMP180_W,data,2);
	I2C3_READ(BMP180_W,0xF6,data,3);
	(*baro).p_reg=(((data[0]<<16)|(data[1]<<8)|data[2])>>5);
	
}

void BMP180_CALC(BAROMETR *baro){
		//temperatura
	(*baro).X1=((*baro).t_reg-(*baro).AC6)*(*baro).AC5*ROZDZIELCZOSC_15;
	(*baro).X2=((*baro).MC*DWADO11/((*baro).X1+(*baro).MD));
	(*baro).B5=(*baro).X1+(*baro).X2;
	(*baro).temperature=((*baro).B5+8)*TEMPERA;
		//cisnienie
	(*baro).B6=(*baro).B5-4000;
	(*baro).X1=((*baro).B2*((*baro).B6*(*baro).B6*ROZDZIELCZOSC_12))*ROZDZIELCZOSC_11;
	(*baro).X2=((*baro).AC2*(*baro).B6)*ROZDZIELCZOSC_11;
	(*baro).X3=(*baro).X1+(*baro).X2;
	(*baro).B3=((((*baro).AC1*4+(*baro).X3)<<3)+2)/4;
	(*baro).X1=((*baro).AC3*(*baro).B6)*ROZDZIELCZOSC_13;
	(*baro).X2= ((*baro).B1*((*baro).B6*(*baro).B6*ROZDZIELCZOSC_12))*ROZDZIELCZOSC_15;
	(*baro).X3=(((*baro).X1+(*baro).X2)+2)*ROZDZIELCZOSC_2;
	(*baro).B4=((*baro).AC4*(unsigned long)((*baro).X3+32768))*ROZDZIELCZOSC_15;
	(*baro).B7=((unsigned long)(*baro).p_reg-(*baro).B3)*(50000>>3);
	if((*baro).B7<0x80000000){
	(*baro).pressure=((*baro).B7*2)/(*baro).B4;
	}else (*baro).pressure= ((*baro).B7/(*baro).B4)*2;
	(*baro).X1=((*baro).pressure*ROZDZIELCZOSC_8)*((*baro).pressure*ROZDZIELCZOSC_8);
	(*baro).X1=((*baro).X1*3038)*ROZDZIELCZOSC_16;
	(*baro).X2=(-7357*(*baro).pressure)*ROZDZIELCZOSC_16;
	(*baro).pressure=(*baro).pressure+((*baro).X1+(*baro).X2+3791)*ROZDZIELCZOSC_4;
	
	//wysokosc
	double tmp=(double)((*baro).pressure/(*baro).P0);
	(*baro).wysokosc=(double)44330.0*(1-pow(tmp,0.1903));

}

