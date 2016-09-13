


#include <stm32f4xx.h>
#include"USART.h"
#include"Komunikacja.h"

void KomunikacjaStructInit(RAMKA *f){
(*f).ADDR_ID=ADDR;
(*f).length=FRAME_LENGTH;
(*f).KONTROLA=CONTROL;
(*f).index=0;
	
}

void ReadIntFromFrame(int16_t *data,RAMKA *f){
	for(int i=0;i<6;i++){
		data[i]=(((*f).frame[2*i+2])|((*f).frame[2*i+3]<<8));
	}
}

void WriteFloatToFrame(float *data,RAMKA *f){
		uint8_t *x;
		for(int i=0;i<3;i++){
			x=(uint8_t*)(&data[i]);
			for(int z=0;z<4;z++)(*f).frame[4*i+2+z]=x[z];
		}
	
	
}

void ReadFloatFromFrame(float *data,RAMKA *f){
	KonwersFloat KonwersjaFloat;
	for(uint8_t i=0;i<3;i++){
		KonwersjaFloat.b[0]=(*f).frame[2+i*4];
		KonwersjaFloat.b[1]=(*f).frame[3+i*4];
		KonwersjaFloat.b[2]=(*f).frame[4+i*4];
		KonwersjaFloat.b[3]=(*f).frame[5+i*4];
		data[i]=KonwersjaFloat.f;
	}
	
	
}

void WriteInt16ToFrame(uint16_t *data,RAMKA *f){
	uint16_t tmp;
	uint8_t *x;
	for(int i=0;i<6;i++){
		x=(uint8_t*)(&data[i]);
		//(*f).frame[2*i+2]=(tmp&0xFF);
		//(*f).frame[2*i+3]=(data[i]>>8);
		for(int z=0;z<2;z++)(*f).frame[2*i+2+z]=x[z];
		
	}
}
