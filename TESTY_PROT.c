#include <stm32f4xx.h>
#include<stdio.h>
#include"TESTY_PROT.h"
#include"defines.h"


char buf[MAXBUF];
uint8_t tab_byte_licznik=0;
uint8_t wybor_kanalu;
uint8_t stream_licznik;
uint8_t stan=0;


uint8_t tab_length(char *t){
	uint8_t i=0;
	for(i=0;i<MAXBUF;i++){
		if(t[i]==0)break;
	}
	return i;
}

//********make stream
void make_stream(char *s,uint8_t licznik_data,int16_t *d){
	char a='a';
	uint8_t length,licznik;
	licznik=0;
	for(uint8_t i=0;i<licznik_data;i++){
		//s[licznik++]=a++;
		s[licznik++]=',';
		sprintf(buf,"%i",d[i]);
		length=tab_length(buf);
		for(uint8_t z=0;z<length;z++){
			s[licznik++]=buf[z];
		}
	}
	if(REAL_TERM_ON){
	s[licznik++]=13;
	s[licznik++]=10;
	}
	s[licznik]=0;
	
}


void send_stream(void){
	
	
	tab_byte_licznik=0;
	stream_licznik=0;
	//oczekiwanie az bufor nadawania bedzie pusty
	
	//while(!((USART6->SR)&(USART_SR_TXE)));
	//odblokowanie przerwania od pustego bufora wysyalnia danych
	USART6->CR1 |=USART_CR1_TXEIE;
	
	
	while(stan==0);
	stan=0;
	
}
