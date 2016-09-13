#include <stm32f4xx.h>

#include"defines.h"
#include"LSM303DLHC.h"
#include"L3GD20.h"
#include"MPU9250.h"
#include"AKCELEROMETR.h"
#include"GPIO_TIM_CLOCK.h"
#include"TESTY_PROT.h"
#include"USART.h"
#include"filtry.h"
#include"Komunikacja.h"
#include"Sterowanie.h"
#include"SPI.h"
#include"GY801.h"

#define TRYB_SUROWY 0

int y_p=0;
int y_u=0;
int start=0;
int tryb=0;
int nowe_bufory=0;
int licznik_bufory_nowe=0;

ORIENTACJA_RATE OrientacjaRate;


FILTRY_CONF FiltryKonfiguracja;
MEMS L3GD20,LSM303DLHC_ACC,LSM303DLHC_MAGNET;
MEMS MPU9250_ACC;
MEMS L3GD40_GYRO,ADXL345_ACC,HMC_MAGNET;
BAROMETR BMP180;
ORIENTACJA_ACC Dane_Akcelerometr,DanePoSredniej;
FILTR_KOMPLEMENTARNY FILTR_DATA;
RAMKA RamkaOdczyt,RamkaZapis;

PWM_CONF PWMdata;
STEROWANIE Sterowanie;

SYGNALY Sygnaly;

SILNIK silnik1,silnik2,silnik3,silnik4;

BUFOR_FILO ACC_BUF_X,ACC_BUF_Y,ACC_BUF_Z,GYRO_BUF_X,GYRO_BUF_Y,GYRO_BUF_Z;
BUFOR_FILO L3GD40_X,L3GD40_Y,L3GD40_Z;
BUFOR_FILO ADXL345_X,ADXL345_Y,ADXL345_Z;
BUFOR_FILO HMC_X,HMC_Y,HMC_Z;
BUFOR_FILO ALFA,BETA,GAMMA;


void initBuffors(void);

void HARDWARE_INIT(void);
void STREAM_TAB_UPDATE(uint8_t x);


volatile uint8_t czas=0;

//tryb testowy surowe dane
#define MAXSTREAM 60
#define MAXDATA 6

 char stream[MAXSTREAM];
volatile int16_t stream_tab[MAXDATA];
uint8_t licznik_stream=0;
uint8_t recived_data=0;
//tryb testowy surowe dane


void ObslugaKomunikacji(void);
void OdczytDanych(void);
void ZapisDanych(void);
void WyslijRamke(RAMKA *f);

uint8_t WHO[1];

int main(){
	
	InitSilnik(&silnik1,ID_SILNIK_1);
	InitSilnik(&silnik2,ID_SILNIK_2);
	InitSilnik(&silnik3,ID_SILNIK_3);
	InitSilnik(&silnik4,ID_SILNIK_4);

	initFiltryConf(&FiltryKonfiguracja);
	
	KomunikacjaStructInit(&RamkaOdczyt);
	KomunikacjaStructInit(&RamkaZapis);
	
	FILTR_DATA.b1=FiltryKonfiguracja.b1;		//PARAMETR FILTRU KOMPLEMENTARNEGO
	FILTR_DATA.b2=FiltryKonfiguracja.b2;
	Dane_Akcelerometr.prog_dolny=FiltryKonfiguracja.prog_dolny_acc;
	Dane_Akcelerometr.prog_gorny=FiltryKonfiguracja.prog_gorny_acc;
	
	HARDWARE_INIT();
	
		InitPWM();
	
	initBuffors();
	//inicjalizacja wstepna czujnikow i ich struktur
	LSM303DLHC_acc_enable(&LSM303DLHC_ACC);
	L3GD20_gyro_enable(&L3GD20);
	L3GD40_ENABLE(&L3GD40_GYRO);
	ADXL345_ENABLE(&ADXL345_ACC);
	HMC5883L_ENABLE(&HMC_MAGNET);
	
	BMP180_ENABLE(&BMP180);
	BMP180.P0=10280;
	//inicjalizacja kalmana
		InitStructPWM(&PWMdata);
		setPWM(&PWMdata);
		InitStructSterowanie(&Sterowanie);
	
	int16_t licznik_wyslanych_danych=0;

	uint8_t costam=0;
	
	while(1){
		if(czas){
			
			//odczyt danych
			L3GD20_READ_AXIS(&L3GD20);
			LSM303DLHC_READ_AXIS(&LSM303DLHC_ACC,AKCELEROMETR);
	
			L3GD40_READ_AXIS(&L3GD40_GYRO);
			ADXL345_READ_AXIS(&ADXL345_ACC);
			HMC5883L_READ_AXIS(&HMC_MAGNET);
		  BMP180_READ(&BMP180);
			BMP180_CALC(&BMP180);
			//PD6_SW;
			//skalowanie danych
			Skalowanie_Danych_Mems(&L3GD20);
			Skalowanie_Danych_Mems(&LSM303DLHC_ACC);
			Skalowanie_Danych_Mems(&L3GD40_GYRO);
			Skalowanie_Danych_Mems(&ADXL345_ACC);
			Skalowanie_Danych_Mems(&HMC_MAGNET);
		//	Skalowanie_Danych_Mems(&LSM303DLHC_MAGNET);
			
			
			
			//Filtracja srednie kroczace 
			LSM303DLHC_ACC.xks10=SredniaKroczaca(&ACC_BUF_X,LSM303DLHC_ACC.xk);
			LSM303DLHC_ACC.yks10=SredniaKroczaca(&ACC_BUF_Y,LSM303DLHC_ACC.yk);
			LSM303DLHC_ACC.zks10=SredniaKroczaca(&ACC_BUF_Z,LSM303DLHC_ACC.zk);
			
			//zyroskop
				L3GD20.xks10=SredniaKroczaca(&GYRO_BUF_X,L3GD20.xk);
			L3GD20.yks10=SredniaKroczaca(&GYRO_BUF_Y,L3GD20.yk);
			L3GD20.zks10=SredniaKroczaca(&GYRO_BUF_Z,L3GD20.zk);
			//l3gd40
			L3GD40_GYRO.xks10=SredniaKroczaca(&L3GD40_X,L3GD40_GYRO.xk);
			L3GD40_GYRO.yks10=SredniaKroczaca(&L3GD40_Y,L3GD40_GYRO.yk);
			L3GD40_GYRO.zks10=SredniaKroczaca(&L3GD40_Z,L3GD40_GYRO.zk);
			//ADXL345
			ADXL345_ACC.xks10=SredniaKroczaca(&ADXL345_X,ADXL345_ACC.xk);
			ADXL345_ACC.yks10=SredniaKroczaca(&ADXL345_Y,ADXL345_ACC.yk);
			ADXL345_ACC.zks10=SredniaKroczaca(&ADXL345_Z,ADXL345_ACC.zk);
			//HMC5883L
			HMC_MAGNET.xks10=SredniaKroczaca(&HMC_X,HMC_MAGNET.xk);
			HMC_MAGNET.yks10=SredniaKroczaca(&HMC_Y,HMC_MAGNET.yk);
			HMC_MAGNET.zks10=SredniaKroczaca(&HMC_Z,HMC_MAGNET.zk);
			
			//Odejmowanie dryftu zyroskop
			OdejmowanieDryftu(&L3GD40_GYRO);
			OdejmowanieDryftu(&L3GD20);
			
			
			if(nowe_bufory==1){
			Obliczanie_Orientacji(&LSM303DLHC_ACC,LSM303DLHC_ACC.xks10,LSM303DLHC_ACC.yks10,LSM303DLHC_ACC.zks10,&Dane_Akcelerometr);
			}else {
					licznik_bufory_nowe++;
				if(licznik_bufory_nowe==100)nowe_bufory=1;
			}
			
			if(Dane_Akcelerometr.stan_dynamiczny){
				FILTR_DATA.b=FILTR_DATA.b2;
				FILTR_DATA.dt=(1.0/250.0);
			}else {
				FILTR_DATA.b=FILTR_DATA.b1;
				FILTR_DATA.dt=0;
			}
		
			
		FILTR_DATA.alfa=FiltrKomplementarny(&FILTR_DATA,L3GD40_GYRO.xks10,Dane_Akcelerometr.beta,FILTR_DATA.alfa);
		FILTR_DATA.beta=FiltrKomplementarny(&FILTR_DATA,(-1)*L3GD40_GYRO.yks10,Dane_Akcelerometr.alfa,FILTR_DATA.beta);
		FILTR_DATA.gamma=FiltrKomplementarny(&FILTR_DATA,L3GD40_GYRO.zks10,Dane_Akcelerometr.gamma,FILTR_DATA.gamma);
		
			CalcOrientacjaRate(&OrientacjaRate,FILTR_DATA.alfa,FILTR_DATA.beta,FILTR_DATA.gamma);
			
		DanePoSredniej.alfa=SredniaKroczaca(&ALFA,FILTR_DATA.alfa);
		DanePoSredniej.beta=SredniaKroczaca(&BETA,FILTR_DATA.beta);
		DanePoSredniej.gamma=SredniaKroczaca(&GAMMA,FILTR_DATA.gamma);
		
			FILTR_DATA.alfa=DanePoSredniej.alfa;
			FILTR_DATA.beta=DanePoSredniej.beta;
			FILTR_DATA.gamma=DanePoSredniej.gamma;
			
			
			
			Sygnaly.alfa_rate_pomiar=OrientacjaRate.AlfaRate;
			Sygnaly.beta_rate_pomiar=OrientacjaRate.BetaRate;
			
		Uchyb(&Sygnaly,FILTR_DATA.alfa,FILTR_DATA.beta-90);
		
		if(tryb==1){
			
			GPIOD->ODR |=LED_RED;
			GPIOD->ODR &=(~LED_GREEN);
		
	//2 regulatory PID
			
			RegSilnik2(&silnik1,&Sygnaly);
			RegSilnik2(&silnik2,&Sygnaly);
			RegSilnik2(&silnik3,&Sygnaly);
			RegSilnik2(&silnik4,&Sygnaly);
			
	//	RegSilnik(&silnik1,silnik1.wysokosc.stalaH,(-1)*Sygnaly.alfa_uchyb,(-1)*Sygnaly.beta_uchyb);
	//	RegSilnik(&silnik2,silnik2.wysokosc.stalaH,Sygnaly.alfa_uchyb,(-1)*Sygnaly.beta_uchyb);
	//	RegSilnik(&silnik3,silnik3.wysokosc.stalaH,(-1)*Sygnaly.alfa_uchyb,Sygnaly.beta_uchyb);
	//	RegSilnik(&silnik4,silnik4.wysokosc.stalaH,Sygnaly.alfa_uchyb,Sygnaly.beta_uchyb);
			
			//bez ukladu roznicowego
	//		RegSilnik(&silnik1,silnik1.wysokosc.stalaH,0,0);
//		RegSilnik(&silnik2,silnik2.wysokosc.stalaH,Sygnaly.alfa_uchyb,0);
	//	RegSilnik(&silnik3,silnik3.wysokosc.stalaH,0,Sygnaly.beta_uchyb);
	//	RegSilnik(&silnik4,silnik4.wysokosc.stalaH,Sygnaly.alfa_uchyb,Sygnaly.beta_uchyb);
			
		PWMdata.PWM_S1=silnik1.PWM;
		PWMdata.PWM_S2=silnik2.PWM;
		PWMdata.PWM_S3=silnik3.PWM;
		PWMdata.PWM_S4=silnik4.PWM;
		
		setPWM(&PWMdata);
		}else{
			
		GPIOD->ODR&=(~LED_RED);
		GPIOD->ODR|=LED_GREEN;
		
		}
		
			ObslugaKomunikacji();
		if(TRYB_SUROWY==1){
		if(recived_data==0){
			
				PWMdata.PWM_S1=0;
				PWMdata.PWM_S2=0;
				PWMdata.PWM_S3=0;
				PWMdata.PWM_S4=0;
			
		//	setPWM(&PWMdata);
		}else{
			PWMdata.PWM_S1=recived_data*10;
			PWMdata.PWM_S2=recived_data*10;
			PWMdata.PWM_S3=recived_data*10;
			PWMdata.PWM_S4=recived_data*10;
			
			setPWM(&PWMdata);
			
			y_p++;
			int16_t tab[6];
			tab[0]=LSM303DLHC_ACC.x;
			tab[1]=LSM303DLHC_ACC.y;
			tab[2]=LSM303DLHC_ACC.z;
			tab[3]=L3GD20.x;
			tab[4]=L3GD20.y;
			tab[5]=L3GD20.z;
			
			make_stream(stream,6,tab);
			send_stream();
			
			GPIOD->ODR ^=LED_BLUE;
			
	
		}
		
	}
		
			

	}
		
	}
}
//********************************************************************************************************//
void HARDWARE_INIT(void){
	
	clockConfig();
SystemCoreClockUpdate();
	//inicjalizacja hardware
	GPIO_LED_INIT();
	usart6_init();
	//usart2_init();
	I2C1_INIT();
	I2C3_INIT();
	SPI1_INIT();
	SPI4_INIT();
	
	
	GPIOD_TIM_INIT();
	TIM4_INIT_500HZ();
	
	
}
//
void initBuffors(void){
	
	FILTR_DATA.dt=(1.0/250.0);
	//FILTR_DATA.dt=0;
	
	initBuforFilo(&ACC_BUF_X,FiltryKonfiguracja.srednia_acc);
	initBuforFilo(&ACC_BUF_Y,FiltryKonfiguracja.srednia_acc);
	initBuforFilo(&ACC_BUF_Z,FiltryKonfiguracja.srednia_acc);
	
	initBuforFilo(&GYRO_BUF_X,FiltryKonfiguracja.srednia_gyro);
	initBuforFilo(&GYRO_BUF_Y,FiltryKonfiguracja.srednia_gyro);
	initBuforFilo(&GYRO_BUF_Z,FiltryKonfiguracja.srednia_gyro);
	
	initBuforFilo(&L3GD40_X,FiltryKonfiguracja.srednia_gyro);
	initBuforFilo(&L3GD40_Y,FiltryKonfiguracja.srednia_gyro);
	initBuforFilo(&L3GD40_Z,FiltryKonfiguracja.srednia_gyro);
	
	initBuforFilo(&ADXL345_X,FiltryKonfiguracja.srednia_acc);
	initBuforFilo(&ADXL345_Y,FiltryKonfiguracja.srednia_acc);
	initBuforFilo(&ADXL345_Z,FiltryKonfiguracja.srednia_acc);
	
	initBuforFilo(&HMC_X,FiltryKonfiguracja.srednia_magnet);
	initBuforFilo(&HMC_Y,FiltryKonfiguracja.srednia_magnet);
	initBuforFilo(&HMC_Z,FiltryKonfiguracja.srednia_magnet);
	
	initBuforFilo(&ALFA,20);
	initBuforFilo(&BETA,20);
  initBuforFilo(&GAMMA,20);
	
	initOrientacjaRate(&OrientacjaRate,0.004);
	
}

//
//**************************************************************************************************//
//*********************************             PRZERWANIA          ********************************//
//**************************************************************************************************//


//*********************************								I2C								********************************//
void I2C1_EV_IRQHandler(void){
	//WYGENEROWANIE START BITU
	if(I2C1->SR1 & I2C_SR1_SB){
		
	}
	//wyslano adres
	if(I2C1->SR1&I2C_SR1_ADDR){
		
	}
	
	//DATA REGISTER EMPTY
	if(I2C1->SR1 & I2C_SR1_TXE){
		
	}
	//RECIVER REGISTER NOT EMPTY
	if(I2C1->SR1 & I2C_SR1_RXNE){
	
	}
	//BYTE TRANSFER FINISHED
	if(I2C1->SR1 & I2C_SR1_BTF){
		
	}
	

}
void I2C1_ER_IRQHandler(void){

	//ack fail
	if(I2C1->SR1&I2C_SR1_AF){
		//ZGASIC SOFTWAROWO
	}
	//GPIOD->ODR |=LED_ORANGE;
}


//********************************					PRZERWANIA TIMER				*********************************//

void TIM4_IRQHandler(void){
	czas=1;
	TIM4->SR &=(~TIM_SR_UIF);
	TIM4->CNT =0;//250HZ;

}	



//*********************************								USART              ********************************//
void USART2_IRQHandler(void){
	//USART_SR_TXE udr przeniesiony do shift register, 
	//mozna wstawic kolejna dana do usart2->dr, mozna wyslac kolejna dana
	
//	if((USART2->SR)&(USART_SR_TXE));
		

//	if((USART2->SR)&(USART_SR_RXNE));
		
	
}

void USART6_IRQHandler(void){
	//USART_SR_TXE udr przeniesiony do shift register, 
	//mozna wstawic kolejna dana do usart6->dr, mozna wyslac kolejna dana
	if(TRYB_SUROWY==0){
	if((USART6->SR)&(USART_SR_TXE)){
		
		//do wysylania strumienia danych
		//wylaczenie przerwania od pustego bufora danych
	//	PD6_SW;
		
		if(RamkaZapis.index<16){
			USART6->DR = RamkaZapis.frame[RamkaZapis.index];
			RamkaZapis.index++;
			
		} 
		else {
			
			USART6->CR1 &=~USART_CR1_TXEIE;
		//	PD6_SW;
		}
			
	
	}
	//USART_SR_RXNE dana gotowa do odczytania z USART6->DR
	if((USART6->SR)&(USART_SR_RXNE)){
	
		
		RamkaOdczyt.frame[RamkaOdczyt.index++]=USART6->DR;
		if(RamkaOdczyt.index==16){
		RamkaOdczyt.index=0;
		RamkaOdczyt.ADDR_ID=RamkaOdczyt.frame[0];
			RamkaOdczyt.CMD=RamkaOdczyt.frame[1];
			RamkaOdczyt.KONTROLA=RamkaOdczyt.frame[14];
		if(RamkaOdczyt.frame[0]==RamkaOdczyt.frame[15]){
					RamkaOdczyt.FRAME_OK=1;
		}else RamkaOdczyt.FRAME_OK=0;
		
	}
	
	
}
	
}  else if(TRYB_SUROWY==1){
	if((USART6->SR)&(USART_SR_RXNE)){
		
				recived_data =USART6->DR;
			
	}
	if((USART6->SR)&(USART_SR_TXE)){
			
		if(stream[stream_licznik]!=0){
		  USART6->DR = stream[stream_licznik];
			y_u++;
			stream_licznik++;
		}else{
			stan=1;
			USART6->CR1 &=~USART_CR1_TXEIE;
		}

	}
	

}

}

/////////////////////////////////////////////////////////////////////////////////////////////////////

void ObslugaKomunikacji(void){
	//sprawdzic operacja bitowa 
	
	ZapisDanych();
	OdczytDanych();
	
	
}


void OdczytDanych(void){
	float tmp[3];
	uint16_t tmpi[6];
	int16_t tmpui[6];
	switch(RamkaOdczyt.CMD){
		case 0x01: // odczyt alfa beta gamma
			GPIOD->ODR ^= LED_ORANGE;
			tmp[0]=(float)FILTR_DATA.alfa;
			tmp[1]=(float)FILTR_DATA.beta;
			tmp[2]=(float)FILTR_DATA.gamma;
			WriteFloatToFrame(tmp,&RamkaZapis);
			RamkaZapis.frame[1]=0x01;
			RamkaOdczyt.CMD=0;
			WyslijRamke(&RamkaZapis);
			break;
		case 0x02://odczyt akcelerometr x y z
	
			tmp[0]=(float)LSM303DLHC_ACC.xks10;
			tmp[1]=(float)LSM303DLHC_ACC.yks10;
			tmp[2]=(float)LSM303DLHC_ACC.zks10;
			WriteFloatToFrame(tmp,&RamkaZapis);
			RamkaZapis.frame[1]=0x02;
			
		RamkaOdczyt.CMD=0;
		WyslijRamke(&RamkaZapis);
		break;
		case 0x03://odczyt akcelerometr wypadkowy
	
			tmp[0]=(float)LSM303DLHC_ACC.wypadkowy;
			tmp[1]=0;
		tmp[2]=0;
		WriteFloatToFrame(tmp,&RamkaZapis);
		RamkaZapis.frame[1]=0x03;
		WyslijRamke(&RamkaZapis);
			
		RamkaOdczyt.CMD=0;
		break;
		case 0x04://akcelerometr prog dolny
			
			tmp[0]=(float)Dane_Akcelerometr.prog_dolny;
			tmp[1]=0;
			tmp[2]=0;
		WriteFloatToFrame(tmp,&RamkaZapis);
		RamkaZapis.frame[1]=0x04;
		WyslijRamke(&RamkaZapis);
		RamkaOdczyt.CMD=0;
		break;
		case 0x05://akcelerometr prog gorny
				tmp[0]=(float)Dane_Akcelerometr.prog_gorny;
			tmp[1]=0;
			tmp[2]=0;
		WriteFloatToFrame(tmp,&RamkaZapis);
		RamkaZapis.frame[1]=0x05;
		WyslijRamke(&RamkaZapis);
		RamkaOdczyt.CMD=0;
		break;
		case 0x06://wczytywanie sterowania
			tmp[0]=PWMdata.MAX_PWM;
			tmp[1]=PWMdata.PWM_TOLERANCJA;
			tmp[2]=Sterowanie.max_wychylenie;
			WriteFloatToFrame(tmp,&RamkaZapis);
		RamkaZapis.frame[1]=0x06;
		
		RamkaOdczyt.CMD=0;
		WyslijRamke(&RamkaZapis);
		break;
		
		case 0x07://pobieranie danych pwm
	
		
			getPWM(&PWMdata);
		tmpi[0]=PWMdata.PWM_S1;
		tmpi[1]=PWMdata.PWM_S2;
		tmpi[2]=PWMdata.PWM_S3;
		tmpi[3]=PWMdata.PWM_S4;
		tmpi[4]=PWMdata.MAX_PWM;
		tmpi[5]=PWMdata.PWM_TOLERANCJA;
		WriteInt16ToFrame(tmpi,&RamkaZapis);
		RamkaZapis.frame[1]=0x07;
		
		RamkaOdczyt.CMD=0;
		WyslijRamke(&RamkaZapis);
		break;
		
		case 0x08://pobieranie uchybu
			tmp[0]=Sygnaly.alfa_uchyb;
			tmp[1]=Sygnaly.beta_uchyb;
		  tmp[2]=0;
		WriteFloatToFrame(tmp,&RamkaZapis);
		RamkaZapis.frame[1]=0x08;
		RamkaOdczyt.CMD=0;
		WyslijRamke(&RamkaZapis);
		break;
		
		case 0x09://dane silnik 1
				tmp[0]=silnik1.wysokosc.wys;
				tmp[1]=silnik1.reg_alfa.Wyjscie;
				tmp[2]=silnik1.reg_beta.Wyjscie;
		WriteFloatToFrame(tmp,&RamkaZapis);
		RamkaZapis.frame[1]=0x09;
		RamkaOdczyt.CMD=0;
		WyslijRamke(&RamkaZapis);
		
		break;
		
		case 0x0A://dane silnik 2
			tmp[0]=silnik2.wysokosc.wys;
			tmp[1]=silnik2.reg_alfa.Wyjscie;
			tmp[2]=silnik2.reg_beta.Wyjscie;
		WriteFloatToFrame(tmp,&RamkaZapis);
		RamkaZapis.frame[1]=0x0A;
		RamkaOdczyt.CMD=0;
		WyslijRamke(&RamkaZapis);
		
		break;
		
		case 0x0B://dane silnik 3
				tmp[0]=silnik3.wysokosc.wys;
				tmp[1]=silnik3.reg_alfa.Wyjscie;
				tmp[2]=silnik3.reg_beta.Wyjscie;
		WriteFloatToFrame(tmp,&RamkaZapis);
		RamkaZapis.frame[1]=0x0B;
		RamkaOdczyt.CMD=0;
		WyslijRamke(&RamkaZapis);
		
		break;
		
		case 0x0C://dane silnik 4
			tmp[0]=silnik4.wysokosc.wys;
			tmp[1]=silnik4.reg_alfa.Wyjscie;
			tmp[2]=silnik4.reg_beta.Wyjscie;
		WriteFloatToFrame(tmp,&RamkaZapis);
		RamkaZapis.frame[1]=0x0C;
		RamkaOdczyt.CMD=0;
		WyslijRamke(&RamkaZapis);
		
		break;
		
		case 0x0D:  //STOP AWARYJNY
			tryb=0;
		
		PWMdata.PWM_S1=0;
		PWMdata.PWM_S2=0;
		PWMdata.PWM_S3=0;
		PWMdata.PWM_S4=0;
		
		setPWM(&PWMdata);
		S1_PWM = 0;
		S2_PWM=0;
		S3_PWM=0;
		S4_PWM=0;
		
		ZerowanieParametrow(&silnik1);
		ZerowanieParametrow(&silnik2);
		ZerowanieParametrow(&silnik3);
		ZerowanieParametrow(&silnik4);
		
		RamkaZapis.frame[1]=0x0D;
		RamkaOdczyt.CMD=0;
		WyslijRamke(&RamkaZapis);
		break;
		
		case 0x0E:  //odczyt konfiguracja filtry 1 srednie kroczace i progi
		tmpi[0]=FiltryKonfiguracja.srednia_acc;	
		tmpi[1]=FiltryKonfiguracja.srednia_gyro;
		tmpi[2]=FiltryKonfiguracja.srednia_magnet;
		tmpi[3]=FiltryKonfiguracja.srednia_baro;
		tmpi[4]=(FiltryKonfiguracja.b1*100.0);
		tmpi[5]=(FiltryKonfiguracja.b2*100.0);
		tmpi[6]=0;
		WriteInt16ToFrame(tmpi,&RamkaZapis);
		RamkaOdczyt.CMD=0;
		RamkaZapis.frame[1]=0x0E;
		WyslijRamke(&RamkaZapis);
		
		break;
		
		case 0x0F:  //odczyt progrow akcelerometru
		tmp[0]=FiltryKonfiguracja.prog_gorny_acc;
		tmp[1]=FiltryKonfiguracja.prog_dolny_acc;
		tmp[2]=0;
		
		WriteFloatToFrame(tmp,&RamkaZapis);
		RamkaOdczyt.CMD=0;
		RamkaZapis.frame[1]=0x0F;
		WyslijRamke(&RamkaZapis);
		
		break;
		
		case 0x10://wysylanie wartosci uchybu
			tmp[0]=Sygnaly.alfa_uchyb;
		tmp[1]=Sygnaly.beta_uchyb;
		tmp[2]=0;
		
		WriteFloatToFrame(tmp,&RamkaZapis);
		RamkaOdczyt.CMD=0;
		RamkaZapis.frame[1]=0x10;
		WyslijRamke(&RamkaZapis);
			
		
		break;
		
		case 0x11: //wyslanie zyroskopu po filtracjio
		tmp[0]=L3GD40_GYRO.xks10;
		tmp[1]=L3GD40_GYRO.yks10;
		tmp[2]=L3GD40_GYRO.zks10;
		
		WriteFloatToFrame(tmp,&RamkaZapis);
		RamkaOdczyt.CMD=0;
		RamkaZapis.frame[1]=0x11;
		WyslijRamke(&RamkaZapis);
		
			
		break;
		
		case 0x12:  //read magnet
		tmp[0]=HMC_MAGNET.xks10;
		tmp[1]=HMC_MAGNET.yks10;
		tmp[2]=HMC_MAGNET.zks10;

			WriteFloatToFrame(tmp,&RamkaZapis);
		RamkaOdczyt.CMD=0;
		RamkaZapis.frame[1]=0x12;
		WyslijRamke(&RamkaZapis);
		break;
		
		case 0x13: //read baro
		tmp[0]=BMP180.temperature;
		tmp[1]=BMP180.pressure;
		tmp[2]=BMP180.wysokosc;
		
		WriteFloatToFrame(tmp,&RamkaZapis);
		RamkaOdczyt.CMD=0;
		RamkaZapis.frame[1]=0x13;
		WyslijRamke(&RamkaZapis);
		
		break;
		
		case 0x14: //read S1 reg rate
		
			
		
				tmp[0]=silnik1.reg_alfa_rate.Wyjscie;
				tmp[1]=silnik1.reg_beta_rate.Wyjscie;
		
				WriteFloatToFrame(tmp,&RamkaZapis);
				RamkaOdczyt.CMD=0;
				RamkaZapis.frame[1]=0x14;
				WyslijRamke(&RamkaZapis);
		
		break;
		
		case 0x15: //read s2 reg rate
							tmp[0]=silnik2.reg_alfa_rate.Wyjscie;
				tmp[1]=silnik2.reg_beta_rate.Wyjscie;
		
				WriteFloatToFrame(tmp,&RamkaZapis);
				RamkaOdczyt.CMD=0;
				RamkaZapis.frame[1]=0x15;
				WyslijRamke(&RamkaZapis);
		break;
		
		case 0x16: //read s3 reg rate
							tmp[0]=silnik3.reg_alfa_rate.Wyjscie;
				tmp[1]=silnik3.reg_beta_rate.Wyjscie;
		
				WriteFloatToFrame(tmp,&RamkaZapis);
				RamkaOdczyt.CMD=0;
				RamkaZapis.frame[1]=0x16;
				WyslijRamke(&RamkaZapis);
		break;
		
		case 0x17: //read s4 reg rate
							tmp[0]=silnik4.reg_alfa_rate.Wyjscie;
				tmp[1]=silnik4.reg_beta_rate.Wyjscie;
		
				WriteFloatToFrame(tmp,&RamkaZapis);
				RamkaOdczyt.CMD=0;
				RamkaZapis.frame[1]=0x17;
				WyslijRamke(&RamkaZapis);
		break;
		
		case 0x18: //read orietnacja rate, prekosci katowe osi alfa beta gamma
			GPIOD->ODR^=LED_BLUE;
			tmp[0]=OrientacjaRate.AlfaRate;
			tmp[1]=OrientacjaRate.BetaRate;
			tmp[2]=OrientacjaRate.GammaRate;
		
			WriteFloatToFrame(tmp,&RamkaZapis);
			RamkaOdczyt.CMD=0;
			RamkaZapis.frame[1]=0x18;
			WyslijRamke(&RamkaZapis);
		break;
		
		case 0x19: //read uchyb rate
		tmp[0]=Sygnaly.alfa_rate_uchyb;
			tmp[1]=Sygnaly.beta_rate_uchyb;
		tmp[2]=0;
		
		WriteFloatToFrame(tmp,&RamkaZapis);
		RamkaOdczyt.CMD=0;
		RamkaZapis.frame[1]=0x19;
		WyslijRamke(&RamkaZapis);
		
		break;
		
		default:
					
			break;
		
		
	}
	
	
}

void ZapisDanych(void){
	float tmp[3];
	uint16_t tmpui[6];
	int16_t tmpi[6];
	
	switch(RamkaOdczyt.CMD){
		case 0x81:		//zapis danych pwm test
		
		
		ReadIntFromFrame(tmpi,&RamkaOdczyt);
		
		PWMdata.PWM_S1=tmpi[0];
		PWMdata.PWM_S2=tmpi[1];
		PWMdata.PWM_S3=tmpi[2];
		PWMdata.PWM_S4=tmpi[3];
		setPWM(&PWMdata);
		RamkaOdczyt.CMD=0;
		
		RamkaZapis.frame[1]=0x81;
		WyslijRamke(&RamkaZapis);
		RamkaOdczyt.CMD=0;
		break;
		
		case 0x82:			//zapis ograniczen pwm
			
		ReadIntFromFrame(tmpi,&RamkaOdczyt);
		PWMdata.MAX_PWM=tmpi[0];
		PWMdata.PWM_TOLERANCJA=tmpi[1];
		Sterowanie.max_wychylenie=tmpi[2];
		RamkaOdczyt.CMD=0;
		
		RamkaZapis.frame[1]=0x82;
		WyslijRamke(&RamkaZapis);
		RamkaOdczyt.CMD=0;
		break;
		
		case 0x83: //stop button
			GPIOD->ODR^=LED_GREEN;
			start=0;
			RamkaZapis.frame[1]=0x83;
		WyslijRamke(&RamkaZapis);
		RamkaOdczyt.CMD=0;
		break;
		
		case 0x84: //gora button
			GPIOD->ODR^=LED_RED;
		start=1;
		RamkaZapis.frame[1]=0x84;
		WyslijRamke(&RamkaZapis);
		RamkaOdczyt.CMD=0;
		break;
		
		case 0x85: //dol button
			GPIOD->ODR^=LED_BLUE;
			
		RamkaZapis.frame[1]=0x85;
		WyslijRamke(&RamkaZapis);
		RamkaOdczyt.CMD=0;
		break;
		
		case 0x87: //skladowa H ustawienie
			
		GPIOD->ODR = LED_ORANGE;
	
		
			ReadIntFromFrame(tmpi,&RamkaOdczyt);
		
		silnik1.wysokosc.stalaH=tmpi[0];
		silnik2.wysokosc.stalaH=tmpi[0];
		silnik3.wysokosc.stalaH=tmpi[0];
		silnik4.wysokosc.stalaH=tmpi[0];
		
		start=tmpi[1];
		
		RamkaOdczyt.CMD=0;
		RamkaZapis.frame[1]=0x87;
		WyslijRamke(&RamkaZapis);
		RamkaOdczyt.CMD=0;
		
		break;
		
		case 0x88:
			ReadIntFromFrame(tmpi,&RamkaOdczyt);
		
		tryb=tmpi[0];
		
		RamkaOdczyt.CMD=0;
		RamkaZapis.frame[1]=0x88;
		WyslijRamke(&RamkaZapis);
		RamkaOdczyt.CMD=0;
		
		break;
		case 0x89: //zapis regulator alfa
			ReadIntFromFrame(tmpi,&RamkaOdczyt);
		
		silnik1.reg_alfa.K=tmpi[0];
		silnik2.reg_alfa.K=tmpi[0];
		silnik3.reg_alfa.K=tmpi[0];
		silnik4.reg_alfa.K=tmpi[0];
		
		silnik1.reg_alfa.Ki=tmpi[1];
		silnik2.reg_alfa.Ki=tmpi[1];
		silnik3.reg_alfa.Ki=tmpi[1];
		silnik4.reg_alfa.Ki=tmpi[1];
		
		silnik1.reg_alfa.ZakresGora=tmpi[2];
		silnik2.reg_alfa.ZakresGora=tmpi[2];
		silnik3.reg_alfa.ZakresGora=tmpi[2];
		silnik4.reg_alfa.ZakresGora=tmpi[2];
		
		silnik1.reg_alfa.ZakresDol=tmpi[3];
		silnik2.reg_alfa.ZakresDol=tmpi[3];
		silnik3.reg_alfa.ZakresDol=tmpi[3];
		silnik3.reg_alfa.ZakresDol=tmpi[3];
		
		silnik1.reg_alfa.Kd = tmpi[4];
		silnik2.reg_alfa.Kd = tmpi[4];
		silnik3.reg_alfa.Kd = tmpi[4];
		silnik4.reg_alfa.Kd = tmpi[4];
		
			RamkaOdczyt.CMD=0;
		RamkaZapis.frame[1]=0x89;
		WyslijRamke(&RamkaZapis);
		RamkaOdczyt.CMD=0;
		
		break;
		
		case 0x8A: //zapis regulator beta
			
		ReadIntFromFrame(tmpi,&RamkaOdczyt);
		
		silnik1.reg_beta.K=tmpi[0];
		silnik2.reg_beta.K=tmpi[0];
		silnik3.reg_beta.K=tmpi[0];
		silnik4.reg_beta.K=tmpi[0];
		
		silnik1.reg_beta.Ki=tmpi[1];
		silnik2.reg_beta.Ki=tmpi[1];
		silnik3.reg_beta.Ki=tmpi[1];
		silnik4.reg_beta.Ki=tmpi[1];
		
		silnik1.reg_beta.ZakresGora=tmpi[2];
		silnik2.reg_beta.ZakresGora=tmpi[2];
		silnik3.reg_beta.ZakresGora=tmpi[2];
		silnik4.reg_beta.ZakresGora=tmpi[2];
		
		silnik1.reg_beta.ZakresDol=tmpi[3];
		silnik2.reg_beta.ZakresDol=tmpi[3];
		silnik3.reg_beta.ZakresDol=tmpi[3];
		silnik4.reg_beta.ZakresDol=tmpi[3];
		
		silnik1.reg_beta.Kd = tmpi[4];
		silnik2.reg_beta.Kd = tmpi[4];
		silnik3.reg_beta.Kd = tmpi[4];
		silnik4.reg_beta.Kd = tmpi[4];
		
		
			RamkaOdczyt.CMD=0;
		RamkaZapis.frame[1]=0x8A;
		WyslijRamke(&RamkaZapis);
		RamkaOdczyt.CMD=0;
		
		break;
		
		case 0x8B://surowe dane akcelerometr zyroskop
			tmpi[0]=LSM303DLHC_ACC.x;
			tmpi[1]=LSM303DLHC_ACC.y;
			tmpi[2]=LSM303DLHC_ACC.z;
			tmpi[3]=L3GD20.x;
			tmpi[4]=L3GD20.y;
			tmpi[5]=L3GD20.z;
		
		
			
	WriteInt16ToFrame(tmpi,&RamkaZapis);
		RamkaZapis.frame[1]=0x8B;
		RamkaOdczyt.CMD=0;
		WyslijRamke(&RamkaZapis);
		
		break;
		
		case 0x8C: //zapis konfiguracja filtrow 1
		ReadIntFromFrame(tmpi,&RamkaOdczyt);
		FiltryKonfiguracja.srednia_acc=tmpi[0];
		FiltryKonfiguracja.srednia_gyro=tmpi[1];
		FiltryKonfiguracja.srednia_magnet=tmpi[2];
		FiltryKonfiguracja.srednia_baro=tmpi[3];
		FiltryKonfiguracja.b1= (float)(tmpi[4]/100.0);
		FiltryKonfiguracja.b2=(float)(tmpi[5]/100.0);
		RamkaOdczyt.CMD=0;
		RamkaZapis.frame[1]=0x8C;
		WyslijRamke(&RamkaZapis);
		getPWM(&PWMdata);
		if(PWMdata.PWM_S1==0 && PWMdata.PWM_S2==0 && PWMdata.PWM_S3==0 && PWMdata.PWM_S4==0){
			
					initBuforFilo(&ACC_BUF_X,FiltryKonfiguracja.srednia_acc);
					initBuforFilo(&ACC_BUF_Y,FiltryKonfiguracja.srednia_acc);
					initBuforFilo(&ACC_BUF_Z,FiltryKonfiguracja.srednia_acc);
	
					initBuforFilo(&GYRO_BUF_X,FiltryKonfiguracja.srednia_gyro);
					initBuforFilo(&GYRO_BUF_Y,FiltryKonfiguracja.srednia_gyro);
					initBuforFilo(&GYRO_BUF_Z,FiltryKonfiguracja.srednia_gyro);
					
					FILTR_DATA.b1=FiltryKonfiguracja.b1;
				FILTR_DATA.b2=FiltryKonfiguracja.b2;
				
				Dane_Akcelerometr.prog_dolny=FiltryKonfiguracja.prog_dolny_acc;
				Dane_Akcelerometr.prog_gorny=FiltryKonfiguracja.prog_gorny_acc;
			GPIOD->ODR |=LED_BLUE;
			
			clearBuffor(&ACC_BUF_X);
			clearBuffor(&ACC_BUF_Y);
			clearBuffor(&ACC_BUF_Z);
			
			clearBuffor(&GYRO_BUF_X);
			clearBuffor(&GYRO_BUF_Y);
			clearBuffor(&GYRO_BUF_Z);
			nowe_bufory=0;
			licznik_bufory_nowe=0;
			}
		
	
		break;
		
		case 0x8D: //zapis konfiguracja filtrow 2
			GPIOD->ODR |= LED_ORANGE;
			ReadFloatFromFrame(tmp,&RamkaOdczyt);
		FiltryKonfiguracja.prog_gorny_acc=tmp[0];
		FiltryKonfiguracja.prog_dolny_acc=tmp[1];
		
		RamkaOdczyt.CMD=0;
		RamkaZapis.frame[1]=0x8D;
		WyslijRamke(&RamkaZapis);
		break;
		
		
		case 0x8E: // zapis regulatora pid alfa Rate
		ReadIntFromFrame(tmpi,&RamkaOdczyt);
		
		
		silnik1.reg_alfa_rate.K=tmpi[0];
		silnik2.reg_alfa_rate.K=tmpi[0];
		silnik3.reg_alfa_rate.K=tmpi[0];
		silnik4.reg_alfa_rate.K=tmpi[0];
		
		silnik1.reg_alfa_rate.Ki=tmpi[1];
		silnik2.reg_alfa_rate.Ki=tmpi[1];
		silnik3.reg_alfa_rate.Ki=tmpi[1];
		silnik4.reg_alfa_rate.Ki=tmpi[1];
		
		silnik1.reg_alfa_rate.ZakresGora=tmpi[2];
		silnik2.reg_alfa_rate.ZakresGora=tmpi[2];
		silnik3.reg_alfa_rate.ZakresGora=tmpi[2];
		silnik4.reg_alfa_rate.ZakresGora=tmpi[2];
		
		silnik1.reg_alfa_rate.ZakresDol=tmpi[3];
		silnik2.reg_alfa_rate.ZakresDol=tmpi[3];
		silnik3.reg_alfa_rate.ZakresDol=tmpi[3];
		silnik3.reg_alfa_rate.ZakresDol=tmpi[3];
		
		silnik1.reg_alfa_rate.Kd = tmpi[4];
		silnik2.reg_alfa_rate.Kd = tmpi[4];
		silnik3.reg_alfa_rate.Kd = tmpi[4];
		silnik4.reg_alfa_rate.Kd = tmpi[4];
		
		RamkaOdczyt.CMD=0;
		RamkaZapis.frame[1]=0x8E;
		WyslijRamke(&RamkaZapis);
		break;
		
		case 0x8F: // zapis regulatora pid beta Rate
			
			ReadIntFromFrame(tmpi,&RamkaOdczyt);
		
		silnik1.reg_beta_rate.K=tmpi[0];
		silnik2.reg_beta_rate.K=tmpi[0];
		silnik3.reg_beta_rate.K=tmpi[0];
		silnik4.reg_beta_rate.K=tmpi[0];
		
		silnik1.reg_beta_rate.Ki=tmpi[1];
		silnik2.reg_beta_rate.Ki=tmpi[1];
		silnik3.reg_beta_rate.Ki=tmpi[1];
		silnik4.reg_beta_rate.Ki=tmpi[1];
		
		silnik1.reg_beta_rate.ZakresGora=tmpi[2];
		silnik2.reg_beta_rate.ZakresGora=tmpi[2];
		silnik3.reg_beta_rate.ZakresGora=tmpi[2];
		silnik4.reg_beta_rate.ZakresGora=tmpi[2];
		
		silnik1.reg_beta_rate.ZakresDol=tmpi[3];
		silnik2.reg_beta_rate.ZakresDol=tmpi[3];
		silnik3.reg_beta_rate.ZakresDol=tmpi[3];
		silnik4.reg_beta_rate.ZakresDol=tmpi[3];
		
		silnik1.reg_beta_rate.Kd = tmpi[4];
		silnik2.reg_beta_rate.Kd = tmpi[4];
		silnik3.reg_beta_rate.Kd = tmpi[4];
		silnik4.reg_beta_rate.Kd = tmpi[4];
		
		RamkaOdczyt.CMD=0;
		RamkaZapis.frame[1]=0x8F;
		WyslijRamke(&RamkaZapis);
		break;
		
		default:
			
		
			break;
		
	}
	
	
}


void WyslijRamke(RAMKA *f){
(*f).index=0;
	//odblokowanie przerwania od pustego bufora wysyalnia danych
	USART6->CR1 |=USART_CR1_TXEIE;
	
}
