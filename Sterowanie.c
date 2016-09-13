#include <stm32f4xx.h>
#include <math.h>

#include"Sterowanie.h"


void InitStructPWM(PWM_CONF *pwm){
	(*pwm).MAX_PWM=0;
	(*pwm).PWM_TOLERANCJA=0;
	(*pwm).PWM_S1=0;
	(*pwm).PWM_S2=0;
	(*pwm).PWM_S3=0;
	(*pwm).PWM_S4=0;
	
}

void InitStructSterowanie(STEROWANIE *str){
	//(*str).max_wychylenie;
}

void InitPWM(void){
	//odblokowanie zegara dla portow A i B
	RCC->AHB1ENR|=(RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN);
	//odblokowanie zegara timerow

		GPIOA->PUPDR |= (1<<2);
	GPIOB->PUPDR |= (1<<20);
	
	GPIOA->OSPEEDR |= ((1<<2)|(1<<3));
	GPIOB->OSPEEDR|= ((1<<20)|(1<<21));
	
	//ustawienie gpio dla pwm
	GPIOA->MODER |= GPIO_MODER_MODER1_1;
	GPIOB->MODER |= GPIO_MODER_MODER10_1;
	
	GPIOA->AFR[0] |= (1<<4);//AF1
	GPIOB->AFR[1] |= (1<<8);//AF1
	
	RCC->APB1ENR|=RCC_APB1ENR_TIM2EN;
	
	TIM2->CCMR1 |= ((1<<14)|(1<<13));//MODE 1  CH2
	TIM2->CCMR2 |= ((1<<6)|(1<<5));//MODE 1 CH3
	TIM2->CCMR1 |=  (1<<11);//preloda ch2
	TIM2->CCMR2 |= (1<<3);//PRELOAD CH3
	
	//ODBLOKOWANIE KONALAOW
	TIM2->CCER |= ((1<<4)|(1<<8));//ODBLOKOWANI
	
	TIM2->ARR =40000;
	TIM2->PSC=8;
	
	//TIM2->CCR2=0;
	//TIM2->CCR3=0;
	
	TIM2->CNT = 0x00;
	TIM2->DIER |= TIM_DIER_UIE;
	//TIM2->CR1 |= (1<<7);//ARPE - AUTO RELOAD PRELOAD ENABLE	TIM2->CR1 |= TIM_CR1_CEN;
	TIM2->EGR |= 1;///JAKIS UPDATE EVENT
	TIM2->CR1 |= TIM_CR1_CEN;
	//NVIC_EnableIRQ(TIM2_IRQn);
	
	
	//TIMER 3 
	GPIOB->PUPDR |= ((1<<0)|(1<<2));
		GPIOB->OSPEEDR|= ((1<<0)|(1<<1)|(1<<2)|(1<<3));
		
		GPIOB->MODER |= (GPIO_MODER_MODER0_1|GPIO_MODER_MODER1_1);
		
		GPIOB->AFR[0] |= ((1<<1)|(1<<5));
		
		RCC->APB1ENR|=RCC_APB1ENR_TIM3EN;
		
		TIM3->CCMR2 |= ((1<<14)|(1<<13));//MODE 1  CH2
	TIM3->CCMR2 |= ((1<<6)|(1<<5));//MODE 1 CH3
	TIM3->CCMR1 |=  (1<<11);//preloda ch2
	TIM3->CCMR2 |= (1<<3);//PRELOAD CH3
	
	//ODBLOKOWANIE KONALAOW
	TIM3->CCER |= ((1<<12)|(1<<8));//ODBLOKOWANI
	
	TIM3->ARR =40000;
	TIM3->PSC=8;
	
	//TIM3->CCR4=0;
//	TIM3->CCR3=0;
	
	TIM3->CNT = 0x00;
	TIM3->DIER |= TIM_DIER_UIE;
	//TIM2->CR1 |= (1<<7);//ARPE - AUTO RELOAD PRELOAD ENABLE	TIM2->CR1 |= TIM_CR1_CEN;
	TIM3->EGR |= 1;///JAKIS UPDATE EVENT
	TIM3->CR1 |= TIM_CR1_CEN;
	//NVIC_EnableIRQ(TIM2_IRQn);
	
}

void setPWM(PWM_CONF *f){
	if((*f).PWM_S1<=0)(*f).PWM_S1=0;
	if((*f).PWM_S2<=0)(*f).PWM_S2=0;
	if((*f).PWM_S3<=0)(*f).PWM_S3=0;
	if((*f).PWM_S4<=0)(*f).PWM_S4=0;
	
	if((*f).PWM_S1>10000)(*f).PWM_S1=10000;
	if((*f).PWM_S2>10000)(*f).PWM_S2=10000;
	if((*f).PWM_S3>10000)(*f).PWM_S3=10000;
	if((*f).PWM_S4>10000)(*f).PWM_S4=10000;
	
	S1_PWM=(*f).PWM_S4+8500;
	S2_PWM=(*f).PWM_S3+8500;
	S3_PWM=(*f).PWM_S2+8500;
	S4_PWM=(*f).PWM_S1+8500;
	
}


void getPWM(PWM_CONF *f){
	
	(*f).PWM_S1=S4_PWM;
	(*f).PWM_S2=S3_PWM;
	(*f).PWM_S3=S2_PWM;
	(*f).PWM_S4=S1_PWM;
	
}

void Uchyb(SYGNALY *sig,double Alfa_pomiar,double Beta_pomiar){
(*sig).alfa_uchyb=(*sig).alfa_zad-Alfa_pomiar;
(*sig).beta_uchyb=(*sig).beta_zad-Beta_pomiar;	
(*sig).alfa_pomiar=Alfa_pomiar;
(*sig).beta_pomiar=Beta_pomiar;
	
}

void UchybRate(SYGNALY *sig,SILNIK *s){
	(*sig).alfa_rate_zad=(*s).reg_alfa.Wyjscie;
	(*sig).beta_rate_zad=(*s).reg_beta.Wyjscie;

	(*sig).alfa_rate_uchyb=(*sig).alfa_rate_zad-(*sig).alfa_rate_pomiar;
	(*sig).beta_rate_uchyb=(*sig).beta_rate_zad-(*sig).beta_rate_pomiar;
	
}

void setRefSignal(SYGNALY *sig,double alfa,double beta){
	(*sig).alfa_zad=alfa;
	(*sig).beta_zad=beta;
	
}

void RegPID(SILNIK *s,double uchyb_alfa,double uchyb_beta,uint8_t ID_REG_PID){
	double tmp;
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	if(ID_REG_PID==ID_REG_PID_STABILIZE){		// Regulator PID stabilizacji
	//Regulator PID alfa
	tmp=((*s).reg_alfa.dti*(*s).reg_alfa.Ki*uchyb_alfa);

	(*s).reg_alfa.a=(*s).reg_alfa.a+tmp;
	
	if((*s).reg_alfa.a<=(*s).reg_alfa.ZakresDol)(*s).reg_alfa.a=(*s).reg_alfa.ZakresDol;
	if((*s).reg_alfa.a>=(*s).reg_alfa.ZakresGora)(*s).reg_alfa.a=(*s).reg_alfa.ZakresGora;

	
	(*s).reg_alfa.WyjscieD=(*s).reg_alfa.Kd*((uchyb_alfa-(*s).reg_alfa.Dtmp)/(*s).reg_alfa.dtd);
	
	(*s).reg_alfa.Wyjscie=0.01*(*s).reg_alfa.K*uchyb_alfa+(*s).reg_alfa.a+(*s).reg_alfa.WyjscieD;
	
	(*s).reg_alfa.Dtmp = uchyb_alfa;
	//Regulator PID beta
	
	tmp=((*s).reg_beta.dti*(*s).reg_beta.Ki*uchyb_beta);
	
	(*s).reg_beta.a=(*s).reg_beta.a+tmp;
	
	if((*s).reg_beta.a<=(*s).reg_beta.ZakresDol)(*s).reg_beta.a=(*s).reg_beta.ZakresDol;
	if((*s).reg_beta.a>=(*s).reg_beta.ZakresGora)(*s).reg_beta.a=(*s).reg_beta.ZakresGora;
	
	(*s).reg_beta.WyjscieD=(*s).reg_beta.Kd*((uchyb_beta-(*s).reg_beta.Dtmp)/(*s).reg_beta.dtd);
	
	(*s).reg_beta.Wyjscie=0.01*(*s).reg_beta.K*uchyb_beta + (*s).reg_beta.a+(*s).reg_beta.WyjscieD;
	
	(*s).reg_beta.Dtmp = uchyb_beta;
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	}else if(ID_REG_PID==ID_REG_PID_RATE){ //regulator PID predkosci katowej
	

		//Regulator PID alfa
	tmp=((*s).reg_alfa_rate.dti*(*s).reg_alfa_rate.Ki*uchyb_alfa);

	(*s).reg_alfa_rate.a=(*s).reg_alfa_rate.a+tmp;
	
	if((*s).reg_alfa_rate.a<=(*s).reg_alfa_rate.ZakresDol)(*s).reg_alfa_rate.a=(*s).reg_alfa_rate.ZakresDol;
	if((*s).reg_alfa_rate.a>=(*s).reg_alfa_rate.ZakresGora)(*s).reg_alfa_rate.a=(*s).reg_alfa_rate.ZakresGora;

	
	(*s).reg_alfa_rate.WyjscieD=(*s).reg_alfa_rate.Kd*((uchyb_alfa-(*s).reg_alfa_rate.Dtmp)/(*s).reg_alfa_rate.dtd);
	
	(*s).reg_alfa_rate.Wyjscie=0.01*(*s).reg_alfa_rate.K*uchyb_alfa+(*s).reg_alfa_rate.a+(*s).reg_alfa_rate.WyjscieD;
	
	(*s).reg_alfa_rate.Dtmp = uchyb_alfa;
	//Regulator PID beta
	
	tmp=((*s).reg_beta_rate.dti*(*s).reg_beta_rate.Ki*uchyb_beta);
	
	(*s).reg_beta_rate.a=(*s).reg_beta_rate.a+tmp;
	
	if((*s).reg_beta_rate.a<=(*s).reg_beta_rate.ZakresDol)(*s).reg_beta_rate.a=(*s).reg_beta_rate.ZakresDol;
	if((*s).reg_beta_rate.a>=(*s).reg_beta_rate.ZakresGora)(*s).reg_beta_rate.a=(*s).reg_beta_rate.ZakresGora;
	
	(*s).reg_beta_rate.WyjscieD=(*s).reg_beta_rate.Kd*((uchyb_beta-(*s).reg_beta_rate.Dtmp)/(*s).reg_beta_rate.dtd);
	
	(*s).reg_beta_rate.Wyjscie=0.01*(*s).reg_beta_rate.K*uchyb_beta + (*s).reg_beta_rate.a+(*s).reg_beta_rate.WyjscieD;
	
	(*s).reg_beta_rate.Dtmp = uchyb_beta;
	}//////////////////////////////////////////////////////////////////////////////////////////////////////////////
}

void setStalaH(SILNIK *s,double H,double uchyb_alfa,double uchyb_beta){
	double eAlfa = uchyb_alfa;
	double eBeta = uchyb_beta;
	if(eAlfa<0)eAlfa=(-1)*eAlfa;
	if(eBeta<0)eBeta=(-1)*eBeta;
	
	(*s).wysokosc.stalaH=H;
	//(*s).wysokosc.deltaH=0.25*((*s).PWM*sin(eAlfa*PI/180)+(*s).PWM*sin(eBeta*PI/180));
	
	(*s).wysokosc.wys=(*s).wysokosc.stalaH + (*s).wysokosc.deltaH;
	
}

void RegSilnik(SILNIK *s,double H,double uchyb_alfa,double uchyb_beta){
	
	setStalaH(s,H,uchyb_alfa,uchyb_beta);
	RegPID(s,uchyb_alfa,uchyb_beta,ID_REG_PID_STABILIZE);
	
	(*s).PWM=(*s).wysokosc.wys+(*s).reg_alfa.Wyjscie+(*s).reg_beta.Wyjscie;
	
	if((*s).PWM<=0)(*s).PWM=0;
	
}

void RegSilnik2(SILNIK *s,SYGNALY *signal){
	
		setStalaH(s,(*s).wysokosc.stalaH,(*signal).alfa_uchyb,(*signal).beta_uchyb);
		
		double eAlfa,eBeta,eAlfaRate,eBetaRate;
		
		//zmiana znaku uchybu w zaleznosci od silnika
		switch((*s).ID_SILNIKA){
		case ID_SILNIK_1:
			eAlfa=(-1)*(*signal).alfa_uchyb;
			eBeta=(-1)*(*signal).beta_uchyb;
		break;
		case ID_SILNIK_2:
			eAlfa=(*signal).alfa_uchyb;
			eBeta=(-1)*(*signal).beta_uchyb;
		break;
		case ID_SILNIK_3:
			eAlfa=(-1)*(*signal).alfa_uchyb;
			eBeta=(*signal).beta_uchyb;
		break;
		case ID_SILNIK_4:
			eAlfa=(*signal).alfa_uchyb;
			eBeta=(*signal).beta_uchyb;
		break;
	}
		
	RegPID(s,eAlfa,eBeta,ID_REG_PID_STABILIZE);			// Regulator PID Stabilize
	
	UchybRate(signal,s);
	//zmiana znaku uchybu w zaleznosci od silnika
	/*
		switch((*s).ID_SILNIKA){
		case ID_SILNIK_1:
			eAlfaRate=(-1)*(*signal).alfa_rate_uchyb;
			eBetaRate=(-1)*(*signal).beta_rate_uchyb;
		break;
		case ID_SILNIK_2:
			eAlfaRate =(*signal).alfa_rate_uchyb;
			eBetaRate = (-1)*(*signal).beta_rate_uchyb;
		break;
		case ID_SILNIK_3:
			eAlfaRate = (-1)*(*signal).alfa_rate_uchyb;
			eBetaRate = (*signal).beta_rate_uchyb;
		break;
		case ID_SILNIK_4:
			eAlfaRate=(*signal).alfa_rate_uchyb;
			eBetaRate=(*signal).beta_rate_uchyb;
		break;
	}
	*/
	eAlfaRate=(*signal).alfa_rate_uchyb;
	eBetaRate=(*signal).beta_rate_uchyb;
	
	RegPID(s,eAlfaRate,eBetaRate,ID_REG_PID_RATE); 				//Regulator PID RATE
	
	(*s).PWM=(*s).wysokosc.wys+(*s).reg_alfa_rate.Wyjscie+(*s).reg_beta_rate.Wyjscie;
	
	if((*s).PWM<=0)(*s).PWM=0;
	
}

void InitSilnik(SILNIK *s,uint8_t ID_SILNIKA){
	// PID STABILIZE
	
	(*s).ID_SILNIKA = ID_SILNIKA;
	(*s).reg_alfa.K=1;
	(*s).reg_alfa.Ki=1;
	(*s).reg_alfa.Kd=1;
	(*s).reg_alfa.dti=1.0/100000.0;
	(*s).reg_alfa.dtd=1.0/250.0;
	(*s).reg_alfa.ZakresDol=-2;
	(*s).reg_alfa.ZakresGora=2;
	
	
	(*s).reg_beta.K=1;
	(*s).reg_beta.Ki=1;
	(*s).reg_beta.Kd=1;
	(*s).reg_beta.dti=1.0/100000.0;
	(*s).reg_beta.dtd=1.0/250.0;
	(*s).reg_beta.ZakresDol=-2;
	(*s).reg_beta.ZakresGora=2;
	
	
	// PID RATE
		(*s).reg_alfa_rate.K=1;
	(*s).reg_alfa_rate.Ki=1;
	(*s).reg_alfa_rate.Kd=1;
	(*s).reg_alfa_rate.dti=1.0/100000.0;
	(*s).reg_alfa_rate.dtd=1.0/250.0;
	(*s).reg_alfa_rate.ZakresDol=-2;
	(*s).reg_alfa_rate.ZakresGora=2;
	
	
	(*s).reg_beta_rate.K=1;
	(*s).reg_beta_rate.Ki=1;
	(*s).reg_beta_rate.Kd=1;
	(*s).reg_beta_rate.dti=1.0/100000.0;
	(*s).reg_beta_rate.dtd=1.0/250.0;
	(*s).reg_beta_rate.ZakresDol=-2;
	(*s).reg_beta_rate.ZakresGora=2;
	
}

void ZerowanieParametrow(SILNIK *s){

(*s).reg_alfa.Wyjscie=0;
(*s).reg_alfa.a=0;
(*s).reg_beta.a=0;
(*s).reg_beta.Wyjscie=0;
(*s).wysokosc.stalaH=0;
(*s).wysokosc.deltaH=0;
(*s).wysokosc.wys=0;
(*s).PWM=0;	
	
(*s).reg_alfa_rate.Wyjscie=0;
(*s).reg_alfa_rate.a=0;
(*s).reg_beta_rate.a=0;
(*s).reg_beta_rate.Wyjscie=0;
	
}


