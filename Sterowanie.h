#ifndef STEROWANIE_H
#define STEROWANIE_H

#define PI 3.14159

#define S1_PWM TIM2->CCR3
#define S2_PWM TIM3->CCR4

#define S3_PWM TIM2->CCR2
#define S4_PWM TIM3->CCR3

#define ID_SILNIK_1 1
#define ID_SILNIK_2 2
#define ID_SILNIK_3 3
#define ID_SILNIK_4 4

#define ID_REG_PID_STABILIZE 1
#define ID_REG_PID_RATE 2

//PWM_S2 PB10 TIM2_CH3  ( PIERWSZY OD LINI ZASILANIA SEPARACJI GALWANICZNEJ)
//PWM_S1 PA1 TIM2_CH2 ( TA SAMA STRONA CO PWM_S1 2GI OD LINI ZASILANIA SEPARACJI GALWANICZNEJ)

//PWM_S3 PB1 TIM3_CH4 (PIERWSY OD MODULU UJCZNIKA MEMS Z BAROMETREM)
//PWM_S4 PB0 TIM3_CH3  ( OBOK POWYZSZEGO);

typedef struct ParametryPwm{
	uint16_t PWM_S1,PWM_S2,PWM_S3,PWM_S4;
	uint16_t MAX_PWM;
	uint16_t PWM_TOLERANCJA;
	
}PWM_CONF;



typedef struct regpi{
	double K;
	double Ki;
	double Kd;
	double Dtmp;
	double a;
	double dti;
	double dtd;
	double Wyjscie;
	double WyjscieD;
	double ZakresGora;
	double ZakresDol;
}REG_PID;

typedef struct Sterowanie{
	
	float max_wychylenie;
	float alfa_ref;
	float beta_ref;
	
}STEROWANIE;

typedef struct skladowah{
	
	double stalaH;
	double deltaH;
	double wys;
	
}SKLADOWA_H;

typedef struct silnik{
	uint8_t ID_SILNIKA;
	REG_PID reg_alfa;
	REG_PID reg_beta;
	REG_PID reg_alfa_rate;
	REG_PID reg_beta_rate;
	SKLADOWA_H wysokosc;
	double PWM;
}SILNIK;

typedef struct sygnal{
	double alfa_pomiar,beta_pomiar;
	double alfa_rate_pomiar,beta_rate_pomiar;
	double alfa_rate_uchyb,beta_rate_uchyb;
	double alfa_uchyb,beta_uchyb;
	double alfa_zad,beta_zad;
	double alfa_rate_zad,beta_rate_zad;
	
}SYGNALY;

void ZerowanieParametrow(SILNIK *s);

void Uchyb(SYGNALY *sig,double Alfa_pomiar,double Beta_pomiar);

void UchybRate(SYGNALY *sig,SILNIK *s);

void setRefSignal(SYGNALY *sig,double alfa,double beta);

void RegPID(SILNIK *s,double uchyb_alfa,double uchyb_beta,uint8_t ID_REG_PID);

void setStalaH(SILNIK *s,double H,double uchyb_alfa,double uchyb_beta);
//1 regulator PID
void RegSilnik(SILNIK *s,double H,double uchyb_alfa,double uchyb_beta);
// 2 regulatory PID
void RegSilnik2(SILNIK *s,SYGNALY *signal);

void InitSilnik(SILNIK *s,uint8_t ID_SILNIKA);

void InitStructPWM(PWM_CONF *pwm);

void InitStructSterowanie(STEROWANIE *str);

void InitPWM(void);

void setPWM(PWM_CONF *f);

void getPWM(PWM_CONF *f);



#endif