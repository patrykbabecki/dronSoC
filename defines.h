

#ifndef DEFINES
#define DEFINES

#define Z250 0.0076
#define Z500 0.0153
#define Z2000 0.061

#define A2 0.000061
#define A4 0.000122
#define A8 0.000244
#define A16 0.000488

#define G13 0.0000396
#define G81 0.0002471


typedef struct mems{
	
	int16_t x,y,z;		//odczytane dane
	
	int32_t p;
	
	uint16_t zakres;		//zakres na jakim obecnie pracuje dany mems
	
	uint16_t prog_gorny;	//prog dolny petli histerezy do zmiany zakresu
	uint16_t prog_dolny;	//prog gorny petli histerezy do zmiany zakresu
	
	uint8_t dryft;				//wartosc dryftu danego mems
	
	double konwersja;			//wspolczynnik zmiany jednostki
	
	double xk,yk,zk;
	
	float xks10,yks10,zks10; //srednia kroczaca 10
	float xks50,yks50,zks50;	//srednia kroczaca 50
	
	double wypadkowy;
}MEMS;



typedef struct k_dryft{
	double alfa,beta,gamma;
}ORIENT_DRYFT;

typedef struct orien{
	double alfa,beta,gamma;
	ORIENT_DRYFT dryft;
}ORIENTACJA;

typedef struct orie{
	double alfa,beta,gamma;
	uint8_t stan_dynamiczny;
	double prog_dolny,prog_gorny;
}ORIENTACJA_ACC;

void OdejmowanieDryftu(MEMS *m);
void Skalowanie_Danych_Mems(MEMS *m);

typedef struct orieRate{
	
	double tmp_alfa,tmp_beta,tmp_gamma;
	
	double AlfaRate,BetaRate,GammaRate;
	
	double dt1;
	
} ORIENTACJA_RATE;

#endif
