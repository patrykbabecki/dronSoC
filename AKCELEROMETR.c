#include <stm32f4xx.h>
#include<math.h>
#include"defines.h"
#include"AKCELEROMETR.h"

void CalcOrientacjaRate(ORIENTACJA_RATE *rate,double alfa, double beta, double gamma){
	(*rate).AlfaRate=(alfa -(*rate).tmp_alfa)/(*rate).dt1;
	(*rate).BetaRate=(beta -(*rate).tmp_beta)/(*rate).dt1;
	(*rate).GammaRate=(gamma-(*rate).tmp_gamma)/(*rate).dt1;
	
	(*rate).tmp_alfa=alfa;
	(*rate).tmp_beta=beta;
	(*rate).tmp_gamma=gamma;
	
}

void initOrientacjaRate(ORIENTACJA_RATE *rate,double dt1){
	(*rate).dt1=dt1;
	
}

void Obliczanie_Orientacji(MEMS *mems,double xks,double yks,double zks,ORIENTACJA_ACC *orient){
	//Skalowanie_Danych_Mems(mems);
	double x,y,z,Rxy,Rzy,Rzx;
	(*mems).wypadkowy=sqrt((xks)*(xks)+(yks)*(yks)+(zks)*(zks));
	
	//sprawdzanie czy wektor wypadkowy jest w okolicy 1 +- 10%
	if(((*orient).prog_dolny<(*mems).wypadkowy)&&((*mems).wypadkowy<(*orient).prog_gorny))(*orient).stan_dynamiczny=0;
	else(*orient).stan_dynamiczny=1;
		
	//dalsze obliczenia
	x=(xks)/(*mems).wypadkowy;
	y=(yks)/(*mems).wypadkowy;
	z=(zks)/(*mems).wypadkowy;
	
	Rxy=sqrt(x*x+y*y);
	Rzy=sqrt(z*z+y*y);
	Rzx=sqrt(z*z+x*x);
	
	(*orient).alfa=RAD*atan2(z/Rzy,y/Rzy);
	
	
	

	(*orient).beta=RAD*atan2(x/Rzx,z/Rzx);
	(*orient).gamma=RAD*atan2(y/Rxy,x/Rxy);
}