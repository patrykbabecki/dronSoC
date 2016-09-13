#ifndef AKCELEROMETR
#define AKCELEROMETR

#define RAD 57.3248



void initOrientacjaRate(ORIENTACJA_RATE *rate, double dt1);

void CalcOrientacjaRate(ORIENTACJA_RATE *rate,double alfa, double beta, double gamma);

void Obliczanie_Orientacji(MEMS *mems,double xks,double yks,double zks,ORIENTACJA_ACC *orient);

#endif
