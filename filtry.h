
#ifndef FILTRY_H
#define FILTRY_H

typedef struct buf_FILO{
	uint8_t length;
	float bufor[75];
	uint8_t index;
	double suma;
}BUFOR_FILO;


void initBuforFilo(BUFOR_FILO *buf,uint8_t length);
void AddNewData(BUFOR_FILO *buf,float data);
float GetLastData(BUFOR_FILO *buf);
void clearBuffor(BUFOR_FILO *buf);

float SredniaKroczaca(BUFOR_FILO *buf,float new_data);

typedef struct ori{
	float alfa,beta,gamma;
	float b;//parametr filtru
	float dt;
	float b1;
	float b2;
}FILTR_KOMPLEMENTARNY;

typedef struct filtry_konfiguracja{
	float prog_dolny_acc,prog_gorny_acc;
	float b1,b2;
	uint16_t srednia_acc,srednia_gyro,srednia_magnet,srednia_baro;
	
}FILTRY_CONF;

void initFiltryConf(FILTRY_CONF *f);

float FiltrKomplementarny(FILTR_KOMPLEMENTARNY *fir,float gyro_data,float acc_angle,float angle);


#endif