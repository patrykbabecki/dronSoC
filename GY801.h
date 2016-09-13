#ifndef GY801_H
#define GY801_H


#define L3GD40 0xD2
#define L3GD40_AXIS 0x28

void L3GD40_ENABLE(MEMS *mems);
void L3GD40_READ_AXIS(MEMS *mems);

//#define ADXL345_ID 0x1D
/*
#define ADXL345_W 0x3A
#define ADXL345_R 0x3B
*/
#define ADXL345_AXIS 0x32

void ADXL345_ENABLE(MEMS *mems);
void ADXL345_READ_AXIS(MEMS *mems);
//JESLI PIN ALT ADRES JEST NISKI TO

#define ADXL345_ID 0x53
#define ADXL345_W 0xA6
#define ADXL345_R 0xA7

//max 200Hz odr

#define BMP180_W 0xEE
#define BMP180_R 0xEF

#define ROZDZIELCZOSC_15 0.00003051
#define ROZDZIELCZOSC_12 0.0002441
#define ROZDZIELCZOSC_11 0.00049
#define ROZDZIELCZOSC_13 0.00122
#define ROZDZIELCZOSC_16 0.00001525
#define ROZDZIELCZOSC_2 0.25
#define ROZDZIELCZOSC_8 0.0039
#define ROZDZIELCZOSC_4 0.0652
#define DWADO11 65536
#define TEMPERA 0.0625


typedef struct baro{
	int16_t AC1,AC2,AC3,B1,B2,MB,MC,MD;
	uint16_t AC4,AC5,AC6;
	long t_reg;
	long p_reg;
	long X1,X2,X3,B5,B6,B3;
	unsigned long B4,B7;
	long temperature;
	long pressure;//Pa
	long P0; //Pa
	double wysokosc;
} BAROMETR;

void BMP180_ENABLE(BAROMETR *baro);
void BMP180_READ(BAROMETR *baro);
void BMP180_CALC(BAROMETR *baro);


#define HMC5883L_W 0x3C
#define HMC5883L_R 0x3D

void HMC5883L_ENABLE(MEMS *mems);
void HMC5883L_READ_AXIS(MEMS *mems);

//ADXL345 ACC	0xE5 ( device id)
//BMP180 BARO	
//HMC5883L MAGNET



#endif
