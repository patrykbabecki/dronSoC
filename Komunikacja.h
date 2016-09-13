#ifndef KOMUNIKACJA_H
#define KOMUNIKACJA_H

#define ADDR 0xAA
#define CONTROL 0xAA
#define MASK_WRITE 0x80
#define FRAME_LENGTH 16

typedef struct kom{
	uint8_t ADDR_ID;
	uint8_t CMD;
	uint8_t data[12];
	uint8_t KONTROLA;
	uint8_t frame[FRAME_LENGTH];
	uint8_t length;
	uint8_t FRAME_OK;
	uint8_t index;
	
}RAMKA;

typedef union KonFloat{
	float f;
	uint8_t b[4];
	
}KonwersFloat;

void ReadFloatFromFrame(float *data,RAMKA *f);

typedef union intToFloat{
	float f;
	uint8_t d[4];
} IntToFloat;

void KomunikacjaStructInit(RAMKA *f);

void ReadIntFromFrame(int16_t *data,RAMKA *f);

void WriteFloatToFrame(float *data,RAMKA *f);

void WriteInt16ToFrame(uint16_t *data,RAMKA *f);




#endif