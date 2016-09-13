#ifndef LSM303DLHC1
#define LSM303DLHC1

//AKCELEROMETR LSM303DLHC
#define ACC_LSM303DLHC_READ 0x33
#define ACC_LSM303DLHC_WRITE 0x32
#define LSM303DLHC 0X32
#define ACC_READ_ALL_AXIS 0xA7

#define AKCELEROMETR 0xA8
#define MAGNETOMETR 0x83

#define M1 3.96e-05

void LSM303DLHC_acc_enable(MEMS *mems);
void LSM303DLHC_READ_AXIS(MEMS *mems,uint8_t TYP);

void LSM303DLHC_magnet_enable(MEMS *mems);



#endif
