#ifndef I2C_DEF
#define I2C_DEF


void I2C1_INIT(void);
void delay(void);
void I2C_READ(int8_t address,int8_t reg_address,uint8_t *data,uint8_t length);
void I2C_WRITE(uint8_t address,uint8_t *data,uint32_t length);

void I2C3_INIT(void);
void I2C3_READ(int8_t address,int8_t reg_address,uint8_t *data,uint8_t length);
void I2C3_WRITE(uint8_t address,uint8_t *data,uint32_t length);

#endif
