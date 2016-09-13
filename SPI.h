#ifndef SPI_DEF
#define SPI_DEF


//SPI4 mpu9250
//PE6 - MOSI
//PE5 - MISO
//PE2 - SCK
//PB7 - CS

typedef union b8b16{
	uint16_t data;
	uint8_t data_in[2];
} KonwersjaSlowo;

void SPI4_INIT(void);
uint8_t spi4_transfer_byte(uint8_t data);
uint16_t spi4_transfer_2byte(uint8_t reg,uint8_t data);

#define MPU_CS_HIGH GPIOB->ODR |= (1<<7)
#define MPU_CS_LOW GPIOB->ODR &=~(1<<7)

#define CS_HIGH GPIOE->ODR |= (1<<3)
#define CS_LOW GPIOE->ODR &=~(1<<3)

void SPI1_INIT(void);
uint8_t spi1_transfer_byte(uint8_t data);


#endif
