
#include <stm32f4xx.h>
#include"SPI.h"

//inicjalizacja SPI1
void SPI1_INIT(void){
//SPI1_SCK ------ PA5
//SPI1_MOSI ----- PA7
//SPI1_MISO ----- PA6
//CS_I2C/SPI ---- PE3
	
	//odblokowanie zegara portow
	RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOEEN);
	//odblokowanie zegara SPI1
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	
	
	
	//konfiguracja CS
	GPIOE->MODER |=GPIO_MODER_MODER3_0;
	
	CS_HIGH;
	//konfiguracja GPIO
	//SPI1 AF5
	
	GPIOA->MODER |= GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;	//AF
	GPIOA->OSPEEDR |= (1<<10)|(1<<12)|(1<<14);	//medium speed
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR5_1 | GPIO_PUPDR_PUPDR6_1 | GPIO_PUPDR_PUPDR7_1;//Pull up
	//AF5 dla pinow 5 6 7 , AF5=0101
	GPIOA->AFR[0] |= (1<<20)|(1<<22)|(1<<24)|(1<<26)|(1<<28)|(1<<30); //
	
	//inicjalizacja SPI1
	
	SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR_0| SPI_CR1_BR_1| SPI_CR1_BR_2;//SOFATWARE SLAVE Fpclk/256
	
	SPI1->CR1 |= SPI_CR1_MSTR | SPI_CR1_SPE;	//USTAWIENIE MASTER I ODBLOKOWANIE 
	
}

void SPI4_INIT(void){
//PE6 - MOSI
//PE5 - MISO
//PE2 - SCK
	//PB7 - CS
	//odblokowanie zegara portow
	RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOBEN);
	
	//odblokowanie zegara SPI1
	RCC->APB2ENR |= RCC_APB2ENR_SPI4EN;
	
	
	
	//konfiguracja CS
	GPIOB->MODER |=GPIO_MODER_MODER7_0;
	
	//MPU_CS_HIGH;
	//konfiguracja GPIO
	//SPI1 AF5
	
	GPIOE->MODER |= GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER2_1;	//AF
	GPIOE->OSPEEDR |= (1<<10)|(1<<12)|(1<<4);	//medium speed
	GPIOE->PUPDR |= GPIO_PUPDR_PUPDR5_1 | GPIO_PUPDR_PUPDR6_1 | GPIO_PUPDR_PUPDR2_1;//Pull up
	//AF5 dla pinow 5 6 7 , AF5=0101
	GPIOE->AFR[0] |= (1<<20)|(1<<22)|(1<<24)|(1<<26)|(1<<8)|(1<<10); //
	
	//inicjalizacja SPI1
	
	SPI4->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR_0| SPI_CR1_BR_1| SPI_CR1_BR_2| SPI_CR1_DFF |SPI_CR1_CPHA ;//SOFATWARE SLAVE Fpclk/256
	
	SPI4->CR1 |= SPI_CR1_MSTR | SPI_CR1_SPE;	//USTAWIENIE MASTER I ODBLOKOWANIE 
	
}

uint8_t spi4_transfer_byte(uint8_t data){
	
	while(!(SPI4->SR & SPI_SR_TXE));
	SPI4->DR = data;
	while(!(SPI4->SR & SPI_SR_RXNE));
	return (uint8_t)SPI4->DR;
	
	
}

uint16_t spi4_transfer_2byte(uint8_t reg,uint8_t data){
	KonwersjaSlowo tmp;
	tmp.data_in[0]=reg;
	tmp.data_in[1]=data;
	while(!(SPI4->SR & SPI_SR_TXE));
	SPI4->DR = tmp.data;
	while(!(SPI4->SR & SPI_SR_RXNE));
	return SPI4->DR;

}

uint8_t spi1_transfer_byte(uint8_t data){
	while(!(SPI1->SR & SPI_SR_TXE));
	SPI1->DR = data;
	while(!(SPI1->SR & SPI_SR_RXNE));
	return (uint8_t)SPI1->DR;
}
