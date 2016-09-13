
#include <stm32f4xx.h>
#include"I2C.h"

void delay(void){
int i;
for(i=0; i<0xFFF;i++);
	
}

//Inicjalizacja I2c
void I2C1_INIT(void){
	//konfiguracja GPIO
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	GPIOB->OTYPER |= ((1<<6)|(1<<9));			//OPEN DREN DAL pb 6 i 9
	GPIOB->PUPDR |= ((1<<13)|(1<<19)); 		//PULL UP
	GPIOB->OSPEEDR |= ((1<<13)|(1<<12)|(1<<19)|(1<<18));
	GPIOB->MODER |= ((1<<13)|(1<<19)); 		//AF 4
	GPIOB->AFR[0] |= (1<<26);							//I2C1
	GPIOB->AFR[1] |= (1<<6);							//I2C1
	
	  //odblokowanie zegara portu B
	
	
	RCC->APB1ENR |=RCC_APB1ENR_I2C1EN;
	//**********************KONFIGURACJA I2C1
	//I2C1->CR1 |= I2C_CR1_SWRST;
	//for(uint32_t i=0;i<0xFFFFF;i++);
	//I2C1->CR1 &=~I2C_CR1_SWRST;
	

	//I2C1->CR1=0;
	I2C1 ->CR2=0x08;//FREQ
	//I2C1->CR2 |=  ITERREN;//PRZERWANIA
	
	I2C1->CCR =0x28;
	I2C1->TRISE =0x09;
	
	//URUCHOMIENIE MAGISTRALI
	I2C1->CR1 |=(I2C_CR1_ACK | I2C_CR1_PE);
	
//	NVIC_EnableIRQ(I2C1_EV_IRQn);
//	NVIC_EnableIRQ(I2C1_ER_IRQn);
	
}
//Funkcja odczytu za pomoca I2C
void I2C_READ(int8_t address,int8_t reg_address,uint8_t *data,uint8_t length){
//	delay();
	I2C1->CR1|=I2C_CR1_ACK;
	//GPIOD->ODR|=LED_GREEN;
	uint32_t dum;
	I2C1->CR1 |=I2C_CR1_START;
	
	while(!(I2C1->SR1&I2C_SR1_SB));
	//delay();
	I2C1->DR=address;
//GPIOD->ODR |=LED_RED;
	while(!(I2C1->SR1&I2C_SR1_ADDR));
//		delay();
	dum =I2C1->SR2;
	//GPIOD->ODR|=LED_BLUE;
	while(!(I2C1->SR1&I2C_SR1_TXE));
	
//	delay();
	I2C1->DR=reg_address;
	//GPIOD->ODR|=LED_ORANGE;
		//sr2=I2C1->SR2;
	//sr1=I2C1->SR1;
	while(!(I2C1->SR1&I2C_SR1_BTF));//I2C->SR1&BTF ==1
	
	//delay();
	I2C1->CR1|=I2C_CR1_START;
	
	
	while(!(I2C1->SR1&I2C_SR1_SB));
	
//	delay();
	I2C1->DR=address|0x01;
	
	
	while(!(I2C1->SR1&I2C_SR1_ADDR));

	
	dum=I2C1->SR2;
//delay();
	while(length){

		
		if(length==1){
			
			I2C1->CR1 &=~ I2C_CR1_ACK;
		
		
			
		}
	//	delay();
		while(!(I2C1->SR1&I2C_SR1_RXNE));
		
		*(data++)=I2C1->DR;
		
	length=length-1;
	}
	
//delay();
	I2C1->CR1 |=I2C_CR1_STOP;
	
	
	while((I2C1->CR1&I2C_CR1_STOP)==I2C_CR1_STOP);
	
		
}
//Funkcja zapisu za pomoca I2C
void I2C_WRITE(uint8_t address,uint8_t *data,uint32_t length){

	//delay();
	
	uint32_t dum;
	
	
	
	I2C1->CR1 |= I2C_CR1_START;

	while(!(I2C1->SR1 & I2C_SR1_SB));		//CZEKANIE NA WYGENEROWANEI STARTU
  
	//delay();
	dum = I2C1->SR1;
	
	I2C1->DR = address;
	
	while(!(I2C1->SR1&I2C_SR1_ADDR));			//CZEKANIE NA WYSLANEI ADRES

	//delay();
	dum = I2C1->SR1;
	dum = I2C1->SR2;
	
	while(length){
		//delay();
			while(!(I2C1->SR1&I2C_SR1_TXE));	//CZEKANIE NA PUSTY DR
				I2C1->DR =*(data++);			//wyslanie 1 bajtu
			length=length-1;
		
			
	}
	//delay();
	while(((I2C1->SR1&I2C_SR1_TXE)==1)||((I2C1->SR1&I2C_SR1_BTF)==0));
	
	

//delay();
	I2C1->CR1|=I2C_CR1_STOP;		//wygenerowanie stopu;

	
	
	while((I2C1->CR1&I2C_CR1_STOP)==I2C_CR1_STOP);
	
	
}



void I2C3_INIT(void){
	//SCL PA8
	//SDA PC9
	
	
	//konfiguracja GPIO
	RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN|RCC_AHB1ENR_GPIOCEN);
	
	GPIOA->OTYPER |= (1<<8);			//OPEN DREN DAL pb 6 i 9
	GPIOC->OTYPER |= (1<<9);
	
	
	GPIOA->PUPDR |= (1<<17);
	GPIOC->PUPDR |= (1<<19);
	
	
	GPIOA->OSPEEDR |= ((1<<16)|(1<<17));
	GPIOC->OSPEEDR |= ((1<<18)|(1<<19));
	
	
	GPIOA->MODER |= (1<<17); 		//AF 4
	GPIOC->MODER |= (1<<19);
	
	GPIOA->AFR[1] |= (1<<2);							//I2C3
	GPIOC->AFR[1] |= (1<<6);							//I2C3
	
	  //odblokowanie zegara portu B
	
	
	RCC->APB1ENR |=RCC_APB1ENR_I2C3EN;
	//**********************KONFIGURACJA I2C1
	//I2C1->CR1 |= I2C_CR1_SWRST;
	//for(uint32_t i=0;i<0xFFFFF;i++);
	//I2C1->CR1 &=~I2C_CR1_SWRST;
	

	//I2C1->CR1=0;
	I2C3 ->CR2=0x08;//FREQ
	//I2C1->CR2 |=  ITERREN;//PRZERWANIA
	
	I2C3->CCR =0x28;
	I2C3->TRISE =0x09;
	
	//URUCHOMIENIE MAGISTRALI
	I2C3->CR1 |=(I2C_CR1_ACK | I2C_CR1_PE);
	
//	NVIC_EnableIRQ(I2C1_EV_IRQn);
//	NVIC_EnableIRQ(I2C1_ER_IRQn);
	
}
//Funkcja odczytu za pomoca I2C
void I2C3_READ(int8_t address,int8_t reg_address,uint8_t *data,uint8_t length){
//	delay();
	I2C3->CR1|=I2C_CR1_ACK;
	//GPIOD->ODR|=LED_GREEN;
	uint32_t dum;
	I2C3->CR1 |=I2C_CR1_START;
	
	while(!(I2C3->SR1&I2C_SR1_SB));
	//delay();
	I2C3->DR=address;
//GPIOD->ODR |=LED_RED;
	while(!(I2C3->SR1&I2C_SR1_ADDR));
//		delay();
	dum =I2C3->SR2;
	//GPIOD->ODR|=LED_BLUE;
	while(!(I2C3->SR1&I2C_SR1_TXE));
	
//	delay();
	I2C3->DR=reg_address;
	//GPIOD->ODR|=LED_ORANGE;
		//sr2=I2C1->SR2;
	//sr1=I2C1->SR1;
	while(!(I2C3->SR1&I2C_SR1_BTF));//I2C->SR1&BTF ==1
	
	//delay();
	I2C3->CR1|=I2C_CR1_START;
	
	
	while(!(I2C3->SR1&I2C_SR1_SB));
	
//	delay();
	I2C3->DR=address|0x01;
	
	
	while(!(I2C3->SR1&I2C_SR1_ADDR));

	
	dum=I2C3->SR2;
//delay();
	while(length){

		
		if(length==1){
			
			I2C3->CR1 &=~ I2C_CR1_ACK;
		
		
			
		}
	//	delay();
		while(!(I2C3->SR1&I2C_SR1_RXNE));
		
		*(data++)=I2C3->DR;
		
	length=length-1;
	}
	
//delay();
	I2C3->CR1 |=I2C_CR1_STOP;
	
	
	while((I2C3->CR1&I2C_CR1_STOP)==I2C_CR1_STOP);
	
		
}
//Funkcja zapisu za pomoca I2C
void I2C3_WRITE(uint8_t address,uint8_t *data,uint32_t length){

	//delay();
	
	uint32_t dum;
	
	
	
	I2C3->CR1 |= I2C_CR1_START;

	while(!(I2C3->SR1 & I2C_SR1_SB));		//CZEKANIE NA WYGENEROWANEI STARTU
  
	//delay();
	dum = I2C3->SR1;
	
	I2C3->DR = address;
	
	while(!(I2C3->SR1&I2C_SR1_ADDR));			//CZEKANIE NA WYSLANEI ADRES

	//delay();
	dum = I2C3->SR1;
	dum = I2C3->SR2;
	
	while(length){
		//delay();
			while(!(I2C3->SR1&I2C_SR1_TXE));	//CZEKANIE NA PUSTY DR
				I2C3->DR =*(data++);			//wyslanie 1 bajtu
			length=length-1;
		
			
	}
	//delay();
	while(((I2C3->SR1&I2C_SR1_TXE)==1)||((I2C3->SR1&I2C_SR1_BTF)==0));
	
	

//delay();
	I2C3->CR1|=I2C_CR1_STOP;		//wygenerowanie stopu;

	
	
	while((I2C3->CR1&I2C_CR1_STOP)==I2C_CR1_STOP);
	
	
}
