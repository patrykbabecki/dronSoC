#include <stm32f4xx.h>
#include"USART.h"



//Funkcja inicjalizujaca USART2
void usart2_init(void){
	//wlasne falgi danych
	//flag_recived=0;
	
	//odblokowanie zegara portu A
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	//ustawienie GPIOC dla USART2
	GPIOA->MODER |= (GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1);	//alternative function
	
	GPIOA->PUPDR |= (GPIO_PUPDR_PUPDR2_0 | GPIO_PUPDR_PUPDR3_0); //pull up
	
	GPIOA->AFR[0] |= ((7<<8)|(7<<12)); //ALTERNATIVE FUNCTION USART6 for af7 pa2 pa3 0111
	
	//odblokowanie zegara usart2
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	
	//ustawienia usart2
	USART2->CR1 |= (USART_CR1_UE | USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_TE);
	
	
	USART2->BRR = SystemCoreClock/baud_usart2;
	
	//Odblokowanie przerwan USART6
	NVIC_EnableIRQ(USART2_IRQn);
	
}

//Funkcja Inijalizuja USART6
void usart6_init(void){
	//wlasne falgi danych
	
	
	//odblokowanie zegara portu C
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	//ustawienie GPIOC dla USART6
	GPIOC->MODER |= (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);	//alternative function
	
	GPIOC->PUPDR |= (GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR7_0); //pull up
	
	GPIOC->AFR[0] |= ((1<<31)|(1<<27)); //ALTERNATIVE FUNCTION USART6 for PC7 PC6
	
	//odblokowanie zegara usart6
	RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
	
	//ustawienia usart6
	USART6->CR1 |= (USART_CR1_UE | USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_TE);
	
	
	USART6->BRR = SystemCoreClock/baud_usart6;
	
	//Odblokowanie przerwan USART6
	NVIC_EnableIRQ(USART6_IRQn);
}


