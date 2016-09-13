#include <stm32f4xx.h>
#include"GPIO_TIM_CLOCK.h"

//UStawienie maksymalnej czestotliwosci pracy 84MHz
void clockConfig(void){

RCC->CR |= 0x00000001;  //HSI oscillator on
      while(!(RCC->CR & 0x00000002)); //wait to HSI stabilition
	
 
     /**
      * reset main clock register
      */
	
      RCC->CR = 0x00000083; //reset all bits of rcc->cr register (reset value is in reference manuel)
      RCC->PLLCFGR = 0x24003010; //reset all bits of rcc->pllcfgr register (reset value is in reference manuel)
      RCC->CFGR = 0x00000000; //reset all bits of rcc->cfgr register (reset value is in reference manuel)
      RCC->CIR = 0x00000000; //reset all bits of rcc->cir register to disable interrupts (reset value is in reference manuel)
 
     /**
      * MCO clock source and prescale selection
      */
      RCC->CFGR &= ~0x00600000; //MCO1 reseted 
      RCC->CFGR |= 0x00600000; //MCO1 = PLL
      RCC->CFGR &= ~0xC0000000; //MCO2 resetted and MCO = SYSCLK
      RCC->CFGR &= ~0x07000000; //MCO1 prescaler resetted
      RCC->CFGR |= 0x06000000; //MCO1 prescaler = 4
      RCC->CFGR &= ~0x38000000; //MCO2 prescaler resetted
      RCC->CFGR |= 0x30000000; //MCO2 prescaler = 4
	
     /**
      * Clock setting
      */
      RCC->CR |= 0x00010000; //HSE oscillator on
      while(!(RCC->CR & 0x00020000)); //wait until HSE is ready		
			
     /**
      * PLL setting
      */		 
      RCC->PLLCFGR &= ~0x0000003F; //PLLM reset
      RCC->PLLCFGR |= 0x00000008; //PLLM = 8
      RCC->PLLCFGR &= ~0x00007FC0; //PLLN reset
      RCC->PLLCFGR |= 0x00005400; //PLLN = 336
      RCC->PLLCFGR &= ~0x00030000;//PLLP reset 
			
			RCC->PLLCFGR |= (1<<16); // PLLP = 4
																	
      RCC->PLLCFGR |= 0x00400000; //PLLSRCR = HSE
      RCC->PLLCFGR &= ~0x0F000000; // PLLQ reset
      RCC->PLLCFGR |= 0x040000000; //PLLQ = 4
      RCC->CR |= 0x01000000; // PLL on
      while(!(RCC->CR & 0x02000000))__NOP(); // wait until pll is ready
			 		 
     /** 
      * Flash latency
      */
      FLASH->ACR &= ~0x00000007; //flash wait state resetted and 
      FLASH->ACR |= 0x00000005; //wait state is 5
			
			
			 RCC->CFGR |= 0x00000002; //PLL selected as the system clock
      while(!(RCC->CFGR & 0x00000008))__NOP(); //wait until PLL used the system clock
	


}

void GPIO_LED_INIT(void){
	//ODBLOKOWANIE ZEGARA DLA PORTU D
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	//ustawienie moder
	
	uint8_t z;
	for(z=12;z<16;z++){
	GPIOD->MODER |= (1<<(z*2));
	}
}


void TIM2_INIT(void){
	//zegar timera....
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	
	
	//PRELOAD?
	//TIM_CR1_CEN odblokowuje timer

	
	TIM2->DIER |= TIM_DIER_UIE;
		
	TIM2->CNT =0xFFAFFFFF;
	
		TIM2->CR1 |= TIM_CR1_CEN;
	
		NVIC_EnableIRQ(TIM2_IRQn);
	
	
}

void TIM4_INIT_500HZ(void){
	//zegar timera....
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	
	
	//PRELOAD?
	//TIM_CR1_CEN odblokowuje timer

	
	TIM4->DIER |= TIM_DIER_UIE;
	TIM4->PSC =4;
		
	TIM4->CNT =0;
	
		TIM4->CR1 |= TIM_CR1_CEN;
	
		NVIC_EnableIRQ(TIM4_IRQn);
	
	

}


void GPIOD_TIM_INIT(void){
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	
	GPIOD->MODER |= GPIO_MODER_MODER6_0;
	
	
}
