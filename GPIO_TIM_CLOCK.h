#ifndef GPIO_TIM_CLOCK
#define GPIO_TIM_CLOCK

#define PD6 (1<<6)
#define PD6_SW GPIOD->ODR^=PD6

#define LED_GREEN (1<<12)
#define LED_ORANGE	(1<<13)
#define LED_RED			(1<<14)
#define LED_BLUE		(1<<15)

void clockConfig(void);
void GPIO_LED_INIT(void);
void TIM2_INIT(void);
void TIM4_INIT_500HZ(void);
void GPIOD_TIM_INIT(void);

#endif
