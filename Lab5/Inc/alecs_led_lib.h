#ifndef ALECS_LED_LIB
#define ALECS_LED_LIB

#include "main.h"
#include  "stm32f0xx_hal.h"

void initiate_LEDs(){
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	GPIOC->MODER |= (GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 
									| GPIO_MODER_MODER9_0 | GPIO_MODER_MODER8_0);
}
char green_is_on(){
	if((GPIOC->ODR & GPIO_ODR_9) == 0)
		return 0;
	else
		return 1;
}

char blue_is_on(){
	if((GPIOC->ODR & GPIO_ODR_7) == 0)
		return 0;
	else
		return 1;
}

char orange_is_on(){
	if((GPIOC->ODR & GPIO_ODR_8) ==0)
		return 0;
	else
		return 1;
}

char red_is_on(){
	if((GPIOC->ODR & GPIO_ODR_6) ==0)
		return 0;
	else
		return 1;
}
void turn_on_red_LED(){
	GPIOC->BSRR |= GPIO_BSRR_BS_6;
}

void turn_on_green_LED(){
	GPIOC->BSRR |= GPIO_BSRR_BS_9;
}

void turn_on_blue_LED(){
	GPIOC->BSRR |= GPIO_BSRR_BS_7;
}

void turn_on_orange_LED(){
	GPIOC->BSRR |= GPIO_BSRR_BS_8;
}

void turn_off_orange_LED(){
	GPIOC->BSRR |= GPIO_BSRR_BR_8;
}

void turn_off_blue_LED(){
	GPIOC->BSRR |= GPIO_BSRR_BR_7;
}

void turn_off_green_LED(){
	GPIOC->BSRR |= GPIO_BSRR_BR_9;
}

void turn_off_red_LED(){
	GPIOC->BSRR |= GPIO_BSRR_BR_6;
}

void toggle_red(){
	if(red_is_on())
		turn_off_red_LED();
	else
		turn_on_red_LED();
}
void toggle_blue(){
	if(blue_is_on())
		turn_off_blue_LED();
	else
		turn_on_blue_LED();
}
void toggle_orange(){
	if(orange_is_on())
		turn_off_orange_LED();
	else
		turn_on_orange_LED();
}
void toggle_green(){
	if(green_is_on())
		turn_off_green_LED();
	else
		turn_on_green_LED();
}

void turn_off_all_LED(){
	turn_off_red_LED();
	turn_off_blue_LED();
	turn_off_green_LED();
	turn_off_orange_LED();
}

void turn_on_all_LED(){
	turn_on_red_LED();
	turn_on_blue_LED();
	turn_on_green_LED();
	turn_on_orange_LED();
}

#endif