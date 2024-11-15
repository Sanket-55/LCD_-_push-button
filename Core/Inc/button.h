#include "stm32f2xx_hal.h"

#ifndef BUTTON_H
#define BUTTON_H

#define NUMBER_OF_BUTTON 4

enum GPIO_State
{
	Low,
	Debounce_Ceck,
	High,
	Iddle_High,

};

typedef struct
{
  	uint16_t gpio;
	enum GPIO_State gpio_state;
	char gpio_debounce_time;
	uint8_t debounce_count;


}button_types;

#define BUTTON_1 GPIO_PIN_2
#define BUTTON_2 GPIO_PIN_3
#define BUTTON_3 GPIO_PIN_4
#define BUTTON_4 GPIO_PIN_5

#define BUTTON_1_DEBOUNCETIME	5
#define BUTTON_2_DEBOUNCETIME	5
#define BUTTON_3_DEBOUNCETIME	5
#define BUTTON_4_DEBOUNCETIME	5
#define CONTINUS_TIME 100
#define UP_BUTTON 1
#define DOWN_BUTTON 2
#define ENTER_BUTTON 3
#define BACK_BUTTON 4

char ButtonStatus(button_types *Button);
uint8_t GetGpio(uint16_t gpio);
//uint8_t Cont_High_Countr=0;
extern button_types Buttons[NUMBER_OF_BUTTON];


#endif // BUTTON_H
