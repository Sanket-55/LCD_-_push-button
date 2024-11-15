#include "button.h"

button_types Buttons[NUMBER_OF_BUTTON] = {
    {BUTTON_1, Low, BUTTON_1_DEBOUNCETIME, 0},
    {BUTTON_2, Low, BUTTON_2_DEBOUNCETIME, 0},
    {BUTTON_3, Low, BUTTON_3_DEBOUNCETIME, 0},
    {BUTTON_4, Low, BUTTON_4_DEBOUNCETIME, 0}
};
char ButtonStatus(button_types *Button)
{
	uint8_t gpio_state;

	gpio_state = GetGpio(Button->gpio);

	if(gpio_state == GPIO_PIN_RESET)
	{
		Button->gpio_state = Low;
		Button->debounce_count = 0;

	}
	else if(gpio_state == GPIO_PIN_SET && (Button->gpio_state == High  ))
	{
		Button->gpio_state = Iddle_High;
	}
	else if(!(gpio_state == GPIO_PIN_SET && (Button->gpio_state == Iddle_High)))
	{
		if(gpio_state == GPIO_PIN_SET && (Button->gpio_state == Debounce_Ceck) && ((Button->debounce_count) < (Button->gpio_debounce_time)))
		{
		 	Button->debounce_count++;
		}
		else if(gpio_state == GPIO_PIN_SET && (Button->gpio_state == Low))
		{
			Button->gpio_state = Debounce_Ceck;
			Button->debounce_count++;
		}
		else
		{
			Button->gpio_state = High;
		}
	}

	else{
//		if(Cont_High_Countr < CONTINUS_TIME)
//			Cont_High_Countr++;
//		else
//		{
//			Cont_High_Countr = 0;
//			Button->gpio_state = High;
//		}
}/*this state will be reached when Button.gpio_state*/


}
