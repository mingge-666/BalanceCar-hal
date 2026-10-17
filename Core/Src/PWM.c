#include "main.h"

extern TIM_HandleTypeDef htim2;

void SetPWM(uint8_t n, int8_t speed){
	if(n==1){
		if(speed<0){
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,-speed);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
		}
		else if(speed>=0){
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,speed);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		}
	}
	
	else if(n==2){
		if(speed>0){
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,speed);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
		}
		else if(speed<=0){
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,-speed);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
		}
	}
}


