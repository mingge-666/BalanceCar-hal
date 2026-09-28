#include "main.h"
#include "Encoder.h"

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

volatile int16_t speed_left = 0;
volatile int16_t speed_right = 0;

int16_t Get_Speed(uint8_t n){
	int16_t Temp;
	if (n == 1)					//指定左电机
	{
		Temp = __HAL_TIM_GET_COUNTER(&htim3);
		__HAL_TIM_SET_COUNTER(&htim3, 0);
		speed_left = Temp;			//返回TIM3（左电机编码器）CNT增量值
	}
	else if (n == 2)			//指定右电机
	{
		Temp = __HAL_TIM_GET_COUNTER(&htim4);
		__HAL_TIM_SET_COUNTER(&htim4, 0);
		speed_right = Temp;			//返回TIM4（右电机编码器）CNT增量值
	}
	return Temp;
}

