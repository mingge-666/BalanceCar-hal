#include "gpio.h"               

uint8_t KeyNum = 0;
uint8_t Get_KeyState(void){
	
	if(HAL_GPIO_ReadPin(GPIOB,Key1_Pin)==0){
		return 1;
	}
	if(HAL_GPIO_ReadPin(GPIOB,Key2_Pin)==0){
		return 2;
	}
	if(HAL_GPIO_ReadPin(GPIOA,Key3_Pin)==0){
		return 3;
	}
	if(HAL_GPIO_ReadPin(GPIOA,Key4_Pin)==0){
		return 4;
	}
	return 0;
}

void Key_Tick(void){
	static uint8_t Count=0,Currstate,Prevstate;
	Count++;
	if(Count>=20){
		Count = 0;
		Prevstate = Currstate;
		Currstate = Get_KeyState();
		if(Currstate == 0&&Prevstate != 0){
			KeyNum = Prevstate;
		}
	}
}

uint8_t Get_KeyNum(void){
	uint8_t temp;
	temp = KeyNum;
	KeyNum = 0;
	return temp;
}
