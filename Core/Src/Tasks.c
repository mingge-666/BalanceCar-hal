#include "PWM.h"
#include "PID.h"
#include "MPU6050.h"
#include "math.h"
#include "Encoder.h"
#include "OLED.h"
#include "main.h"

void Stop_Task(void);
extern uint8_t RunState;
extern TIM_HandleTypeDef htim2;

PID_t AnglePID = {
	.Kp = 5,
	.Ki = 0.1,
	.Kd = 5,
	
	.OutMax = 80,
	.OutMin = -80,
	.ErrorIntMax = 150,	
	.ErrorIntMin = -150,
	.offset = 3.5,
};

PID_t SpeedPID = {
	.Kp = 2.5,
	.Ki = 0.07,
	.Kd = 0.05,
	.ErrorIntMax = 150,
	.ErrorIntMin = -150,
	.OutMax = 40,
	.OutMin = -40,
};

PID_t TurnPID = {
	.Kp = 4,
	.Ki = 3,
	.Kd = 0,
	
	.OutMax = 50,
	.OutMin = -50,
	.ErrorIntMax = 20,
	.ErrorIntMin = -20,
};

int16_t GX,GY,GZ,AX,AY,AZ;
int16_t AvePWM,DifPWM=0,LeftPWM,RightPWM;

float LeftSpeed, RightSpeed;
float AveSpeed, DifSpeed;

float Angle_Acc;
float Angle_Gyro;
float Angle;

void Get_Data(void){
	MPU6050_GetData(&AX,&AY,&AZ,&GX,&GY,&GZ);
		
	GY += 24;
	AX+=30;

	Angle_Acc = -atan2(AX, AZ) / 3.14159 * 180;
	Angle_Acc += 0.3;
	Angle_Gyro = Angle+GY/32768.0 * 2000 * 0.01;
			
	float Alpha = 0.01;
	Angle = Alpha*Angle_Acc + (1-Alpha)*Angle_Gyro;
	
	if(Angle >=45||Angle<=-45){
		RunState = 0;
	}
}

void Angle_Task(void){
	if(RunState){
		AnglePID.Actual = Angle;
		PID_Update(&AnglePID);
		AvePWM = -AnglePID.Out;
				
		LeftPWM = AvePWM+DifPWM/2;
		RightPWM = AvePWM - DifPWM/2;
				
		if (LeftPWM > 100) {LeftPWM = 100;} else if (LeftPWM < -100) {LeftPWM = -100;}
		if (RightPWM > 100) {RightPWM = 100;} else if (RightPWM < -100) {RightPWM = -100;}
		SetPWM(1,LeftPWM);
		SetPWM(2,RightPWM);
	}
	else{
		Stop_Task();
	}
}

void Position_Task(void){
	if(RunState){
		LeftSpeed = Get_Speed(1)/ 44.0 / 0.05 / 9.27666;
		RightSpeed = -Get_Speed(2)/ 44.0 / 0.05 / 9.27666;
		DifSpeed = LeftSpeed - RightSpeed;
		AveSpeed = (LeftSpeed + RightSpeed)/2;
		
		SpeedPID.Actual = AveSpeed;//速度环调控
		PID_Update(&SpeedPID);
		AnglePID.Target = SpeedPID.Out;
				
		TurnPID.Actual = DifSpeed;//角度环调控
		PID_Update(&TurnPID);
		DifPWM = TurnPID.Out;
	}
}

void OLED_Task(void){
	OLED_Clear();
	//X:0-88列；Y：0-48行；每行相差8
	
	OLED_Printf(0, 0, OLED_6X8, "  Angle");						
	OLED_Printf(0, 8, OLED_6X8, "P:%05.2f", AnglePID.Error0);		
	OLED_Printf(0, 16, OLED_6X8, "I:%05.2f", AnglePID.ErrorInt);		
	OLED_Printf(0, 24, OLED_6X8, "D:%05.2f", (AnglePID.Actual-AnglePID.Actual1));	
	OLED_Printf(0, 32, OLED_6X8, "T:%+05.1f", AnglePID.Target);
	OLED_Printf(0, 40, OLED_6X8, "A:%+05.1f", Angle);			
	OLED_Printf(0, 48, OLED_6X8, "O:%+05.1f", AnglePID.Out);

	OLED_Printf(0, 56, OLED_6X8, "GY:%+05d", GY);				
	OLED_Printf(56, 56, OLED_6X8, "offset:%02.1f", AnglePID.offset);

		//速度环
	OLED_Printf(50, 0, OLED_6X8, "Speed");	
	OLED_Printf(50, 8, OLED_6X8, "%05.2f", SpeedPID.Error0);		
	OLED_Printf(50, 16, OLED_6X8, "%05.2f", SpeedPID.ErrorInt);		
	OLED_Printf(50, 24, OLED_6X8, "%05.2f", (SpeedPID.Actual-SpeedPID.Actual1));		
	OLED_Printf(50, 32, OLED_6X8, "%+05.1f", SpeedPID.Target);	
	OLED_Printf(50, 40, OLED_6X8, "%+05.1f", AveSpeed);			
	OLED_Printf(50, 48, OLED_6X8, "%+05.1f", SpeedPID.Out);	
		
		//转向环
	OLED_Printf(88, 0, OLED_6X8, "Turn");	
	OLED_Printf(88, 8, OLED_6X8, "%05.2f", TurnPID.Error0);			
	OLED_Printf(88, 16, OLED_6X8, "%05.2f", TurnPID.ErrorInt);		
	OLED_Printf(88, 24, OLED_6X8, "%05.2f", (TurnPID.Actual - TurnPID.Actual1));		
	OLED_Printf(88, 32, OLED_6X8, "%+05.1f", TurnPID.Target);	
	OLED_Printf(88, 40, OLED_6X8, "%+05.1f", DifSpeed);			
	OLED_Printf(88, 48, OLED_6X8, "%+05.1f", TurnPID.Out);	

	OLED_Update();
}

void Stop_Task(void){
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,0);
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
}


