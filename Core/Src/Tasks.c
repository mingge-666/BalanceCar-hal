#include "PWM.h"
#include "PID.h"
#include "MPU6050.h"
#include "math.h"
#include "Encoder.h"
#include "OLED.h"
#include "main.h"
#include "BlueSerial.h"
#include <string.h>
#include <stdlib.h>

void Stop_Task(void);
extern uint8_t RunState;
extern TIM_HandleTypeDef htim2;

PID_t AnglePID = {
	.Kp = 5,
	.Ki = 0.1,
	.Kd = 5,
	
	.OutMax = 80,
	.OutMin = -80,
	.ErrorIntMax = 350,	
	.ErrorIntMin = -350,
	.offset = 3.5,
};

PID_t SpeedPID = {
	.Kp = 2,
	.Ki = 0.06,
	.Kd = 0,
	.ErrorIntMax = 150,
	.ErrorIntMin = -150,
	.OutMax = 25,
	.OutMin = -25,
};

PID_t TurnPID = {
	.Kp = 4,
	.Ki = 3,
	.Kd = 0,
	
	.OutMax = 50,
	.OutMin = -50,
	.ErrorIntMax = 30,
	.ErrorIntMin = -30,
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
	if(AnglePID.Out>=15){
		OLED_Printf(0, 0, OLED_6X8, "1");	
	}
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
	if(SpeedPID.Out>=15){
		OLED_Printf(0, 0, OLED_6X8, "2");	
	}
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
	
	OLED_Printf(12, 0, OLED_6X8, "Angle");						
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

void BlueTooth_Control(void){
			if (BlueSerial_RxFlag == 1)		//如果收到数据包
		{
			char *Tag = strtok(BlueSerial_RxPacket, ",");	//提取数据1，定义为标签Tag
			if (strcmp(Tag, "key") == 0)					//Tag为key，收到按键数据包
			{
				char *Name = strtok(NULL, ",");				//提取数据2，定义为按键名称
				char *Action = strtok(NULL, ",");			//提取数据3，定义为按键动作
				
				/*此处可执行按键操作，目前程序暂时没用到按键*/
			}
			else if (strcmp(Tag, "slider") == 0)			//Tag为slider，收到滑杆数据包
			{
				char *Name = strtok(NULL, ",");				//提取数据2，定义为滑杆名称
				char *Value = strtok(NULL, ",");			//提取数据3，定义为滑杆值
				
				/*执行滑杆操作*/
				if (strcmp(Name, "AngleKp") == 0)			//如果滑杆名称是AngleKp
				{
					AnglePID.Kp = atof(Value);				//则把滑杆值赋值给角度环Kp
				}
				else if (strcmp(Name, "AngleKi") == 0)		//如果滑杆名称是AngleKi
				{
					AnglePID.Ki = atof(Value);				//则把滑杆值赋值给角度环Ki
				}
				else if (strcmp(Name, "AngleKd") == 0)		//如果滑杆名称是AngleKd
				{
					AnglePID.Kd = atof(Value);				//则把滑杆值赋值给角度环Kd
				}
				else if (strcmp(Name, "SpeedKp") == 0)		//如果滑杆名称是SpeedKp
				{
					SpeedPID.Kp = atof(Value);				//则把滑杆值赋值给速度环Kp
				}
				else if (strcmp(Name, "SpeedKi") == 0)		//如果滑杆名称是SpeedKi
				{
					SpeedPID.Ki = atof(Value);				//则把滑杆值赋值给速度环Ki
				}
				else if (strcmp(Name, "SpeedKd") == 0)		//如果滑杆名称是SpeedKd
				{
					SpeedPID.Kd = atof(Value);				//则把滑杆值赋值给速度环Kd
				}
				else if (strcmp(Name, "TurnKp") == 0)		//如果滑杆名称是TurnKp
				{
					TurnPID.Kp = atof(Value);				//则把滑杆值赋值给转向环Kp
				}
				else if (strcmp(Name, "TurnKi") == 0)		//如果滑杆名称是TurnKi
				{
					TurnPID.Ki = atof(Value);				//则把滑杆值赋值给转向环Ki
				}
				else if (strcmp(Name, "TurnKd") == 0)		//如果滑杆名称是TurnKd
				{
					TurnPID.Kd = atof(Value);				//则把滑杆值赋值给转向环Kd
				}
				else if (strcmp(Name, "Offset") == 0)		//如果滑杆名称是Offset
				{
					AnglePID.offset = atof(Value);		//则把滑杆值赋值给OutOffset
				}
			}
			else if (strcmp(Tag, "joystick") == 0)			
			{
				int8_t LH = atoi(strtok(NULL, ","));		
				int8_t LV = atoi(strtok(NULL, ","));	
				int8_t RH = atoi(strtok(NULL, ","));		
				int8_t RV = atoi(strtok(NULL, ","));	
				
				/*执行摇杆操作*/
				SpeedPID.Target = LV / 25.0;	
				TurnPID.Target = RH / 25.0;	
			}
			
			BlueSerial_RxFlag = 0;	
		}
}


