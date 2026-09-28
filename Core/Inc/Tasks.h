#ifndef __TASKS_H
#define __TASKS_H

#include "PID.h"
extern PID_t AnglePID;
extern PID_t SpeedPID;
extern PID_t TurnPID;
void Angle_Task(void);
void Position_Task(void);
void Get_Data(void);
void OLED_Task(void);
void Stop_Task(void);

#endif


