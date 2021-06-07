#ifndef __CONTROL_H
#define __CONTROL_H

#include "filter.h"

extern unsigned int g_nMainEventCount;
extern unsigned int g_nGetPulseCount;
extern unsigned int g_nSpeedControlCount;
extern float g_fCarAngle;
extern unsigned int g_nLeftMotorPulse, g_nRightMotorPulse;
extern int g_nTargetSpeed;
extern int g_nLeftBias;
extern int g_nRightBias;
extern float g_fLeftMotorOut, g_fRightMotorOut;
extern int g_nSpeedControlPeriod;

extern long g_lLeftMotorPulseAction;
extern long g_lRightMotorPulseAction;




void GetMPUData(void);
void AngleCalculate(void);
void GetMotorPulse(void);
int SpeedInnerControl(int nPulse, int nTarget, int nPwm, int nErrorPrev);
void SetMotorVoltageAndDirection(int nLeftMotorPwm, int nRightMotorPwm);
void MotorOutput(void);
void AngleControl(void);
void SpeedControl(void);
void SpeedControlOutput(void);

#endif
