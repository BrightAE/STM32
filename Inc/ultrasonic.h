#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

extern int Distance;
extern int UltraError;
extern unsigned int TIM1CH4_CAPTURE_STA;
extern unsigned int TIM1CH4_CAPTURE_VAL;

void ReadDistance(void);
char InfraredDetect(void);
void UltraSelfCheck(void);
int IsUltraOK(void);

#endif
