#ifndef __POWER_CONTROL_H
#define __POWER_CONTROL_H
#include "config_parameter.h"

#define MOTOR_M1  		0
#define MOTOR_M2  		1
#define MOTOR_M3  		2
#define MOTOR_M4  		3


extern motorPWM motor_PWM;
extern motorPWM motor_PWMratio;
u16 limitThrust(int value);
void powerControl(Control_signal *control);
void Set_motors_Ratio(u32 id, u16 ithrust);

#endif /*__POWER_CONTROL_H */
