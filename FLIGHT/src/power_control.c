#include "power_control.h"
#include "math.h"

motorPWM motor_PWM;
motorPWM motor_PWMratio;

u16 limitThrust(int value)
{
	if(value > UINT16_MAX)
	{
		value = UINT16_MAX;
	}
	else if(value < 0)
	{
		value = 0;
	}

	return (u16)value;
}

void powerControl(Control_signal *control)	/*功率输出控制*/
{
	s16 r = control->roll;
	s16 p = control->pitch;
	
	motor_PWM.m1 = limitThrust(control->thrust + p + control->yaw );
	motor_PWM.m2 = limitThrust(control->thrust - r - control->yaw );
	motor_PWM.m3 = limitThrust(control->thrust - p + control->yaw );
	motor_PWM.m4 = limitThrust(control->thrust + r - control->yaw );
	
	Set_motors_Ratio(MOTOR_M1,motor_PWM.m1);
	Set_motors_Ratio(MOTOR_M2,motor_PWM.m2);
	Set_motors_Ratio(MOTOR_M3,motor_PWM.m3);
	Set_motors_Ratio(MOTOR_M4,motor_PWM.m4);
}

void Set_motors_Ratio(u32 id, u16 ithrust)
{
			//u16 ratio=ithrust;
			short pwmratio;
			float thrust = ((float)ithrust / POWER_MAX);
			thrust = thrust > 5.0f ? 5.0f : thrust;
			thrust = thrust < 0.2f ? 0.2f : thrust;//最大最小推力限制
			//float ratio = f(thrust);这一行是控制量与电机推力关系式
			ratio = ratio > 1.0f ? 1.0f : ratio;
			pwmratio =	(short)(ratio*(PWM_CNT-1));

	 	
		switch(id)
		{
			case 0:		/*MOTOR_M1*/
				TIM_SetCompare1(TIM2,pwmratio);
				motor_PWMratio.m1 = (float)pwmratio;
				break;
			case 1:		/*MOTOR_M2*/
				TIM_SetCompare2(TIM2,pwmratio);
				motor_PWMratio.m2 = (float)pwmratio;
				break;
			case 2:		/*MOTOR_M3*/
				TIM_SetCompare3(TIM2,pwmratio);
				motor_PWMratio.m3 = (float)pwmratio;
				break;
			case 3:		/*MOTOR_M4*/				
				TIM_SetCompare4(TIM2,pwmratio);
				motor_PWMratio.m4 = (float)pwmratio;
				break;
			default: break;
		}	
	}

