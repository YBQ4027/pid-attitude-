#include <math.h>
#include "state_control.h"
#include "attitude.h"
#include "config_parameter.h"
#include "maths.h"
#include "flight_control.h"

//static float Thrust;
static attitude_t attitudeDesired;
static attitude_t rateDesired;

void stateControlInit(void)
{
	Init_attitudeControl(ANGEL_PID_DT,RATE_PID_DT);/*初始化*/
}

//bool stateControlTest(void)  ParamDisired   Control_signal   sensorData_t *sensors

void StateControl(Control_signal *control,State *state,ParamDesired *paramdisired,float actualThrust)
{
		attitudeDesired.yaw = paramdisired->attitude_desire.yaw;//-------------------until
	
			if(attitudeDesired.yaw > 180.0f) 
				attitudeDesired.yaw -= 360.0f;
			if(attitudeDesired.yaw < -180.0f) 
				attitudeDesired.yaw += 360.0f;
		//attitudeDesired.yaw += setpoint->attitude.yaw/ANGEL_PID_RATE; /*期望YAW 速率模式*/	
			
		attitudeDesired.roll = paramdisired->attitude_desire.roll;
		attitudeDesired.pitch = paramdisired->attitude_desire.pitch;	
			
		//attitudeDesired.roll += configParam.trimR;	/*叠加微调值*/
		//attitudeDesired.pitch += configParam.trimP;		
		
		PID_attitude_Angle(&state->attitude, &attitudeDesired, &rateDesired);
		PID_attitude_Rate(&state->attitudeRate,&rateDesired,control);	/*输出期望角加速度*/
			
		//paramdisired->thrust尚未设置
			
		//Thrust = constrainf(actualThrust, 0, 32768);	/*油门限幅*/
		
		control->thrust = actualThrust;
		control->thrust = constrainf(actualThrust, 0, POWER_MAX);	/*油门限幅*/
}
