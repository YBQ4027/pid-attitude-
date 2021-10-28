#include "config_parameter.h"
#include "attitude.h"
short loop_num;
ConfigParam configParam;
Constant constant;
State flight_state;
static ConfigParam Defult_ConfigParam=
{
	.Thrust=3400,
	.pidAngle=	
	{	
		.roll=
		{
			.kp=0.5,
			.ki=0.0,
			.kd=0.0,
		},
		.pitch=
		{
			.kp=0.5,
			.ki=0.0,
			.kd=0.0,
		},
		.yaw=
		{
			.kp=0.05,
			.ki=0.0,
			.kd=0,
		},
	},	
	.pidRate=	
	{	
		.roll=
		{
			.kp=300.0,
			.ki=0.0,
			.kd=6.5,
		},
		.pitch=
		{
			.kp=300.0,
			.ki=0.0,
			.kd=6.5,
		},
		.yaw=
		{
			.kp=200.0,
			.ki=18.5,
			.kd=0.0,
		},
		
	}
};	





void Init_configParam(void)
{
	pidAngleRoll.KP=Defult_ConfigParam.pidAngle.roll.kp;
	pidAngleRoll.KI=Defult_ConfigParam.pidAngle.roll.ki;
	pidAngleRoll.KD=Defult_ConfigParam.pidAngle.roll.kd;
	
	pidAnglePitch.KP=Defult_ConfigParam.pidAngle.pitch.kp;
	pidAnglePitch.KI=Defult_ConfigParam.pidAngle.pitch.ki;
	pidAnglePitch.KD=Defult_ConfigParam.pidAngle.pitch.kd;
	
	pidAngleYaw.KP=Defult_ConfigParam.pidAngle.yaw.kp;
	pidAngleYaw.KI=Defult_ConfigParam.pidAngle.yaw.ki;
	pidAngleYaw.KD=Defult_ConfigParam.pidAngle.yaw.kd;
	
	pidRateRoll.KP=Defult_ConfigParam.pidRate.roll.kp;
	pidRateRoll.KI=Defult_ConfigParam.pidRate.roll.ki;
	pidRateRoll.KD=Defult_ConfigParam.pidRate.roll.kd;
	
	pidRatePitch.KP=Defult_ConfigParam.pidRate.pitch.kp;
	pidRatePitch.KI=Defult_ConfigParam.pidRate.pitch.ki;
	pidRatePitch.KD=Defult_ConfigParam.pidRate.pitch.kd;
	
	pidRateYaw.KP=Defult_ConfigParam.pidRate.yaw.kp;
	pidRateYaw.KI=Defult_ConfigParam.pidRate.yaw.ki;
	pidRateYaw.KD=Defult_ConfigParam.pidRate.yaw.kd;
}

