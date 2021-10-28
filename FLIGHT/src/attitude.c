#include <stdbool.h>
#include "attitude.h"
#include "config_parameter.h"
#include "pid.h"
#include "usart1.h"
//姿态环pid，包括角度环和角速度环


PID_structure pidAngleRoll;		//滚转角
PID_structure pidAnglePitch;	//俯仰角
PID_structure pidAngleYaw;		//偏航角

PID_structure pidRateRoll;		//滚转角速度
PID_structure pidRatePitch;		//俯仰角速度
PID_structure pidRateYaw;		//偏航角速度

static inline int16_t pidOutLimit(float in)
{
	if (in > INT16_MAX)
		return INT16_MAX;
	else if (in < -INT16_MAX)
		return -INT16_MAX;
	else
		return (int16_t)in;
}

/*初始化姿态角pid*/
void Init_attitudeControl(float anglePidDt,float ratePidDt)	
{
	/*初始化pid结构*/
	Init_pid(&pidAngleRoll,0,configParam.pidAngle.roll,anglePidDt);
	Init_pid(&pidAnglePitch,0,configParam.pidAngle.pitch,anglePidDt);
	Init_pid(&pidAngleYaw,0,configParam.pidAngle.yaw,anglePidDt);
	
	Init_pid(&pidRateRoll,0,configParam.pidRate.roll,ratePidDt);
	Init_pid(&pidRatePitch,0,configParam.pidRate.pitch,ratePidDt);
	Init_pid(&pidRateYaw,0,configParam.pidRate.yaw,ratePidDt);
	
	/*初始化pid限幅*/
	Set_pid_integLimit(&pidAngleRoll,PID_ANGLE_ROLL_INTEGRATION_LIMIT);
	Set_pid_integLimit(&pidAnglePitch,PID_ANGLE_PITCH_INTEGRATION_LIMIT);
	Set_pid_integLimit(&pidAngleYaw,PID_ANGLE_YAW_INTEGRATION_LIMIT);
	
	Set_pid_integLimit(&pidRateRoll, PID_RATE_ROLL_INTEGRATION_LIMIT);
	Set_pid_integLimit(&pidRatePitch,PID_RATE_PITCH_INTEGRATION_LIMIT);
	Set_pid_integLimit(&pidRateYaw,PID_RATE_YAW_INTEGRATION_LIMIT);
}

//bool attitudeControlTest()	/*姿态角控制测试*/

void PID_attitude_Rate(Axis_3_f *actualRate,attitude_t *desiredRate,Control_signal *output)	/* 角速度环PID */
{
	output->roll = pidOutLimit(Update_pid(&pidRateRoll, desiredRate->roll - actualRate->x));
	output->pitch = pidOutLimit(Update_pid(&pidRatePitch, desiredRate->pitch - actualRate->y));
	output->yaw = pidOutLimit(Update_pid(&pidRateYaw, desiredRate->yaw - actualRate->z));
}

void PID_attitude_Angle(attitude_t *actualAngle,attitude_t *desiredAngle,attitude_t *outDesiredRate)	/* 角度环PID */
{

	outDesiredRate->roll = Update_pid(&pidAngleRoll, desiredAngle->roll - actualAngle->roll);
	outDesiredRate->pitch = Update_pid(&pidAnglePitch, desiredAngle->pitch - actualAngle->pitch);

	float yawError = desiredAngle->yaw - actualAngle->yaw ;
	if (yawError > 180.0f) 		/*偏航角：-180度~180度*/
		yawError -= 360.0f;
	else if (yawError < -180.0) 
		yawError += 360.0f;
	outDesiredRate->yaw = Update_pid(&pidAngleYaw, yawError);
}


void Reset_attitude_AnglePID(void)	/*复位姿态角PID*/
{
	Reset_pid(&pidAngleRoll);
	Reset_pid(&pidAnglePitch);
	Reset_pid(&pidAngleYaw);
}

void Reset_attitude_RatePID(void)	/*复位姿态角速度PID*/
{
	Reset_pid(&pidRateRoll);
	Reset_pid(&pidRatePitch);
	Reset_pid(&pidRateYaw);
}

void Write_PIDparameter_To_ConfigParam(void)	/*记录PID参数到configParam*/
{
	configParam.pidAngle.roll.kp = pidAngleRoll.KP;
	configParam.pidAngle.roll.ki = pidAngleRoll.KI;
	configParam.pidAngle.roll.kd = pidAngleRoll.KD;
	
	configParam.pidAngle.pitch.kp = pidAnglePitch.KP;
	configParam.pidAngle.pitch.ki = pidAnglePitch.KI;
	configParam.pidAngle.pitch.kd = pidAnglePitch.KD;
	
	configParam.pidAngle.yaw.kp = pidAngleYaw.KP;
	configParam.pidAngle.yaw.ki = pidAngleYaw.KI;
	configParam.pidAngle.yaw.kd = pidAngleYaw.KD;
	
	configParam.pidRate.roll.kp = pidRateRoll.KP;
	configParam.pidRate.roll.ki = pidRateRoll.KI;
	configParam.pidRate.roll.kd = pidRateRoll.KD;
	
	configParam.pidRate.pitch.kp = pidRatePitch.KP;
	configParam.pidRate.pitch.ki = pidRatePitch.KI;
	configParam.pidRate.pitch.kd = pidRatePitch.KD;
	
	configParam.pidRate.yaw.kp = pidRateYaw.KP;
	configParam.pidRate.yaw.ki = pidRateYaw.KI;
	configParam.pidRate.yaw.kd = pidRateYaw.KD;
	
}
