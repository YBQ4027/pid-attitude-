#ifndef _ATTITUDE_H_
#define _ATTITUDE_H_
#include "stdbool.h"
#include "config_parameter.h"
#include "pid.h"

//角度环积分限幅
#define PID_ANGLE_ROLL_INTEGRATION_LIMIT    100.0
#define PID_ANGLE_PITCH_INTEGRATION_LIMIT   100.0
#define PID_ANGLE_YAW_INTEGRATION_LIMIT     100.0

//角速度环积分限幅
#define PID_RATE_ROLL_INTEGRATION_LIMIT		100.0
#define PID_RATE_PITCH_INTEGRATION_LIMIT	100.0
#define PID_RATE_YAW_INTEGRATION_LIMIT		100.0

extern PID_structure pidAngleRoll;		//滚转角pid
extern PID_structure pidAnglePitch;		//俯仰角pid
extern PID_structure pidAngleYaw;		//偏航角pid

extern PID_structure pidRateRoll;		//滚转角速度pid
extern PID_structure pidRatePitch;		//俯仰角速度pid
extern PID_structure pidRateYaw;		//偏航角速度pid


void Init_attitudeControl(float anglePidDt,float ratePidDt);
void PID_attitude_Rate(Axis_3_f *actualRate,attitude_t *desiredRate,Control_signal *output);////////////////////////////////////
void PID_attitude_Angle(attitude_t *actualAngle,attitude_t *desiredAngle,attitude_t *outDesiredRate);
void Reset_attitude_AnglePID(void);
void Reset_attitude_RatePID(void);
void Write_PIDparameter_To_ConfigParam(void);



#endif	/* __ATTITUDE_H_ */
