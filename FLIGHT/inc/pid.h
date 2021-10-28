#ifndef _PID_H_
#define _PID_H_
#include "stdbool.h"
#include "config_parameter.h"

#define 	DEFAULT_PID_INTEGRATION_LIMIT  3000;	//默认pid积分值上限
#define  	DEFAULT_PID_OUTPUT_LIMIT       2000;	//默认pid输出上限，设为0时不设限

void Init_pid(PID_structure* pid,const float disired,const pidParam pidparam,const float dt);
float Update_pid(PID_structure* pid,const float error);
void Set_pid_KP(PID_structure* pid,const float new_kp);
void Set_pid_KI(PID_structure* pid,const float new_ki);
void Set_pid_KD(PID_structure* pid,const float new_kd);
void Set_pid_integLimit(PID_structure* pid,const float new_integlimit);	
void Set_pid_outputLimit(PID_structure* pid,const float new_outputlimit);
void Set_pid_Disired(PID_structure* pid,const float new_disired);
void Set_pid_dt(PID_structure* pid,const float new_dt);
float Get_pid_Error(PID_structure* pid);
float Get_pid_Output(PID_structure* pid);
void Reset_pid(PID_structure* pid);


#endif	/* __PID_H */
