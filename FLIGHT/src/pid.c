#include "pid.h"

//pid参数初始化
void Init_pid(PID_structure* pid,const float disired,const pidParam pidparam,const float dt)
{
	pid->error		 =	0;
	pid->preerror	 =	0;
	pid->disired	 =	disired;
	pid->KP			 =	pidparam.kp;
	pid->KI			 =	pidparam.ki;
	pid->KD			 =	pidparam.kd;
	pid->dt			 =	dt;
	pid->integLimit	 = 	DEFAULT_PID_INTEGRATION_LIMIT;
	pid->outputLimit = 	DEFAULT_PID_OUTPUT_LIMIT;
};

//pid更新
float Update_pid(PID_structure* pid,const float error)
{
	//float Output;
	pid->error	=	error;
	pid->integ	+=	pid->error * pid->dt;	//误差积分
	//积分限幅
	if (pid->integ > pid->integLimit)
	{
		pid->integ = pid->integLimit;
	}
	else if (pid->integ < -pid->integLimit)
	{
		pid->integ = -pid->integLimit;
	}
	pid->deriv	=	(pid->error - pid->preerror) / pid->dt;	//误差微分
	
	pid->Pout= pid->KP * pid->error;
	pid->Iout= pid->KI * pid->integ;
	pid->Dout= pid->KD * pid->deriv;
	
	pid->Output = pid->Pout + pid->Iout + pid->Dout;
	
	//输出限幅
	
	if(pid->outputLimit != 0)
	{
		if (pid->Output > pid->outputLimit)
		{
			pid->Output = pid->outputLimit;
		}
	else if (pid->Output < -pid->outputLimit)
		{
			pid->Output = -pid->outputLimit;
		}
	}
	
	pid->preerror = pid->error;
	return pid->Output;
	
}

//设置KP
void Set_pid_KP(PID_structure* pid,const float new_kp)
{
	pid->KP = new_kp;
}

//设置KI
void Set_pid_KI(PID_structure* pid,const float new_ki)
{
	pid->KI = new_ki;
}

//设置KD
void Set_pid_KD(PID_structure* pid,const float new_kd)
{
	pid->KD = new_kd;
}

//设置pid积分值上限
void Set_pid_integLimit(PID_structure* pid,const float new_integlimit)
{
	pid->integLimit = new_integlimit;
}

//设置pid输出上限
void Set_pid_outputLimit(PID_structure* pid,const float new_outputlimit)
{
	pid->outputLimit = new_outputlimit;
}

//设置pid期望值
void Set_pid_Disired(PID_structure* pid,const float new_disired)
{
	pid->disired = new_disired;
}

//设置pid时间步长
void Set_pid_dt(PID_structure* pid,const float new_dt)
{
	pid->dt = new_dt;
}

//读出偏差信号
float Get_pid_Error(PID_structure* pid)
{
	return pid->error;
}

//读出输出信号
float Get_pid_Output(PID_structure* pid)
{
	return pid->Output;
}

//复位
void Reset_pid(PID_structure* pid)
{
	pid->error		=	0;
	pid->preerror	=	0;
	pid->integ      =   0;
	pid->deriv      =   0;
}



