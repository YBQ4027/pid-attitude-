#ifndef _CONFIG_PARAMETER_H_
#define _CONFIG_PARAMETER_H_
#include "sys.h"
#include "stdbool.h"

#define PWM_CNT 2000 
#define POWER_MAX		5000

#define RATE_5_HZ		5
#define RATE_10_HZ		10
#define RATE_25_HZ		25
#define RATE_50_HZ		50
#define RATE_100_HZ		100
#define RATE_200_HZ 	200
#define RATE_250_HZ 	250
#define RATE_500_HZ 	500
#define RATE_1000_HZ 	1000

#define RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK % (MAIN_LOOP_RATE / RATE_HZ)) == 0)


#define MAIN_LOOP_RATE 			RATE_1000_HZ
#define MAIN_LOOP_DT			(u32)(1000/MAIN_LOOP_RATE)	/*单位ms*/

#define ATTITUDE_ESTIMAT_RATE	RATE_250_HZ	//姿态解算速率
#define ATTITUDE_ESTIMAT_DT		(1.0/RATE_250_HZ)


#define RATE_PID_RATE			RATE_500_HZ //角速度环（内环）PID速率
#define RATE_PID_DT				(1.0/RATE_500_HZ)

#define ANGEL_PID_RATE			ATTITUDE_ESTIMAT_RATE //角度环（外环）PID速率
#define ANGEL_PID_DT			(1.0/ATTITUDE_ESTIMAT_RATE)


	
typedef struct  	/*常数*/
{
	short magx_ca;
	short magy_ca;
	short magz_ca;
	float Gyro_K;
	float Acc_K;
}Constant;

typedef struct
{
	float KP;			//比例系数			set
	float KI;			//积分系数			set
	float KD;			//微分系数			set
	float integLimit;   //积分限幅			set
	float outputLimit;  //输出限幅			set
	float disired;		//期望信号			set
	float dt;           //时间步长			set
	float Pout;			//比例环节
	float Iout;			//积分环节
	float Dout;			//微分环节
	float error;		//偏差信号			get
	float preerror;		//先前的偏差信号
	float integ;		//积分值
	float deriv;		//微分值	
	float Output;		//输出信号			get
}PID_structure;	

//PID系数
typedef struct 
{
	float kp;
	float ki;
	float kd;
} pidParam;


//姿态角PID参数
typedef struct
{
	pidParam roll;
	pidParam pitch;	
	pidParam yaw;	
} pidParam_attitude;

//配置参数
typedef struct	
{
	pidParam_attitude 	pidAngle;		/*角度PID*/	
	pidParam_attitude 	pidRate;		/*角速度PID*/
	float 				Thrust;		/*油门基础值*/

} ConfigParam;

typedef struct  	/*姿态角*/
{

	float roll;
	float pitch;
	float yaw;
} attitude_t;

typedef struct  	/*三轴float*/
{

	float x;
	float y;
	float z;
}Axis_3_f;

typedef struct  	/*三轴short*/
{

	short x;
	short y;
	short z;
}Axis_3_s;


typedef struct  	/*imu*/
{

	Axis_3_s acc;
	Axis_3_s gyro;
	Axis_3_s mag;
}sensor_data;

typedef struct  	/*控制信号*/
{

	float roll;
	float pitch;
	float yaw;
	float thrust;
}Control_signal;

typedef struct
{
	attitude_t attitude_desire;		// 角度（deg）
	attitude_t attitudeRate_desire;	// 角速度(deg/s)
	float thrust;					//推力
}ParamDesired;						//期望值

typedef struct
{
	attitude_t 	attitude;
	Axis_3_f 	attitudeRate;
	//quaternion_t attitudeQuaternion;
	//point_t position;
	//velocity_t velocity;
	//acc_t acc;
} State;							//当前状态

typedef struct 
{
	u32 m1;
	u32 m2;
	u32 m3;
	u32 m4;
	
}motorPWM;

extern ConfigParam configParam;
extern Constant constant;
extern State flight_state;
extern short loop_num;		/*循环次数*/	

void Init_configParam(void);
#endif
