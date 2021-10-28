#include "config_parameter.h"
#include "sensorfusion_mahony.h"
#include "pid.h"
#include "math.h"
#include "sensor.h"
#include "key.h"

#define Ki 0.005f
#define Kp 5.0f
#define halfT 0.00121f  //半个采样周期时长，单位s      还需要调整halfT


//分为九轴和六轴


//陀螺仪、加速度计、磁力计数据融合出姿态四元数
float q0=1.0f, q1=0.0f, q2=0.0f, q3=0.0f;//四元数
static float exInt=0, eyInt=0, ezInt=0;

 void Update_AHRS_mahony(sensor_data* sensordata)
{
	
	float norm;
//	float prepitch,preyaw,preroll;
	float ax,ay,az,gx,gy,gz,mx,my,mz;
    float hx, hy, hz, bx, bz;
	float vx, vy, vz, wx, wy, wz; 
	//v*当前姿态计算得来的重力在机载坐标下三轴上的分量
	float ex, ey, ez;
    //先计算好相关乘积项为了后面做矩阵运算做准备
	float q0q0=q0*q0;
	float q0q1=q0*q1;
	float q0q2=q0*q2;
	float q0q3=q0*q3;
	float q1q1=q1*q1;
	float q1q2=q1*q2;
	float q1q3=q1*q3;
	float q2q2=q2*q2;
	float q2q3=q2*q3;
	float q3q3=q3*q3;
		
	ax = (float)sensordata->acc.x;
	ay = (float)sensordata->acc.y;
	az = (float)sensordata->acc.z;
			
	//将陀螺仪AD值转换为 弧度/s
	gx = (float)sensordata->gyro.x*constant.Gyro_K/1000.0f;
	gy = (float)sensordata->gyro.y*constant.Gyro_K/1000.0f;
	gz = (float)sensordata->gyro.z*constant.Gyro_K/1000.0f;

	mx = (float)sensordata->mag.x;
	my =(float)sensordata->mag.y;
	mz = (float)sensordata->mag.z;
           
    //加速度计和地磁计向量标准化
    norm = sqrt(ax*ax+ay*ay+az*az); 
    ax = ax / norm;
    ay = ay / norm;
    az = az / norm;
    norm = sqrt(mx*mx+my*my+mz*mz); 
    mx = mx / norm;
    my = my / norm;
    mz = mz / norm;
           
    //计算得到地磁计在理论地磁坐标系下的机体上三个轴的分量
    hx = 2*mx*(0.5f-q2q2-q3q3) + 2*my*(q1q2-q0q3) + 2*mz*(q1q3+q0q2);
    hy = 2*mx*(q1q2+q0q3) + 2*my*(0.5f-q1q1-q3q3) + 2*mz*(q2q3-q0q1);
    hz = 2*mx*(q1q3-q0q2) + 2*my*(q2q3+q0q1) + 2*mz*(0.5f-q1q1-q2q2);   
            
    //当罗盘水平旋转的时候，航向角在0-360之间变化
    bx = sqrt((hx*hx) + (hy*hy));
    bz = hz; 
           
    //用四元数表示的方向余弦矩阵解算出理论载体坐标b系下的三轴加速度大小
    //加速度计重力向量转换到b系
    vx = 2*(q1*q3 - q0*q2);
    vy = 2*(q0*q1 + q2*q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
	
    //地磁计在n系下磁向量转换到b系下，反向使用DCM得到
    wx = 2*bx*(0.5f-q2q2-q3q3) + 2*bz*(q1q3-q0q2);
    wy = 2*bx*(q1q2-q0q3) + 2*bz*(q0q1+q2q3);
    wz = 2*bx*(q0q2+q1q3) + 2*bz*(0.5f-q1q1-q2q2);  
           
    //加速计补偿和磁力计补偿
    //叉积运算
    ex = (ay*vz-az*vy) + (my*wz-mz*wy);
    ey = (az*vx-ax*vz) + (mz*wx-mx*wz);
    ez = (ax*vy-ay*vx) + (mx*wy-my*wx);
            
    exInt = exInt + ex*Ki;
    eyInt = eyInt + ey*Ki;
    ezInt = ezInt + ez*Ki;
    //将误差PI后补偿到陀螺仪
    gx = gx + Kp*ex+exInt;
    gy = gy + Kp*ey+eyInt;
    gz = gz + Kp*ez+ezInt;
	
    flight_state.attitudeRate.x= gx ;
	flight_state.attitudeRate.y= gy ;
	flight_state.attitudeRate.z= gz ;
	
	
    //一阶龙格库塔法更新四元数
    q0 = q0 + (-q1*gx - q2*gy - q3*gz) * halfT;
    q1 = q1 + (q0*gx + q2*gz - q3*gy) * halfT;
    q2 = q2 + (q0*gy - q1*gz + q3*gx) * halfT;
    q3 = q3 + (q0*gz + q1*gy - q2*gx) * halfT;  
	
	//规范化Pitch、Roll轴四元数
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 / norm;
	q1 = q1 / norm;
    q2 = q2 / norm;
	q3 = q3 / norm;
	
//	prepitch = flight_state.attitude.pitch;
//	preroll = flight_state.attitude.roll;
//	preyaw = flight_state.attitude.yaw;
	//计算得到俯仰角/横滚角/航向角
	flight_state.attitude.pitch = asin(-2*q1*q3 + 2*q0*q2)*57.3;								// pitch
	flight_state.attitude.roll = -atan2(2*q2*q3 + 2*q0*q1, - 2*q1*q1 - 2*q2*q2 + 1)*57.3;			// roll
	flight_state.attitude.yaw = -((atan2(2*(q1*q2 + q0*q3),q0*q0 + q1*q1 - q2*q2 - q3*q3)*57.3));	//yaw
	
}

 void Update_IMU_mahony(sensor_data* sensordata)
{
	
	float norm;
//	float prepitch,preyaw,preroll;
	float ax,ay,az,gx,gy,gz;
	float vx, vy, vz; 
	//v*当前姿态计算得来的重力在机载坐标下三轴上的分量
	float ex, ey, ez;
    //先计算好相关乘积项为了后面做矩阵运算做准备
	float q0q0=q0*q0;
	float q0q1=q0*q1;
	float q0q2=q0*q2;
	float q0q3=q0*q3;
	float q1q1=q1*q1;
	float q1q2=q1*q2;
	float q1q3=q1*q3;
	float q2q2=q2*q2;
	float q2q3=q2*q3;
	float q3q3=q3*q3;
		
	ax = (float)sensordata->acc.x;
	ay = (float)sensordata->acc.y;
	az = (float)sensordata->acc.z;
			
	//将陀螺仪AD值转换为 弧度/s
	gx = (float)sensordata->gyro.x*constant.Gyro_K/1000.0f;
	gy = (float)sensordata->gyro.y*constant.Gyro_K/1000.0f;
	gz = (float)sensordata->gyro.z*constant.Gyro_K/1000.0f;

           
    //加速度计和地磁计向量标准化
    norm = sqrt(ax*ax+ay*ay+az*az); 
    ax = ax / norm;
    ay = ay / norm;
    az = az / norm;
           
           
    //用四元数表示的方向余弦矩阵解算出理论载体坐标b系下的三轴加速度大小
    //加速度计重力向量转换到b系
    vx = 2*(q1*q3 - q0*q2);
    vy = 2*(q0*q1 + q2*q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
           
    //加速计补偿和磁力计补偿
    //叉积运算
    ex = (ay*vz-az*vy);
    ey = (az*vx-ax*vz);
    ez = (ax*vy-ay*vx);
            
    exInt = exInt + ex*Ki;
    eyInt = eyInt + ey*Ki;
    ezInt = ezInt + ez*Ki;
    //将误差PI后补偿到陀螺仪
    gx = gx + Kp*ex+exInt;
    gy = gy + Kp*ey+eyInt;
    gz = gz + Kp*ez+ezInt;
	
    flight_state.attitudeRate.x= gx ;
	flight_state.attitudeRate.y= gy ;
	flight_state.attitudeRate.z= gz ;
	
	
    //一阶龙格库塔法更新四元数
    q0 = q0 + (-q1*gx - q2*gy - q3*gz) * halfT;
    q1 = q1 + (q0*gx + q2*gz - q3*gy) * halfT;
    q2 = q2 + (q0*gy - q1*gz + q3*gx) * halfT;
    q3 = q3 + (q0*gz + q1*gy - q2*gx) * halfT;  
	
	//规范化Pitch、Roll轴四元数
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 / norm;
	q1 = q1 / norm;
    q2 = q2 / norm;
	q3 = q3 / norm;
	
//	prepitch = flight_state.attitude.pitch;
//	preroll = flight_state.attitude.roll;
//	preyaw = flight_state.attitude.yaw;
	//计算得到俯仰角/横滚角/航向角
	flight_state.attitude.pitch = asin(-2*q1*q3 + 2*q0*q2)*57.3;								// pitch
	flight_state.attitude.roll = -atan2(2*q2*q3 + 2*q0*q1, - 2*q1*q1 - 2*q2*q2 + 1)*57.3;			// roll
	flight_state.attitude.yaw = -((atan2(2*(q1*q2 + q0*q3),q0*q0 + q1*q1 - q2*q2 - q3*q3)*57.3));	//yaw
	
}
