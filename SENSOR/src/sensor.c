#include "sensor.h"
#include "delay.h"
#include "config_parameter.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "hmc.h"
#include "math.h"


void Init_sensor()
{
	Init_MPU6050();										//初始化MPU6050六轴
	Init_HMC5883L();									//初始化HMC5883L磁力计
	constant.Gyro_K=Get_Gyro_K(SET_GYRO_RANGE);			//设置陀螺仪数据缩放(使用时/1000.0f)
	constant.Acc_K=Get_Acc_K(SET_GYRO_RANGE);			//设置加速度计数据缩放(使用时/1000.0f)
	constant.magx_ca=-156;
	constant.magy_ca=-123;
	constant.magz_ca=-340;								//磁力计数据校准
}

//void Calibration_sensor()
	

void Measure_sensor(sensor_data *sensordata)
{
		    MPU_Get_Accelerometer(&sensordata->acc);	//得到加速度传感器数据
		    MPU_Get_Gyroscope(&sensordata->gyro);		//得到陀螺仪数据
		    HMC_Get_magnetometer(&sensordata->mag);		//得到磁力计数据
			sensordata->mag.x+=constant.magx_ca;
			sensordata->mag.y+=constant.magy_ca;
			sensordata->mag.z+=constant.magz_ca;		//磁力计数据校准
}
