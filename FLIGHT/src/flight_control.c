#include "flight_control.h"
#include "config_parameter.h"
#include "sensor.h"
#include "sensorfusion_mahony.h"
#include "state_control.h"
#include "power_control.h"
//static ParamDesired setdesire;	/*目标状态*/

sensor_data Sensordata;
Control_signal control;
//static State state;
ParamDesired paramdisired;
//void stabilizerTask(void* param)
void flight_control_task()
{	

		Measure_sensor(&Sensordata);
		
		Update_AHRS_mahony(&Sensordata);
		//Update_IMU_mahony(&Sensordata);
		StateControl(&control,&flight_state,&paramdisired,actualThrust);
		powerControl(&control);
}
