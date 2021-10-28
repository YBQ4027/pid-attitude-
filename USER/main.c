#include "sys.h"
#include "delay.h"
#include "sensor.h"
#include "usart.h"
#include "led.h"
#include "key.h"
#include "pwm.h"
#include "attitude.h"
#include "state_control.h"
#include "flight_control.h"
#include "math.h"
#include "power_control.h"
#include "usart1.h"
#include "config_parameter.h"
#include "sensorfusion_mahony.h"

float actualThrust;
 
int main(void)
{ 
	u8 report=1;			    //默认开启上报
	u8 key;
	loop_num=0;                  //记录循环次数
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);            //初始化延时函数
	uart_init(500000);		    //初始化串口波特率为500000
	//LED_Init();					//初始化LED 
	KEY_Init();					//初始化按键
	Init_sensor();
	stateControlInit();
	Init_configParam();			//设置默认pid参数
	paramdisired.attitude_desire.pitch=0;
	paramdisired.attitude_desire.roll=0;
	paramdisired.attitude_desire.yaw=0;
	actualThrust=1.0f/5.0f * POWER_MAX;//max 1.4118f
	TIM2_PWM_Init(PWM_CNT-1,84-1);	//84M/84=1Mhz的计数频率,重装载值500
	//500Hz
	while(1)
	{
		key=KEY_Scan(0);
		if(key==KEY0_PRES)
		{
			report=!report;
		}
		else if(key==KEY2_PRES)
	    {
			loop_num=0;
	    }
		else if(key==KEY1_PRES)
	    {
			actualThrust+=(0.02f * POWER_MAX);
	    }
		//else if(key==KEY2_PRES)
	   // {
		//	actualThrust-=(0.02f * POWER_MAX);
	   // }
		if(report)
        {		
	     //   LED1=0;//数据上传时LED1亮
			//Measure_sensor(&Sensordata);
			flight_control_task();
			
			//KEY1键按下时将偏航角置0
			//if(key==KEY2_PRES)
			//{
			//yaw0=flight_state.attitude.yaw;
			//}
			//flight_state.attitude.yaw-=yaw0;
			
			//usart1_report_senserdata(&Sensordata,0XF1);//9
			//usart1_report_PID(&pidAngleRoll,&pidAnglePitch,&pidAngleYaw,0XF1);//9
			//usart1_report_Angle(0X01);//3
			//usart1_report_Angle(0XF1);//3
			//usart1_report_Control(&control,0XF2);//4
			//usart1_report_motorPWM(&motor_PWM,0XF2);//4
			//usart1_report_motorPWM(&motor_PWMratio,0XF3);//4
			//usart1_report_loop_num(0XF2);//3
		    loop_num++;
        }
		//else LED1=1;
	} 	
}
