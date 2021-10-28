
#include "usart1.h"

//串口1发送1个字符 
//c:要发送的字符
void usart1_send_char(u8 c)
{

	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); 
    USART_SendData(USART1,c);   

} 

//传送数据给匿名四轴上位机软件(V4版本)
//fun:功能字
//len:data区有效数据个数
void usart1_niming_report(u8 fun,u8*data,u8 len)
{
	u8 send_buf[32]={0x00};
	u8 i;
	if(len>28)return;	//最多28字节数据 
	send_buf[len+4]=0;	//校验数置零
	send_buf[0]=0XAA;	//帧头
	send_buf[1]=0XAA;	//帧头
	send_buf[2]=fun;	//功能字
	send_buf[3]=len;	//数据长度
	for(i=0;i<len;i++)send_buf[4+i]=data[i];			//复制数据
	for(i=0;i<len+4;i++)send_buf[len+4]+=send_buf[i];	//计算校验和	
	for(i=0;i<len+5;i++)usart1_send_char(send_buf[i]);	//发送数据到串口1 
}
	
//通过串口1上报结算后的姿态数据给电脑
//格式：AAAA 01 int16 ROL*100 int16 PIT*100 int16 YAW*100 int16 ALT_CSB(超声波高度,单位厘米) int32 ALT_PRS(气压计高度,单位毫米）
//只发送姿态角，其它置0
//roll:横滚角.单位0.01度。 -18000 -> 18000 对应 -180.00  ->  180.00度
//pitch:俯仰角.单位 0.01度。-9000 - 9000 对应 -90.00 -> 90.00 度
//yaw:航向角.单位为0.1度 0 -> 3600  对应 0 -> 360.0度
void usart1_report_Angle(u8 fun)
{
	short pitch,roll,yaw;
	pitch=(short)(flight_state.attitude.pitch*100);
	roll=(short)(flight_state.attitude.roll*100);
	yaw=(short)(flight_state.attitude.yaw*100);
	u8 tbuf[6]={0x00}; 
	u8 i;
	for(i=0;i<12;i++)tbuf[i]=0;//清0
	tbuf[0]=(roll>>8)&0XFF;
	tbuf[1]=roll&0XFF;
	tbuf[2]=(pitch>>8)&0XFF;
	tbuf[3]=pitch&0XFF;
	//tbuf[4]=(yaw>>8)&0XFF;
	//tbuf[5]=yaw&0XFF;
	usart1_niming_report(fun,tbuf,6);//姿态角显示帧,0X01
} 

//以传感器显示帧(功能码02)发送加速度传感器数据,陀螺仪数据和磁力计数据
void usart1_report_senserdata(sensor_data 	*sensordata,u8 fun)
{
	u8 tbuf[18]={0x00}; 
	tbuf[0]=((sensordata->acc.x)>>8)&0XFF;
	tbuf[1]=(sensordata->acc.x)&0XFF;
	tbuf[2]=(sensordata->acc.y>>8)&0XFF;
	tbuf[3]=sensordata->acc.y&0XFF;
	tbuf[4]=(sensordata->acc.z>>8)&0XFF;
	tbuf[5]=sensordata->acc.z&0XFF; 
	
	tbuf[6]=(sensordata->gyro.x>>8)&0XFF;
	tbuf[7]=sensordata->gyro.x&0XFF;
	tbuf[8]=(sensordata->gyro.y>>8)&0XFF;
	tbuf[9]=sensordata->gyro.y&0XFF;
	tbuf[10]=(sensordata->gyro.z>>8)&0XFF;
	tbuf[11]=sensordata->gyro.z&0XFF;
	
    tbuf[12]=(sensordata->mag.x>>8)&0XFF;
	tbuf[13]=sensordata->mag.x&0XFF;
	tbuf[14]=(sensordata->mag.y>>8)&0XFF;
	tbuf[15]=sensordata->mag.y&0XFF;
	tbuf[16]=(sensordata->mag.z>>8)&0XFF;
	tbuf[17]=sensordata->mag.z&0XFF;
	usart1_niming_report(fun,tbuf,18);
}

void usart1_report_Control(Control_signal *control,u8 fun)
{
	short pitch,roll,yaw,thrust;
	pitch=(short)control->pitch;
	roll=(short)control->roll;
	yaw=(short)control->yaw;
	thrust=(short)control->thrust;
	u8 tbuf[8]={0x00}; 
	u8 i;
	for(i=0;i<12;i++)tbuf[i]=0;//清0
	tbuf[0]=(roll>>8)&0XFF;
	tbuf[1]=roll&0XFF;
	tbuf[2]=(pitch>>8)&0XFF;
	tbuf[3]=pitch&0XFF;
	tbuf[4]=(yaw>>8)&0XFF;
	tbuf[5]=yaw&0XFF;
	tbuf[6]=(thrust>>8)&0XFF;
	tbuf[7]=thrust&0XFF;
	usart1_niming_report(fun,tbuf,8);//姿态角显示帧,0X01
} 

void usart1_report_PID(PID_structure *pidAngleRoll,PID_structure *pidAnglePitch,PID_structure * pidAngleYaw,u8 fun)
{
	short rollkp,rollki,rollkd,pitchkp,pitchki,pitchkd,yawkp,yawki,yawkd;
	rollkp=(short)pidAngleRoll->KP;
	rollki=(short)pidAngleRoll->KI;
	rollkd=(short)pidAngleRoll->KD;
	
	pitchkp=(short)pidAnglePitch->KP;
	pitchki=(short)pidAnglePitch->KI;
	pitchkd=(short)pidAnglePitch->KD;
	
	yawkp=(short)pidAngleYaw->KP;
	yawki=(short)pidAngleYaw->KI;
	yawkd=(short)pidAngleYaw->KD;
	u8 tbuf[18]={0x00}; 
	u8 i;
	for(i=0;i<12;i++)tbuf[i]=0;//清0
	tbuf[0]=(rollkp>>8)&0XFF;
	tbuf[1]=rollkp&0XFF;
	tbuf[2]=(rollki>>8)&0XFF;
	tbuf[3]=rollki&0XFF;
	tbuf[4]=(rollkd>>8)&0XFF;
	tbuf[5]=rollkd&0XFF;
	
	tbuf[6]=(pitchkp>>8)&0XFF;
	tbuf[7]=pitchkp&0XFF;
	tbuf[8]=(pitchki>>8)&0XFF;
	tbuf[9]=pitchki&0XFF;
	tbuf[10]=(pitchkd>>8)&0XFF;
	tbuf[11]=pitchkd&0XFF;
	
	tbuf[12]=(yawkp>>8)&0XFF;
	tbuf[13]=yawkp&0XFF;
	tbuf[14]=(yawki>>8)&0XFF;
	tbuf[15]=yawki&0XFF;
	tbuf[16]=(yawkd>>8)&0XFF;
	tbuf[17]=yawkd&0XFF;
	usart1_niming_report(fun,tbuf,18);//姿态角显示帧,0X01
} 

void usart1_report_motorPWM(motorPWM *motor_PWM_,u8 fun)
{
	short m1,m2,m3,m4;
	m1=(short)motor_PWM_->m1;
	m2=(short)motor_PWM_->m2;
	m3=(short)motor_PWM_->m3;
	m4=(short)motor_PWM_->m4;

	u8 tbuf[8]={0x00}; 
	u8 i;
	for(i=0;i<12;i++)tbuf[i]=0;//清0
	tbuf[0]=(m1>>8)&0XFF;
	tbuf[1]=m1&0XFF;
	tbuf[2]=(m2>>8)&0XFF;
	tbuf[3]=m2&0XFF;
	tbuf[4]=(m3>>8)&0XFF;
	tbuf[5]=m3&0XFF;
	
	tbuf[6]=(m4>>8)&0XFF;
	tbuf[7]=m4&0XFF;
	usart1_niming_report(fun,tbuf,8);//姿态角显示帧,0X01
} 

void usart1_report_loop_num(u8 fun)
{
	u8 tbuf[4]={0x00}; 
	u8 i;
	for(i=0;i<4;i++)tbuf[i]=0;//清0
	tbuf[0]=(loop_num>>8)&0XFF;
	tbuf[1]=loop_num&0XFF;

	usart1_niming_report(fun,tbuf,4);//姿态角显示帧,0X01
} 
