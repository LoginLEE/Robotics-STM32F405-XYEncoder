#include "positioning.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"



#define GYRO_COM	COM3
#define GYRO_BAUDRATE 115200

//#define raw_data_mode

/*** Command List ***/
#define SEND_START		0x05
#define SEND_X				0x11
#define SEND_Y				0x12
#define SEND_A				0x13
#define SEND_END			0x06

#define outputScaling 0.15f

u8 gyro_auto_update = 1;		// auto_update == 1 , position will send automatically

u8 uart_buf[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
u8 uart_buf_rec = 0;
u16 uart_cnt = 0;

#define countPerMM (2048.0f)/(2.0f*PI*23.7f)// count /2 * PI * r

float SHIFT_X = -25 * countPerMM;//-25; //mm
float SHIFT_Y = 90 * countPerMM;//90; //mm

float flip_x = 1.0f, flip_y = 1.0f, flip_a = -1.0f;

float xError, yError, changeOfX, changeOfY, changeOfA, lastEncoderX, lastEncoderY, encoderX, encoderY, lastGyroAngle, gyroAngle, calX, calY, sinAngle, cosAngle;

s16 outputX, outputY, outputA;

void position_uart_init(void)
{
	uart_init(GYRO_COM, GYRO_BAUDRATE);
}
float GgyroAngle(){return gyroAngle;}
void position_update(void)
{
	lastEncoderX = encoderX;
	lastEncoderY = encoderY;
	lastGyroAngle = gyroAngle;
	
	encoderX = flip_x*(float)getRawEncX();
	encoderY = flip_y*(float)getRawEncY();
	gyroAngle = getGyro();
	while(gyroAngle>180.0f)gyroAngle-=360.0f;
	while(gyroAngle<-180.0f)gyroAngle+=360.0f;
	//while(gyroAngle<0.0f)gyroAngle+=360.0f;
	
	changeOfX = encoderX - lastEncoderX;
	changeOfY = encoderY - lastEncoderY;
	changeOfA = gyroAngle - lastGyroAngle;

	outputA = (s16)(gyroAngle*10.0f);
	s16 angle = outputA / 10;
	sinAngle = arm_sin_f32(((float)angle)/180 * PI);
	cosAngle = arm_cos_f32(((float)angle)/180 * PI);
	
	yError = 2 * PI * SHIFT_X * changeOfA / 360.0f;
	xError = 2 * PI * SHIFT_Y * changeOfA / 360.0f;
	changeOfX -= xError;// X_FLIP*(x-SHIFT_X+(SHIFT_X*int_cos(angle)-SHIFT_Y*int_sin(angle))/10000);
	changeOfY -= yError;// Y_FLIP*(y-SHIFT_Y+(SHIFT_Y*int_cos(angle)+SHIFT_X*int_sin(angle))/10000);
	
	calX += changeOfX * cosAngle - changeOfY * sinAngle;
	calY += changeOfY * cosAngle + changeOfX * sinAngle;
	
	outputX = (s16)(calX * outputScaling);
	outputY = (s16)(calY * outputScaling);
}

void reset_possition(void)
{
	
	calX = 0;
	calY = 0;
}
float getxError(void)
{
	return xError;
}

float getyError(void)
{
	return yError;
}

float getchangeOfX(void)
{
	return changeOfX;
}

float getchangeOfY(void)
{
	return changeOfY;
}

float getchangeOfA(void)
{
	return changeOfA;
}

float getSinAngle(void)
{
	return sinAngle;
}

float getCosAngle(void)
{
	return cosAngle;
}

float getcalX(void)
{
	return calX;
}

float getcalY(void)
{
	return calY;
}

s16 getOutputX(void)
{
	return outputX;
}

s16 getOutputY(void)
{
	return outputY;
}

s16 getOutputA(void)
{
	return outputA;
}

u8 sendCount;

void send_pos(void)
{
	//***Original Part X Y Angle***//
	
	s16 x, y;
	#ifdef raw_data_mode
	{
		x = getRawEncX(); //raw output mode
		y = getRawEncY(); //raw output mode
	}
	#else
	{
		x = outputX;
		y = outputY;
	}
	#endif
	u8 send_buf[8];
	s16 a = outputA;
	send_buf[0] = SEND_START & 0xFF;
	send_buf[1] = (x & 0xFF00)>> 8;
	send_buf[2] = x & 0x00FF;
	send_buf[3] = (y & 0xFF00)>> 8;
	send_buf[4] = y & 0x00FF;
	send_buf[5] = (a & 0xFF00)>> 8;
	send_buf[6] = a & 0x00FF;
	send_buf[7] = SEND_END & 0xFF;
	
	/*
	uart_tx_byte(GYRO_COM, send_buf[sendCount]);
	sendCount++;
	if(sendCount == 8)sendCount = 0;
	*/
	
	#define ascii_data_mode
	
	#ifdef ascii_data_mode
	//uart_tx_blocking(COM3, "Pitch:%f Roll:%f Yaw:%f EncX:%d EncY:%d\n",getRawGyroX(), getRawGyroY(), getRawGyro(), getRawEncX(), getRawEncY());
	//uart_tx_blocking(COM3, "%f\n", getGyro());
	
	#else
	uart_tx_byte(GYRO_COM, send_buf[0]);
	uart_tx_byte(GYRO_COM, send_buf[1]);
	uart_tx_byte(GYRO_COM, send_buf[2]);
	uart_tx_byte(GYRO_COM, send_buf[3]);
	uart_tx_byte(GYRO_COM, send_buf[4]);
	uart_tx_byte(GYRO_COM, send_buf[5]);
	uart_tx_byte(GYRO_COM, send_buf[6]);
	uart_tx_byte(GYRO_COM, send_buf[7]);
	#endif
	
	//uart_tx_blocking(COM3, "%f %f %d\n", getcalX(), getcalY(), getOutputA());
	
	/*
	uart_tx_byte(COM1, SEND_X);
	uart_tx_byte(COM1, SEND_Y);
	uart_tx_byte(COM1, SEND_A);
	uart_tx_byte(COM1, x >> 8);
	uart_tx_byte(COM1, x & 0xFF);
	uart_tx_byte(COM1, y >> 8);
	uart_tx_byte(COM1, y & 0xFF);
	uart_tx_byte(COM1, a >> 8);
	uart_tx_byte(COM1, a & 0xFF);
	*/
	
}






