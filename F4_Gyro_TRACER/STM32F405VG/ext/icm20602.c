#include "icm20602.h"
#include "buzzer.h"



#define gyroSensitive 3700.0f//30200.0f//32768.0f //Gyro sensor sensitive
#define accWeight 70.0f
#define driftCorr 0.01f
int8_t setFlag = 0;
float th = 0.13f; //high pass filter th

//#define noiseThreshold 0.0005f
#define FSQRT3BY4 0.866025403784438646763723170752936183471402626905190314027f
#define CONVERT_TO_RADS PI/180.0f
#define CONVERT_TO_DEGS 180.0f/PI

Vec3 deltaRpy;
Vec4 q, r, quaternion, deltaQuat;

float xGyroSensitive = gyroSensitive;
float yGyroSensitive = gyroSensitive;
float zGyroSensitive = gyroSensitive;
Vec3 gbias = {0.0f, 0.0f, 0.0f};

float rawGyroX, rawGyroY, rawGyroZ, rawAccX, rawAccY, rawAccZ;
float gyroX, gyroY, gyroZ, accX, accY, accZ;
float fineGyro;
u32 imu_ticks = 0;

u32 recalTimes;
float lastFineGyro;
u32 recalibrating_count;

float lastGyro;
float dif;
float changeOfDif;
float accumulatedError;

float noiseError;
float fineFineGyro;

u8 tempcheck;
float fianl_gyro;
float changingFilterOffset;


// System constants
#define deltat 0.0001f // sampling period 
#define gyroMeasError 3.14159265358979f * (0.0f / 180.0f) // gyroscope measurement error in rad/s (shown as 5 deg/s)
#define beta sqrt(3.0f / 4.0f) * gyroMeasError // compute beta
#define CONVERT_TO_DEGS 180.0f/PI
// Global system variables
float a_x, a_y, a_z; // accelerometer measurements
float w_x, w_y, w_z; // gyroscope measurements
float SEq_1 = 1.0f, SEq_2 = 0.0f, SEq_3 = 0.0f, SEq_4 = 0.0f; // estimated orientation quaternion elements with initial conditions

void icm20602_init()
{
	 //SPI and GPIO Struct Initialization
   SPI_InitTypeDef   	SPI_InitStructure;
   GPIO_InitTypeDef 	GPIO_InitStructure;
   
   //Clock and Pins mapping to SPI2
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
   RCC_APB1PeriphClockLPModeCmd(RCC_APB1Periph_SPI2,ENABLE);
   GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
   GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
	 GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);
	
   /* Configure icm20602_SPI Pin: SCL(clock), SDO(MISO) and SDA(MOSI) */
   GPIO_InitStructure.GPIO_Pin = icm20602_SCL_PIN | icm20602_SDO_PIN | icm20602_SDA_PIN;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
   GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
   GPIO_Init(icm20602_SCL_PORT, &GPIO_InitStructure);
   
   /* Configure icm20602_SPI Pin: CS */
   GPIO_InitStructure.GPIO_Pin = icm20602_CS_PIN;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
   GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
   GPIO_Init(icm20602_CS_PORT, &GPIO_InitStructure);
   
   /* icm20602_SPI configuration */
   SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
   SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
   SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
   SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
   SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
   SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
   SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
   SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; 
   SPI_Init(icm20602_SPI, &SPI_InitStructure);
   
   /* Enable IMU_SPI   */
   SPI_Cmd(icm20602_SPI, ENABLE);
   SPI_CalculateCRC( icm20602_SPI , DISABLE );		// Disable the CRC checking
   SPI_SSOutputCmd( icm20602_SPI , DISABLE );	
	 
	 /* Update Timer init */
	RCC_APB1PeriphClockCmd(IMU_RCC, ENABLE);
	
	//Setup timer
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;
  TIM_TimeBaseStructure.TIM_Period = 100-1;//0.1ms
  TIM_TimeBaseInit(IMU_TIM, &TIM_TimeBaseStructure);
	
	TIM_ITConfig(IMU_TIM, TIM_IT_Update, ENABLE);
	
	//Setup interrupt
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = IMU_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	icm20602_config();
	
	TIM_Cmd(IMU_TIM, ENABLE);
	
	imu_filter_reset(accWeight, driftCorr);
}


u16 countt, count1, count2;
float finalDif, lastHighpass;
float gyroOffset = 0;
float lastDif = 0;
u16 difCount;

void IMU_IRQ_HANDLER(void)
{
	if (TIM_GetITStatus(IMU_TIM, TIM_IT_Update) != RESET){
		imu_ticks++;

		icm20602_gyro_update();

		if(countt++>10)
		{
			if(count2++>100)
			{
			dif = gyroZ - lastGyro;
			if(dif < th && dif > -th)gyroOffset += dif;//dif = 0;
			else if(dif > 0)gyroOffset+=th;
			else if(dif < 0)gyroOffset-=th;
			//fineGyro = gyroZ - gyroOffset;
			lastGyro = gyroZ;
			
			
			if(lastDif - dif < 0.05f && lastDif - dif > -0.05f)difCount++;
			else difCount = 0;
			if(difCount>=1000)
			{
				if(dif < 0)th = -dif + 0.001f;
				else th = dif+ 0.001f;
				difCount = 0;
			}
			count2 = 0;
			}
			fineGyro = gyroZ - gyroOffset;
			finalDif = lastHighpass - fineGyro;
			if(((finalDif > 0.8f || finalDif < -0.8f)) &&(finalDif <180.0f && finalDif >-180.0f) &&!setFlag)changingFilterOffset-=finalDif / 3.0f;//2.5f;
			
			//changingFilterOffset = 0;
			fianl_gyro = fineGyro + changingFilterOffset;
			while(fianl_gyro>180.0f)fianl_gyro-=360.0f;
			while(fianl_gyro<-180.0f)fianl_gyro+=360.0f;
			
			lastHighpass = fineGyro;
			lastDif = dif;
			countt = 0;
			if(setFlag)setFlag = 0;
		}
		
		
		position_update();
		
		if(count1++>10)//1ms
		{
			//uart_tx_blocking(COM3, "fine%f raw%f dif%f off%f difC%d TH%f RawIMU%f fianlDif%f\n", fineGyro, gyroZ, dif, gyroOffset, difCount, th, rawGyroZ, finalDif);
			//uart_tx_blocking(COM3, "RX%d RY%d RA%f CX%f CY%f CA%f EX%f EY%f\n", getRawEncX(), getRawEncY(), getGyro(), getchangeOfX(), getchangeOfY(), getchangeOfA(), getxError(), getyError());
			//uart_tx_blocking(COM3, "Output %d %d %d\n\n", getOutputX(), getOutputY(), getOutputA());
			//uart_tx_blocking(COM3, "out%f finalDif%f last%f chOff%f\n", fianl_gyro, finalDif, lastHighpass, changingFilterOffset);
			uart_tx_blocking(COM3, "%f\n", fianl_gyro);
			count1 = 0;
		}
		
		TIM_ClearITPendingBit(IMU_TIM, TIM_IT_Update);
	}
}

void reset_IMU(void)
{
	changingFilterOffset = -fianl_gyro + changingFilterOffset - gyroOffset;
	gyroOffset = 0;
	setFlag = 1;
}

float getLastGyro(void)
{
	return lastFineGyro;
}

u8 getCheck(void)
{
	return tempcheck;
}

float getFineFineGyro(void)
{
	return fineFineGyro;
}

Vec4 unit_quaternion(void) {
	return (Vec4){1.0f, 0.0f, 0.0f, 0.0f};
}

Vec3 zero_vector(void) {
	return (Vec3){0.0f, 0.0f, 0.0f};
}


static float beta1, zeta;
void imu_filter_reset(float acc_weight, float drift_corr) {
	quaternion = unit_quaternion();
	gbias = zero_vector();
	
	//Relative contribution of accelerometer and gyroscope
	//Higher beta equals accelerometer considered more reliable, Noisy but converges quicker
	//Lower beta reduces noise but slower to converge/respond to changes
	//Find the optimal
	beta1 = acc_weight * FSQRT3BY4 * CONVERT_TO_RADS;
	//Aggressiveness of integrator in estimating the gyro bias
	zeta = drift_corr * FSQRT3BY4 * CONVERT_TO_RADS;
}

void icm20602_config()
{
	_delay_ms(1); 
	icm20602WriteRegister(REG_PWR_MGMT_1, 0x80);// full reset of chip
	_delay_ms(1);
	icm20602WriteRegister(REG_PWR_MGMT_1, 0x01);// set clock to internal PLL
	_delay_ms(1);
	icm20602WriteRegister(REG_PWR_MGMT_2, 0x00);// place accel and gyro on standby
	_delay_ms(1);
	icm20602WriteRegister(REG_USER_CTRL, 0x00);// disable fifo
	_delay_ms(1);
	icm20602WriteRegister(REG_I2C_IF, 0x40);// disable chip I2C communications
	_delay_ms(1);
	icm20602WriteRegister(REG_ACCEL_CONFIG_2, 0x08);// set acc
	_delay_ms(1);
	icm20602WriteRegister(REG_CONFIG, 0x00);// // bypass dpf
	_delay_ms(1);
	icm20602WriteRegister(REG_GYRO_CONFIG, 0x09);// set dps
	_delay_ms(1);
	icm20602WriteRegister(REG_FIFO_EN, 0x10);// enable gyro FIFO
	_delay_ms(1);
	icm20602WriteRegister(REG_PWR_MGMT_2, 0x00);// enable gyro
	_delay_ms(1); 
}

u8 icm20602_ReadWriteByte(u8 TxData)
{                                         
		while (SPI_I2S_GetFlagStatus(icm20602_SPI, SPI_I2S_FLAG_TXE) == RESET);
		SPI_I2S_SendData(icm20602_SPI, TxData);
		while (SPI_I2S_GetFlagStatus(icm20602_SPI, SPI_I2S_FLAG_RXNE) == RESET);
		return ((uint8_t)SPI_I2S_ReceiveData(icm20602_SPI));                          
}


/**
  * @brief  Sending a command
  * @param  command: one byte command to be sent
  * @retval None
  */
void icm20602_write_command(uint8_t command)
{
		GPIO_ResetBits(icm20602_CS_PORT, icm20602_CS_PIN);//enable spi command pin
	
		/* Loop while DR register in not emplty */
		while (SPI_I2S_GetFlagStatus(icm20602_SPI, SPI_I2S_FLAG_TXE) == RESET);
		/* Send byte through the TFT_SPI peripheral */
		SPI_I2S_SendData(icm20602_SPI, command);
		while (SPI_I2S_GetFlagStatus(icm20602_SPI, SPI_I2S_FLAG_RXNE) == RESET);
		SPI_I2S_ReceiveData(icm20602_SPI);
	
		GPIO_SetBits(icm20602_CS_PORT, icm20602_CS_PIN);//disable spi command pin
}

u8 icm20602_ReadWriteLen(SPI_TypeDef *instance, u8 *out, const u8 *in, int len)
{
    u8 b;
    instance->DR;
    while (len--) 
		{
				b = in ? *(in++) : 0xFF;
				while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_TXE) == RESET){} 
				SPI_I2S_SendData(instance, b);
				while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_RXNE) == RESET){}
				b = SPI_I2S_ReceiveData(instance);
				if (out)*(out++) = b;
    }
    return 1;
}

u8 icm20602WriteRegister(u8 Wreg,u8 Wdata)
{
    u8 status;
    GPIO_ResetBits(icm20602_CS_PORT, icm20602_CS_PIN);//enable spi command pin
    status=icm20602_ReadWriteByte(Wreg);
    status = icm20602_ReadWriteByte(Wdata);
    GPIO_SetBits(icm20602_CS_PORT, icm20602_CS_PIN);//disable spi command pin
    return (status);
}


u8 icm20602ReadRegister(u8 Rreg, u8 length, u8 *Rdata)
{
    GPIO_ResetBits(icm20602_CS_PORT, icm20602_CS_PIN);
    icm20602_ReadWriteByte(Rreg|0x80);
		icm20602_ReadWriteLen(icm20602_SPI, Rdata, 0, length);
    GPIO_SetBits(icm20602_CS_PORT, icm20602_CS_PIN);
    return 0;
}

void icm20602_read_gyro_raw(float *p_x, float *p_y, float *p_z)
{
	int16_t x, y, z;
	static u8 rawGyro[6] = {0};
	
	icm20602ReadRegister(REG_GYRO_XOUT_H, 6, rawGyro);
	
	x = 0x00 | rawGyro[0];
	x <<= 8;
	x |= rawGyro[1];
	
	y = 0x00 | rawGyro[2];
	y <<= 8;
	y |= rawGyro[3];
	
	z = 0x00 | rawGyro[4];
	z <<= 8;
	z |= rawGyro[5];

	*p_x = (float)x / xGyroSensitive;
	*p_y = (float)y / yGyroSensitive;
	*p_z = (float)z / zGyroSensitive;
}

void icm20602_read_acc_raw(float *p_x, float *p_y, float *p_z)
{
	
	int16_t x, y, z;
	static u8 rawAcc[6] = {0};
	
	icm20602ReadRegister(REG_ACCEL_XOUT_H, 6, rawAcc);
	
	x = 0x00 | rawAcc[0];
	x <<= 8;
	x |= rawAcc[1];
	
	y = 0x00 | rawAcc[2];
	y <<= 8;
	y |= rawAcc[3];
	
	z = 0x00 | rawAcc[4];
	z <<= 8;
	z |= rawAcc[5];

	*p_x = (float)x;
	*p_y = (float)y;
	*p_z = (float)z;

}

void icm20602_gyro_update(void)
{
	icm20602_read_gyro_raw(&rawGyroX,&rawGyroY,&rawGyroZ);
	icm20602_read_acc_raw(&rawAccX, &rawAccY, &rawAccZ);
	//filterUpdate(rawGyroX, rawGyroY, rawGyroZ, rawAccX, rawAccY, rawAccZ);
	madgwick_filter_update(rawGyroX, rawGyroY, rawGyroZ, rawAccX, rawAccY, rawAccZ, &gbias);
	get_gyro_from_quat(&gyroX, &gyroY, &gyroZ);
}

void filterUpdate(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z)
{
	// Local system variables
	float norm; // vector norm
	float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion derrivative from gyroscopes elements
	float f_1, f_2, f_3; // objective function elements
	float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
	float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
	// Axulirary variables to avoid reapeated calcualtions
	float halfSEq_1 = 0.5f * SEq_1;
	float halfSEq_2 = 0.5f * SEq_2;
	float halfSEq_3 = 0.5f * SEq_3;
	float halfSEq_4 = 0.5f * SEq_4;
	float twoSEq_1 = 2.0f * SEq_1;
	float twoSEq_2 = 2.0f * SEq_2;
	float twoSEq_3 = 2.0f * SEq_3;
	
	// Normalise the accelerometer measurement
	norm = sqrtf(a_x * a_x + a_y * a_y + a_z * a_z);
	a_x /= norm;
	a_y /= norm;
	a_z /= norm;
	// Compute the objective function and Jacobian
	f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
	f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
	f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
	J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
	J_12or23 = 2.0f * SEq_4;
	J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
	J_14or21 = twoSEq_2;
	J_32 = 2.0f * J_14or21; // negated in matrix multiplication
	J_33 = 2.0f * J_11or24; // negated in matrix multiplication
	// Compute the gradient (matrix multiplication)
	SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
	SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
	SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
	SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;
	// Normalise the gradient
	norm = sqrtf(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
	SEqHatDot_1 /= norm;
	SEqHatDot_2 /= norm;
	SEqHatDot_3 /= norm;
	SEqHatDot_4 /= norm;

	// Compute the quaternion derrivative measured by gyroscopes
	SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
	SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
	SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
	SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;
	
	deltaQuat.q1 = SEqDot_omega_1;
	deltaQuat.q2 = SEqDot_omega_2;
	deltaQuat.q3 = SEqDot_omega_3;
	deltaQuat.q4 = SEqDot_omega_4;
	
	// Compute then integrate the estimated quaternion derrivative
	SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
	SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
	SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
	SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;
	
	//SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4 to find angular velocity
	
	// Normalise quaternion
	norm = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
	SEq_1 /= norm;
	SEq_2 /= norm;
	SEq_3 /= norm;
	SEq_4 /= norm;
	
	quaternion.q1 = SEq_1;
	quaternion.q2 = SEq_2;
	quaternion.q3 = SEq_3;
	quaternion.q4 = SEq_4;
}

float inv_sqrt(float x) {
#ifdef USE_QUAKE_INVSQRT
	float halfx = 0.5f * x;
	float y = x;
	int32_t i = *(int32_t *) &y;
	i = 0x5f3759df - (i >> 1);
	y = *(float *) &i;
	y = y * (1.5f - (halfx * y * y));
	return y;
#else
	float _sqrt; 
	arm_sqrt_f32(x, &_sqrt);
	return 1.0f/_sqrt;
#endif
}


void madgwick_filter_update(float gx, float gy, float gz, float ax, float ay, float az, Vec3 *gbias) {
	float q1 = SEq_1, q2 = SEq_2, q3 = SEq_3, q4 = SEq_4;         // short name local variable for readability

	float norm;                                               // vector norm
	float f1, f2, f3;                                         // objective function elements
	float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
	float qDot1, qDot2, qDot3, qDot4;
	float hatDot1, hatDot2, hatDot3, hatDot4;
	float gerrx, gerry, gerrz, gbiasx=gbias->x, gbiasy=gbias->y, gbiasz=gbias->z;  // gyro bias error

	// Remember to normalise accelerometer measurement outside
	// Unless you changed something after this comment I normalize and ensure the accel reading is nonzero
	// So I dont need the check in every filter iteration

	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;
	float _2q4 = 2.0f * q4;

	// Compute the objective function and Jacobian
	f1 = _2q2 * q4 - _2q1 * q3 - ax;
	f2 = _2q1 * q2 + _2q3 * q4 - ay;
	f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
	J_11or24 = _2q3;
	J_12or23 = _2q4;
	J_13or22 = _2q1;
	J_14or21 = _2q2;
	J_32 = 2.0f * J_14or21;
	J_33 = 2.0f * J_11or24;

	// Compute the gradient (matrix multiplication)
	hatDot1 = J_14or21 * f2 - J_11or24 * f1;
	hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
	hatDot3 = J_12or23 * f2 - J_33 * f3 - J_13or22 * f1;
	hatDot4 = J_14or21 * f1 + J_11or24 * f2;

	// Normalize the gradient
	if ((hatDot1==0.0f) && (hatDot2==0.0f) && (hatDot3==0.0f) && (hatDot4==0.0f)) return;
	norm = inv_sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
	hatDot1 *= norm;
	hatDot2 *= norm;
	hatDot3 *= norm;
	hatDot4 *= norm;

	// Compute and remove estimated gyroscope biases
	float dtz = deltat*zeta;
	gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
	gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
	gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;
	
	gbiasx += gerrx * dtz;//deltat * zeta; 
	gbiasy += gerry * dtz;//deltat * zeta; 
	gbiasz += gerrz * dtz;//deltat * zeta; 
	
	gx -= gbiasx;
	gy -= gbiasy;
	gz -= gbiasz;
	
	// Remember estimated gyroscope biases
	gbias->x = gbiasx;
	gbias->y = gbiasy;
	gbias->z = gbiasz;

	// Compute the quaternion derivative
	float _halfq1 = 0.5f * q1;
	float _halfq2 = 0.5f * q2;
	float _halfq3 = 0.5f * q3;
	float _halfq4 = 0.5f * q4;
	qDot1 = -_halfq2 * gx - _halfq3 * gy - _halfq4 * gz;
	qDot2 = _halfq1 * gx + _halfq3 * gz - _halfq4 * gy;
	qDot3 = _halfq1 * gy - _halfq2 * gz + _halfq4 * gx;
	qDot4 = _halfq1 * gz + _halfq2 * gy - _halfq3 * gx;

	// Correct estimated quaternion derivative
	qDot1 -= beta1 * hatDot1;
	qDot2 -= beta1 * hatDot2;
	qDot3 -= beta1 * hatDot3;
	qDot4 -= beta1 * hatDot4;
	
	// Finally integrate
	q1 += qDot1 * deltat;
	q2 += qDot2 * deltat;
	q3 += qDot3 * deltat;
	q4 += qDot4 * deltat;
	
	// Normalize the quaternion
	norm = inv_sqrt(q1*q1 + q2*q2 + q3*q3 + q4*q4);
	SEq_1 = q1 * norm;
	SEq_2 = q2 * norm;
	SEq_3 = q3 * norm;
	SEq_4 = q4 * norm;
}

void get_gyro_from_quat(float* rollOutput, float* pitchOutput, float* yawOutput) {
	// Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
	// In this coordinate system, the positive z-axis is down toward Earth. 
	// Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
	// Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
	// Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
	// These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
	// Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
	// applied in the correct order which for this configuration is yaw, pitch, and then roll.
	// For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
	// Yaw, Pitch, Roll
	float q12_q32 = SEq_1*SEq_1 - SEq_3*SEq_3;
	float q22_q42 = SEq_2*SEq_2 - SEq_4*SEq_4;
	
	//Yaw
	// rpy.z = atan2(2.0f * (q2 * q3 + q1 * q4), q1 * q1 + q2 * q2 - q3 * q3 - q4 * q4) * 180.0f / PI;   
	*yawOutput = atan2f(
		(2.0f*(SEq_2*SEq_3 + SEq_1*SEq_4)),
		q12_q32 + q22_q42
		) * CONVERT_TO_DEGS;
	//Roll
	// rpy.x = inv_tan(2.0f * (q1 * q2 + q3 * q4), q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4) * 180.0f / PI;
	*rollOutput = atan2f(
		2.0f*(SEq_1*SEq_2 + SEq_3*SEq_4),
		q12_q32 - q22_q42
		) * CONVERT_TO_DEGS;
	//Pitch
	*pitchOutput = -asinf(2.0f * (SEq_2 * SEq_4 - SEq_1 * SEq_3)) * CONVERT_TO_DEGS;
	
}

float getRawGyroX(void)
{
	return gyroX;
}

float getRawGyroY(void)
{
	return gyroY;
}

float getRawGyro(void)
{
	return gyroZ;
}

float getGyro(void)
{
	return fineGyro;
}	

void imu_recal(u32 seconds)
{
	recalibrating_count = seconds*10000;
	recalTimes = recalibrating_count;
	changeOfDif = 0;
}

/*
static inline void quat_mul(Vec4* t, Vec4* q, Vec4* r) {
	float q1 = q->q1;
	float q2 = q->q2;
	float q3 = q->q3;
	float q4 = q->q4;  
	float r1 = r->q1;
	float r2 = r->q2;
	float r3 = r->q3;
	float r4 = r->q4;  
	t->q1 = r1*q1 - r2*q2 - r3*q3 - r4*q4;
	t->q2 = r1*q2 + r2*q1 - r3*q4 + r4*q3;
	t->q3 = r1*q3 + r2*q4 + r3*q1 - r4*q2;
	t->q4 = r1*q4 - r2*q3 + r3*q2 + r4*q1;
}
*/

void get_velocity(float* getX, float* getY, float* getZ)
{
	drpy_from_quat(&quaternion, &deltaQuat, &deltaRpy);
	*getX = deltaRpy.x;
	*getY = deltaRpy.y;
	*getZ = deltaRpy.z;
}


static void drpy_from_quat(Vec4* q, Vec4* dq, Vec3 *rpy) {
	Vec4 qinv;
	quat_inv(&qinv, q);
	quat_mu(rpy, 2*CONVERT_TO_DEGS, dq, &qinv);
}

static inline void quat_inv(Vec4* t, Vec4* q) {
	t->q1 =  q->q1;
	t->q2 = -q->q2;
	t->q3 = -q->q3;
	t->q4 = -q->q4;
}

static inline void quat_mu(Vec3* w, float scale, Vec4* q, Vec4* r) {
	float q1 = q->q1;
	float q2 = q->q2;
	float q3 = q->q3;
	float q4 = q->q4;  
	float r1 = r->q1;
	float r2 = r->q2;
	float r3 = r->q3;
	float r4 = r->q4;  
	w->x = scale * (r1*q2 + r2*q1 - r3*q4 + r4*q3);
	w->y = scale * (r1*q3 + r2*q4 + r3*q1 - r4*q2);
	w->z = scale * (r1*q4 - r2*q3 + r3*q2 + r4*q1);
}


