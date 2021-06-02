#ifndef __ICM20602
#define __ICM20602

#include "stm32f4xx.h"
#include "math.h"
#include "approx_math.h"
#include "trigon_math.h"
#include "macro_math.h"
#include "ticks.h"
#include "positioning.h"

#define IMU_TIM					TIM7
#define IMU_RCC					RCC_APB1Periph_TIM7
#define IMU_IRQ					TIM7_IRQn
#define IMU_IRQ_HANDLER	TIM7_IRQHandler


#define icm20602_SCL_PIN		GPIO_Pin_13
#define icm20602_SCL_PORT		GPIOB

#define icm20602_SDA_PIN		GPIO_Pin_15
#define icm20602_SDA_PORT		GPIOB

#define icm20602_SDO_PIN		GPIO_Pin_14
#define icm20602_SDO_PORT		GPIOB

#define icm20602_CS_PIN			GPIO_Pin_12
#define icm20602_CS_PORT		GPIOB

#define icm20602_SPI			SPI2

#define REG_XG_OFFS_TC_H 0x04
#define REG_XG_OFFS_TC_L 0x05
#define REG_YG_OFFS_TC_H 0x07
#define REG_YG_OFFS_TC_L 0x08
#define REG_ZG_OFFS_TC_H 0x0A
#define REG_ZG_OFFS_TC_L 0x0B
#define REG_SELF_TEST_X_ACCEL 0x0D
#define REG_SELF_TEST_Y_ACCEL 0x0E
#define REG_SELF_TEST_Z_ACCEL 0x0F
#define REG_XG_OFFS_USRH 0x13
#define REG_XG_OFFS_USRL 0x14
#define REG_YG_OFFS_USRH 0x15
#define REG_YG_OFFS_USRL 0x16
#define REG_ZG_OFFS_USRH 0x17
#define REG_ZG_OFFS_USRL 0x18
#define REG_SMPLRT_DIV 0x19
#define REG_CONFIG 0x1A
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_CONFIG_2 0x1D
#define REG_LP_MODE_CFG 0x1E
#define REG_ACCEL_WOM_X_THR 0x20
#define REG_ACCEL_WOM_Y_THR 0x21
#define REG_ACCEL_WOM_Z_THR 0x22
#define REG_FIFO_EN 0x23
#define REG_FSYNC_INT 0x36
#define REG_INT_PIN_CFG 0x37
#define REG_INT_ENABLE 0x38
#define REG_FIFO_WM_INT_STATUS 0x39
#define REG_INT_STATUS 0x3A
#define REG_ACCEL_XOUT_H 0x3B
#define REG_ACCEL_XOUT_L 0x3C
#define REG_ACCEL_YOUT_H 0x3D
#define REG_ACCEL_YOUT_L 0x3E
#define REG_ACCEL_ZOUT_H 0x3F
#define REG_ACCEL_ZOUT_L 0x40
#define REG_TEMP_OUT_H 0x41
#define REG_TEMP_OUT_L 0x42
#define REG_GYRO_XOUT_H 0x43
#define REG_GYRO_XOUT_L 0x44
#define REG_GYRO_YOUT_H 0x45
#define REG_GYRO_YOUT_L 0x46
#define REG_GYRO_ZOUT_H 0x47
#define REG_GYRO_ZOUT_L 0x48
#define REG_SELF_TEST_X_GYRO 0x50
#define REG_SELF_TEST_Y_GYRO 0x51
#define REG_SELF_TEST_Z_GYRO 0x52
#define REG_FIFO_WM_TH1 0x60
#define REG_FIFO_WM_TH2 0x61
#define REG_SIGNAL_PATH_RESET 0x68
#define REG_ACCEL_INTEL_CTRL 0x69
#define REG_USER_CTRL 0x6A
#define REG_PWR_MGMT_1 0x6B
#define REG_PWR_MGMT_2 0x6C
#define REG_I2C_IF 0x70
#define REG_FIFO_COUNTH 0x72
#define REG_FIFO_COUNTL 0x73
#define REG_FIFO_R_W 0x74
#define REG_WHO_AM_I 0x75
#define REG_XA_OFFSET_H 0x77
#define REG_XA_OFFSET_L 0x78
#define REG_YA_OFFSET_H 0x7A
#define REG_YA_OFFSET_L 0x7B
#define REG_ZA_OFFSET_H 0x7D
#define REG_ZA_OFFSET_L 0x7E

#define REG_WHO_AM_I_CONST 0X12

typedef struct{
	float x;
	float y;
	float z;
} Vec3;

typedef struct{
	float q1;
	float q2;
	float q3;
	float q4;
} Vec4;

void icm20602_init(void);
void icm20602_config(void);
void reset_IMU(void);
void madgwick_filter_update(float gx, float gy, float gz, float ax, float ay, float az, Vec3 *gbias);
void imu_filter_reset(float acc_weight, float drift_corr);
u8 icm20602WriteRegister(u8 Wreg,u8 Wdata);
u8 icm20602ReadRegister(u8 reg, u8 length, u8 *data);
void icm20602_read_gyro_raw(float *p_x, float *p_y, float *p_z);
void icm20602_read_gyro(float *g_x, float *g_y, float *g_z);
void icm20602_gyro_update(void);
void icm20602_read_temp_raw(float *p_x);
void filterUpdate(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z);
void get_gyro_from_quat(float* rollOutput, float* pitchOutput, float* yawOutput);
void imu_recal(u32 seconds);
float getGyro(void);
float getRawGyroX(void);
float getRawGyroY(void);
float getRawGyro(void);
float getDif(void);
float getLastDif(void);
float getLastGyro(void);
u8 getCheck(void);
float getFineFineGyro(void);
float getChangeOfDif(void);
float getChangeOfChangeOfDif(void);

void get_velocity(float* getX, float* getY, float* getZ);
static void drpy_from_quat(Vec4* q, Vec4* dq, Vec3 *rpy);
static inline void quat_inv(Vec4* t, Vec4* q);
static inline void quat_mu(Vec3* w, float scale, Vec4* q, Vec4* r);
#endif		/* __LCD_RED_H */


