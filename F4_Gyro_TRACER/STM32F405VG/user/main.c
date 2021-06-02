/**
** H K U S T
** Robocon 2018
**
** F4 Library
*/

#define gyro_auto_update 1

#include "main.h"

#define first_IMU_recal 

#ifdef first_IMU_recal
u8 tempFlag=1;
#else
u8 tempFlag=0;
#endif

int main(void) {
	SystemInit();
	SystemCoreClockUpdate();
	gpio_rcc_init_all();
	led_init();
	encoder_accumulation_Init();
	ticks_init();
	
	
	_delay_ms(10);
	icm20602_init();
	
	
	position_uart_init();
	
		

	u32 last_ticks = 0;
	(void)last_ticks;

	while(1) {
		
		//Main Program
		u32 this_ticks = get_ticks();	
		if (this_ticks == last_ticks) continue;
		

		if(this_ticks > 3000 && tempFlag)
		{
			//imu_recal(3);
			reset_IMU();
			tempFlag = 0;
		}
		//reset_IMU();

		static u32 last_uart_send_ticks = 0;
		if (this_ticks - last_uart_send_ticks >= 100){
			//send_pos();
			last_uart_send_ticks = this_ticks;
		}
			
		static u32 last_uart_ticks = 0;
		if (this_ticks - last_uart_ticks >= 100){
			
			/*
			float xx, yy, zz;
			get_velocity(&xx, &yy, &zz);
			uart_tx_blocking(COM3, "Vx : %f\n", xx);
			uart_tx_blocking(COM3, "Vy : %f\n", yy);
			uart_tx_blocking(COM3, "Vz : %f\n", zz);
			*/
			
			//uart_tx_blocking(COM2, "EncX : %d\n", getEncRawCNT(ENC0));
			//uart_tx_blocking(COM3, "EncY : %d\n", getEncRawCNT(ENC1));
			//uart_tx_blocking(COM1, "CalX : %f\n", getcalX());
			//uart_tx_blocking(COM1, "CalY : %f\n", getcalY());
			//uart_tx_blocking(COM1, "sAngle : %f\n", getSinAngle());
			//uart_tx_blocking(COM1, "cAngle : %f\n", getCosAngle());
			//uart_tx_blocking(COM1, "Angle : %f\n", getOutputA());

			//uart_tx_blocking(COM3, "Gyro : %f\n", getGyroZ());
			//uart_tx_blocking(COM3, "Gyro : %f\n", getGyro());
			
			
			//uart_tx_blocking(COM3, "%f %f %d\n", getcalX(), getcalY(), getOutputA());
			//uart_tx_blocking(COM3, "%d %f %f %f %f %d %f\n", getOutputA(), GgyroAngle(), getFineFineGyro(), getLastGyro(), getGyro(), getCheck(), getRawGyro());

			led_blink(LED_1);
			last_uart_ticks = this_ticks;
		}
		
		static u32 last_led_ticks = 0;
		if (this_ticks - last_led_ticks >= 50){
			led_blink(LED_1);
			last_led_ticks = this_ticks;
		}
		

	}
}


