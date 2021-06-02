#ifndef	_POSITIONING_H
#define	_POSITIONING_H
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"

#include "gpio.h"
#include "math.h"
#include "encoder.h"
#include "icm20602.h"
#include "uart.h"



void send_pos(void);
void position_uart_init(void);

void position_update(void);
void reset_possition(void);
float getSinAngle(void);
float getCosAngle(void);
float getcalX(void);
float getcalY(void);
float getxError(void);

float getyError(void);

float getchangeOfX(void);
float getchangeOfA(void);
float getchangeOfY(void);
float getxError(void);
float getyError(void);
s16 getOutputX(void);
s16 getOutputY(void);
s16 getOutputA(void);
float GgyroAngle(void);

#endif
