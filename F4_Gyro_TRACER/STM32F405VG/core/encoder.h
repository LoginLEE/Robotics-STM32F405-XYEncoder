/**********************************************************
 * @file   	CLJ_f4ENCODER.h
 * @author 	Jonathan Ch	an - RBC Hardware team 2018
 * @contact 21800000 call me
 * @date   	15-July-2018
 * @brief  	This library is for HKUST Robocon Any f4 board
 * @version v1.0
 * @update
 *		2018/07/15 - FUCK uuu
 * @Features
 *		1. no
 * @Rules
 *		1. If you edit this library, please write documentary and leave your name here 
 **********************************************************
 */
 
 
#ifndef	_ENCODER_H
#define	_ENCODER_H
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"

#include "gpio.h"

//encoderAccumulationRCC 100100 Hz

#define encoderAccumulationTIM					TIM3
#define encoderAccumulationRCC					RCC_APB1Periph_TIM3
#define encoderAccumulationIRQ					TIM3_IRQn
#define encoderAccumulationIRQ_HANDLER	TIM3_IRQHandler

/*
Remap means is the timer needs remap
*/
//ENC   TIMER  PinA	 PinB 	RCC                 	line  AF   					
#define ENC_TABLE \
X(ENC0, TIM2,  PA15,  PB3,	RCC_APB1Periph_TIM2, 	1, 		GPIO_AF_TIM2) \
X(ENC1, TIM5,  PA0,  PA1,		RCC_APB1Periph_TIM5, 	1, 		GPIO_AF_TIM5) 

#define X(a, b, c, d, e, f, g) a,
typedef enum {
	ENC_TABLE
} ENCPort;
#undef X

typedef struct{
	TIM_TypeDef* timer;
	const GPIO* Pin_A;
	const GPIO* Pin_B;
	const uint32_t rcc;
	const uint8_t rcc_line;
	const uint8_t af;
} ENCStruct;

#define X(a, b, c, d, e, f, g) {b, &c, &d, e, f, g},
static const ENCStruct ENCPorts[] = {ENC_TABLE};
#undef X


#ifdef __cplusplus
 extern "C" {
#endif 

void encoder_Init(ENCPort _encoder);
uint32_t getEncRawCNT(ENCPort _encoder);
void encoder_accumulation_Init(void);
int32_t getRawEncX(void);
int32_t getRawEncY(void);
int32_t getEncX(void);
int32_t getEncY(void);
#ifdef __cplusplus
}
#endif


/* End of Library */
#endif
