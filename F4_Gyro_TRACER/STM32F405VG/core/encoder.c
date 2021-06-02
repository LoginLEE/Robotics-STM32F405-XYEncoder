#include "encoder.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"

u32 count;
static int32_t RawX, RawY, xEncoderReading, yEncoderReading;

void encoder_Init(ENCPort encoder){
	
	
	const ENCStruct* _encoder = &ENCPorts[encoder];

	gpio_af_init(_encoder->Pin_A, GPIO_OType_PP, GPIO_PuPd_NOPULL, _encoder->af);	//A
	gpio_af_init(_encoder->Pin_B, GPIO_OType_PP, GPIO_PuPd_NOPULL, _encoder->af);	//B
	if(_encoder->rcc_line == 1){
		RCC_APB1PeriphClockCmd(_encoder->rcc, ENABLE);
	}
	else{
		RCC_APB2PeriphClockCmd(_encoder->rcc, ENABLE);
	}
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 0xffffffff;
	TIM_TimeBaseStructure.TIM_Prescaler = 1;	//1 for 1024count QAQ
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(_encoder->timer, &TIM_TimeBaseStructure);
	TIM_EncoderInterfaceConfig(_encoder->timer, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_Cmd(_encoder->timer, ENABLE);
	
	/*
	//encoder channel init
	const ENCStruct* _encoder = &ENCPorts[encoder];
	
	TIM_DeInit(_encoder->timer);

	gpio_af_init(_encoder->Pin_A, GPIO_OType_PP, GPIO_PuPd_NOPULL, _encoder->af);	//A
	gpio_af_init(_encoder->Pin_B, GPIO_OType_PP, GPIO_PuPd_NOPULL, _encoder->af);	//B
	
	RCC_APB1PeriphClockCmd(_encoder->rcc, ENABLE);
	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 0xffffffff;
	TIM_TimeBaseStructure.TIM_Prescaler = 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(_encoder->timer, &TIM_TimeBaseStructure);
	TIM_EncoderInterfaceConfig(_encoder->timer, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Falling);
	
	_encoder->timer->CNT = (u32)0; 
	TIM_Cmd(_encoder->timer, ENABLE);
	
	*/
	
}


void encoder_accumulation_Init(void){
	
	TIM_DeInit(TIM2);
	TIM_DeInit(TIM5);
	
	//encoder channel init
	encoder_Init(ENC0);
	encoder_Init(ENC1);
	

	/*
	RCC_APB1PeriphClockCmd(encoderAccumulationRCC, ENABLE);
	
	//Setup timer
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_Prescaler = 86 - 1;
  TIM_TimeBaseStructure.TIM_Period = 10-1;
  TIM_TimeBaseInit(encoderAccumulationTIM, &TIM_TimeBaseStructure);
	
	TIM_ITConfig(encoderAccumulationTIM, TIM_IT_Update, ENABLE);
	
	//Setup interrupt
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = encoderAccumulationIRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(encoderAccumulationTIM, ENABLE);	
	
	*/
	
}

uint32_t getEncRawCNT(ENCPort _encoder){	
	return (u32)ENCPorts[_encoder].timer->CNT;
}

void encoderAccumulationIRQ_HANDLER(void){
	if (TIM_GetITStatus(encoderAccumulationTIM, TIM_IT_Update) != RESET){
		
		RawX = getEncRawCNT(ENC1);
		RawY = getEncRawCNT(ENC0);
		
		TIM_ClearITPendingBit(encoderAccumulationTIM, TIM_IT_Update);
	}
}

int32_t getRawEncX(void){return getEncRawCNT(ENC1);}
int32_t getRawEncY(void){return getEncRawCNT(ENC0);}
int32_t getEncX(void){return xEncoderReading;}
int32_t getEncY(void){return yEncoderReading;}


/* end of library */
