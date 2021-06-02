#ifndef _DEFINE_H
#define _DEFINE_H

/**
	PIN_ON_TOP = 		0
	PIN_ON_LEFT = 	1
	PIN_ON_BOTTOM =	2
	PIN_ON_RIGHT = 	3
*/
#define ORIENTATION_SETTING 0

#ifndef ARM_MATH_CM4
	#define ARM_MATH_CM4
#endif

#define BUTTON_1							BOARD_BTN_1
#define BUTTON_2							BOARD_BTN_2
#define JOYSTICK_N 						JS_BTN_A
#define JOYSTICK_E 						JS_BTN_B
#define JOYSTICK_S 						JS_BTN_C
#define JOYSTICK_W 						JS_BTN_D
#define JOYSTICK_M						JS_BTN_M

#define u8 	uint8_t
#define u16 uint16_t
#define u32 uint32_t
#define u64 uint64_t
#define s8 	int8_t
#define s16 int16_t
#define s32 int32_t
#define s64 int64_t

//typedef int64_t s64;
//typedef uint32_t u64;


// This is the motor id of the current motor
#define THIS_MOTOR MOTOR_4
extern u8 this_motor;

//Enable this if this poor motor somehow does not has a encoder
#define ENCODER_DISABLED false

//This can be 3 or 5
#define MOTOR_CONTROLLER_VERSION 5

#if (MOTOR_CONTROLLER_VERSION != 3) && (MOTOR_CONTROLLER_VERSION != 5)
	#error Motor controller version not supported
#endif

//These two options are for activating the high and low limit switches for lifting/angle tuning application
//When the switch is triggered, it will clear PID, do open loop zero, and emit signal to mainboard
//After that, only commands that move in the opposite direction as the limit switch will be respected.
#define ACTIVATE_HIGH_LIMIT_SWITCH	false
#define ACTIVATE_LOW_LIMIT_SWITCH	false
	
//For Motor controller 3.2 and 5.0, there are on-board pull-down resistors
//Most external encoders are using pull-up. In this case you need to remove the onboard resistors and change this to pull-up.

#define ENCODER_INPUT_MODE GPIO_Mode_IN_FLOATING

//Fuck You! Use 512! By Ben Wong
#define ENCODER_RESOLUTION 1024

#if ENCODER_RESOLUTION==512 
	#define ENCODER_PRESCALER 1
#elif ENCODER_RESOLUTION==1024
	//Else is 1024
	#define ENCODER_PRESCALER 2
#else
	//Other resolution? You are drunk, go home
	#error Hi Today is a good day
#endif


	
//Direction: 1 or -1
#define PWM_DIR (1)
#define ENC_DIR (1)

//Define this if you trust the PID!
//#define I_TRUST_PID
#define I_TRUST_MYSELF
//Note that this is useless now, but I'm keeping it because I wanted more trust and love in this world :)

#define MAX_ORIG_VEL (150000)
#define MAX_ORIG_ACC (900000)
#define SERIAL_PLOTTER  0 
#define COOLTERM		1
#define FEEDBACK_MODE SERIAL_PLOTTER

#define MALF_FLASH_FREQ 42
#define NORM_FLASH_FREQ 256

// f1 gen1 shooter clamp relase when reaching encoder count (range)
// 1 is clip, 0 is release
#define CLAMP_RELEASE 0
#define CLAMP_CLIP 1

#define PID_CAN_FEEDBACK false // on/off CAN pid feedback
#define PNEU_CAN_FEEDBACK false

#define TFT_ENABLE false
#define PENU_ENABLE false
#define DEBUG_STREAM true
#define PENU_SENSOR_ENABLE false

#endif 
