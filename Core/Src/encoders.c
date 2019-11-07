/*
 * encoders.c
 *
 *  Created on: Jul 26, 2019
 *      Author: Caleb Terrill
 */


#include "encoders.h"
#include "main.h"

/*The volatile keyword tells the compiler not to do any optimizations for these variables.
 * Since these variables can be changed asynchronously (ie. not stemming from anything originally called from the main function),
 * the volatile keyword prevents the compiler from making any optimizations to the code that would be valid if
 * the variable was only changed synchronously, but would ruin your logic if the variable can be changed asynchronously.
 * */
volatile int32_t EncoderRCounts;
volatile int32_t EncoderLCounts;
volatile int8_t EncoderRPrev;
volatile int8_t EncoderLPrev;

/*Resets the encoder counts to 0.*/
void Encoders_Reset(void)
{
	EncoderRCounts = 0;
	EncoderLCounts = 0;

	int8_t RF = HAL_GPIO_ReadPin(EncoderRF_GPIO_Port, EncoderRF_Pin);
	int8_t RB = HAL_GPIO_ReadPin(EncoderRB_GPIO_Port, EncoderRB_Pin);
	EncoderRPrev = (RF<<1) | RB;

	int8_t LF = HAL_GPIO_ReadPin(EncoderLF_GPIO_Port, EncoderLF_Pin);
	int8_t LB = HAL_GPIO_ReadPin(EncoderLB_GPIO_Port, EncoderLB_Pin);
	EncoderLPrev = (LF<<1) | LB;
}

/*
 * Callback function triggered by a rising or falling edge of a signal coming from your encoders.
 * We have implemented the logic for you that will increment or decrement the proper encoder counts based on the change.
 * */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == EncoderRF_Pin || GPIO_Pin == EncoderRB_Pin)
	{
		int8_t RF = HAL_GPIO_ReadPin(EncoderRF_GPIO_Port, EncoderRF_Pin);
		int8_t RB = HAL_GPIO_ReadPin(EncoderRB_GPIO_Port, EncoderRB_Pin);
		int8_t EncoderRCurr = (RF<<1) | RB;

		if(EncoderRCurr!= EncoderRPrev && (EncoderRCurr ^ EncoderRPrev) != 0x3)
			(((EncoderRPrev & 0x1) ^ ((EncoderRCurr & 0x2) >> 1)) == 1)? --EncoderRCounts:++EncoderRCounts;

		EncoderRPrev = EncoderRCurr;
	}
	else if(GPIO_Pin == EncoderLF_Pin || GPIO_Pin == EncoderLB_Pin)
	{
		int8_t LF = HAL_GPIO_ReadPin(EncoderLF_GPIO_Port, EncoderLF_Pin);
		int8_t LB = HAL_GPIO_ReadPin(EncoderLB_GPIO_Port, EncoderLB_Pin);
		int8_t EncoderLCurr = (LF<<1) | LB;

		if(EncoderLCurr!= EncoderLPrev && (EncoderLCurr ^ EncoderLPrev) != 0x3)
			(((EncoderLPrev & 0x1) ^ ((EncoderLCurr & 0x2) >> 1)) == 1)? --EncoderLCounts:++EncoderLCounts;

		EncoderLPrev = EncoderLCurr;
	}
}

/* Allows you to get the right encoder counts from outside this c file. */
int32_t GetEncoderRCounts(void) {

	return EncoderRCounts;

}

/* Allows you to get the left encoder counts from outside this c file. */
int32_t GetEncoderLCounts(void) {

	return EncoderLCounts;

}
