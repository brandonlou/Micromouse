/*
 * irs.c
 *
 *  Created on: Jul 27, 2019
 *      Author: Caleb Terrill
 */

#include "irs.h"
#include "main.h"


#pragma GCC push_options
#pragma GCC optimize ("O0")
void Delay() { //this is how you can do a delay, modify the DELAY_COUNT in the .h file if your readings aren't consistent
	volatile uint32_t counter = DELAY_COUNT;
	while(counter--);
}
#pragma GCC pop_options


uint32_t Irs_Read_Right(void) //these functions should use IRs_Read to read the right IR (just one line)
{
	return Irs_Read(IR_RIGHT);
}


uint32_t Irs_Read_FrontR(void)
{
	return Irs_Read(IR_FRONTR);
}


uint32_t Irs_Read_FrontL(void)
{
	return Irs_Read(IR_FRONTL);
}


uint32_t Irs_Read_Left(void)
{
	return Irs_Read(IR_LEFT);
}

// this function should handle turning on an IR emitter with the correct port and pin (depending on which IR is passed to it),
// it should then call Analog_Read to read the IR value from the right IR, and finally turn off the IR emitter.
// don't forget to delay between turning on, reading, and turning off your IRs!
uint32_t Irs_Read(Ir ir)
{

	GPIO_TypeDef* port;
	uint16_t pin;

	switch(ir) {
		case IR_RIGHT:
			port = IrRightE_GPIO_Port;
			pin = IrRightE_Pin;
			break;

		case IR_FRONTR:
			port = IrFrontRE_GPIO_Port;
			pin = IrFrontRE_Pin;
			break;

		case IR_FRONTL:
			port = IrFrontLE_GPIO_Port;
			pin = IrFrontLE_Pin;
			break;

		case IR_LEFT:
			port = IrLeftE_GPIO_Port;
			pin = IrLeftE_Pin;
			break;
	}

	// Turn on LED
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);

	// Wait a bit...
	Delay();

	// Get reading and turn of LED
	uint32_t value = Analog_Read(ir);
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);

	// Wait again...
	Delay();

	return value;
}

uint32_t Analog_Read(Ir ir)
{
	uint32_t channel;

	switch(ir)
	{
		case IR_RIGHT: //this picks the IR direction to choose the right ADC.
			channel = ADC_CHANNEL_0;
			break;
		case IR_FRONTR:
			channel = ADC_CHANNEL_4;
			break;
		case IR_FRONTL:
			channel = ADC_CHANNEL_11;
			break;
		case IR_LEFT:
			channel = ADC_CHANNEL_10;
			break;
		default:
			return 0;
	}

	ADC_ChannelConfTypeDef sConfig = {0}; //this initializes the IR ADC [Analog to Digital Converter]
	ADC_HandleTypeDef *hadc1_ptr = Get_HAdc1_Ptr(); //this is a pointer to your hal_adc, you will need this when you call HAL_ADC_PollForConversion
	//this pointer will also be used to read the analog value, val = HAL_ADC_GetValue(hadc1_ptr);

	sConfig.Channel = channel;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	HAL_ADC_ConfigChannel(hadc1_ptr, &sConfig);

	HAL_ADC_Start(hadc1_ptr); //this starts the ADC

	uint32_t sum = 0;
	uint8_t measurements = 0;

	while(measurements < NUM_SAMPLES) //this takes multiple measurements
	{
		if(HAL_ADC_PollForConversion(hadc1_ptr,HAL_MAX_DELAY) == HAL_OK) //this makes sure the ADC has recieved a value
		{
			sum += HAL_ADC_GetValue(hadc1_ptr); // this is actually doing the reading
			++measurements;
		}
	}

	HAL_ADC_Stop(hadc1_ptr); //this stops the ADC
	return sum/NUM_SAMPLES;
}
