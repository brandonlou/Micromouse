/*
 * irs.h
 *
 *  Created on: Jul 27, 2019
 *      Author: Caleb Terrill
 */

#ifndef IRS_H_
#define IRS_H_

#include "main.h"

#define NUM_SAMPLES (uint8_t) 1 //you may want to add a number of samples to read each IR/average these readings, remember the time delay on the IRs is mostly turning them on and off, so leaving them on and measuring a few times isn't bad
#define DELAY_COUNT 0 //adjust this so your IRs stay on long enough for a good reading

typedef enum
{
  IR_RIGHT = 0,
  IR_FRONTR = 1,
  IR_FRONTL = 2,
  IR_LEFT = 3,
}Ir;


uint32_t Irs_Read_Right(void);
uint32_t Irs_Read_FrontR(void);
uint32_t Irs_Read_FrontL(void);
uint32_t Irs_Read_Left(void);
uint32_t Irs_Read(Ir ir);
uint32_t Analog_Read(Ir ir);

#endif /* IRS_H_ */
