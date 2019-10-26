/*
 * encoders.h
 *
 *  Created on: Jul 26, 2019
 *      Author: Caleb Terrill
 */

#ifndef ENCODERS_H_
#define ENCODERS_H_

#include "main.h"

void Encoders_Reset(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
int32_t GetEncoderRCounts(void);
int32_t GetEncoderLCounts(void);

#endif /* ENCODERS_H_ */
