/*
 * pid.h
 *
 *  Created on: Oct 26, 2019
 *      Author: Caleb Terrill
 */

#ifndef PID_H_
#define PID_H_

#include "main.h"

void PID_Reset(void);
void PID_Update(void);
void PID_Set_GoalD(int32_t d);
void PID_Set_GoalA(int32_t a);
int8_t PID_Done(void);

#endif /* PID_H_ */
