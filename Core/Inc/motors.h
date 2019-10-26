/*
 * motors.h
 *
 *  Created on: Jul 26, 2019
 *      Author: Caleb Terrill
 */

#ifndef MOTORS_H_
#define MOTORS_H_

#include "main.h"
#include <stdlib.h> // Brandon added this

#define MAX_TIM_COUNTS 3200
#define PWM_MAX 0.8

typedef enum
{
  MOTOR_RF = 0,
  MOTOR_RB = 1,
  MOTOR_LF = 2,
  MOTOR_LB = 3,
}Motor;

void Motors_Reset(void);
void MotorR_PWM_Set(float pwm);
void MotorL_PWM_Set(float pwm);
void PWM_Set(Motor motor, float pwm);



#endif /* MOTORS_H_ */
