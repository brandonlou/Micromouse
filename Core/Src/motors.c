/*
 * motors.c
 *
 *  Created on: Jul 26, 2019
 *      Author: Caleb Terrill
 */

#include "main.h"
#include "motors.h"


void Motors_Reset(void) {
	//Set all the motors' duty cycle to 0%.

	PWM_Set(MOTOR_RF, 0);
	PWM_Set(MOTOR_RB, 0);
	PWM_Set(MOTOR_LF, 0);
	PWM_Set(MOTOR_LB, 0);
}


void MotorR_PWM_Set(float pwm) {
	/*
	 * The input to this function will be positive for forward motion and negative for backwards motion.
	 * Use the PWM_Set function and pass in either MOTOR_RF or MOTOR_RB based on the sign of pwm as the 1st argument,
	 * and pass abs(pwm) as the 2nd argument.
	 * Remember to not have forwards and backwards high at the same time, so when setting one direction,
	 * also set the other direction to 0.
	 */

	PWM_Set((pwm > 0)? MOTOR_RF : MOTOR_RB, abs(pwm));
	PWM_Set((pwm > 0)? MOTOR_RB : MOTOR_RF, 0);

}


void MotorL_PWM_Set(float pwm) {
	/*
	 * The input to this function will be positive for forward motion and negative for backwards motion.
	 * Use the PWM_Set function and pass in either MOTOR_LF or MOTOR_LB based on the sign of pwm as the 1st argument,
	 * and pass abs(pwm) as the 2nd argument.
	 * Remember to not have forwards and backwards high at the same time, so when setting one direction,
	 * also set the other direction to 0.
	 */

	PWM_Set((pwm > 0)? MOTOR_LF : MOTOR_LB, abs(pwm));
	PWM_Set((pwm > 0)? MOTOR_LB : MOTOR_LF, 0);
}

/*
 * Sets the duty cycle for a motor.
 * The motor input can either be MOTOR_RF, MOTOR_RB, MOTOR_LF, or MOTOR_LB.
 * The pwm value is the fraction of the period where the output is high, so 0.5 corresponds to a 50% duty cycle.
 * */
void PWM_Set(Motor motor, float pwm)
{

	TIM_HandleTypeDef *htim_ptr;
	uint32_t channel;

	switch(motor)
	{
		case MOTOR_RF:
			htim_ptr = Get_HTim3_Ptr();
			channel = TIM_CHANNEL_2;
			break;
		case MOTOR_RB:
			htim_ptr = Get_HTim2_Ptr();
			channel = TIM_CHANNEL_3;
			break;
		case MOTOR_LF:
			htim_ptr = Get_HTim4_Ptr();
			channel = TIM_CHANNEL_1;
			break;
		case MOTOR_LB:
			htim_ptr = Get_HTim3_Ptr();
			channel = TIM_CHANNEL_1;
			break;
		default:
			return;
	}

	if(pwm > PWM_MAX)
		pwm = PWM_MAX;
	else if (pwm < 0)
		pwm = 0;

	TIM_OC_InitTypeDef sConfigOC = {0};

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = pwm * MAX_TIM_COUNTS;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(htim_ptr, &sConfigOC, channel);
    HAL_TIM_PWM_Start(htim_ptr, channel);
}
