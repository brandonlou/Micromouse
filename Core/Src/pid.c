/*
 * pid.c
 *
 *  Created on: Oct 26, 2019
 *      Author: Caleb Terrill
 */

#include "main.h"
#include "encoders.h"
#include "motors.h"

const float BASE_PWM_X = 0.4;
const float kP_W = 0.0;
const float kD_W = 0.0;

int32_t goalD = 0;
int32_t goalA = 0;
float wErrorOld = 0;

void PID_Reset(void) {

	/* Reset your motors, encoders, and all the variables in this .c file. */
	Motors_Reset();
	Encoders_Reset();
	goalD = 0;
	goalA = 0;
	wErrorOld = 0.0;

}

//float errorBuffer[15]; IMPLEMENT ERROR BUFFER ONLY IF NOT STRAIGHT AND CANNOT FIX MORE
//int counter = 0;

/*Should be called every millisecond to update your motor PWM duty cycles*/
void PID_Update(void) {

	/*This is the meat of your PID.
	 * Here you should be reading from your encoders, finding your error,
	 * calculating PWM duty cycles for your distance and angle correction,
	 * and setting motor PWMs.
	 *
	 * If you are sufficiently close to the goal, you should keep track of how long you
	 * have been in a sufficiently close state. This will be helpful for the PID_Done function.
	 * Perhaps you will only return that the PID is done if you have been sufficiently close for 50 straight
	 * calls of this function... or something like that.*/

	// Get sensor feedback
	int32_t right = GetEncoderRCounts();
	int32_t left  = GetEncoderLCounts();
	int32_t distanceTravelled = (right + left)/2;
	int32_t angleTravelled = left - right;

	// x_controller
	float pwmX = BASE_PWM_X;

	// w_controller
	float wError = goalA - angleTravelled;
	float pwmW = kP_W * wError + kD_W * (wError - wErrorOld) * 0.01; // Is 0.01 dt?
	wErrorOld = wError;

	// Update motor PWM
	MotorR_PWM_Set(pwmX + pwmW);
	MotorL_PWM_Set(pwmX - pwmW);

}

/* Set your PID goal distance. */
void PID_Set_GoalD(int32_t d) {

	goalD = d;

}

/* Set your PID goal angle. */
void PID_Set_GoalA(int32_t a) {

	goalA = a;

}

/* Can be used by your controller to know when the PID is sufficiently close to the goal state. */
int8_t PID_Done(void) {
	/*Recommended: return 1 if the PID is sufficiently close to the goal for a long enough amount of time, else return 0.*/
}
