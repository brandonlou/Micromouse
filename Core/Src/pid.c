/*
 * pid.c
 *
 *  Created on: Oct 26, 2019
 *      Author: Caleb Terrill
 */

#include "main.h"
#include "encoders.h"
#include "motors.h"

float limitPwm(float pwm);

const float maxSpeed = 0.5;
const float minSpeed = 0.2; // Note: If PID never finishes, increase minSpeed

const float kP_X = 0.02; // TODO: Tune
const float kD_X = 0.005; // TODO: Tune

const float kP_W = 0.04;
const float kD_W = 0.01;

int32_t goalD = 0;
int32_t goalA = 0;

float xErrorOld = 0;
float wErrorOld = 0;

const int threshold = 500;
int counter = 0;

/* Reset motors, encoders, and all necessary variables */
void PID_Reset(void) {

	Motors_Reset();
	Encoders_Reset();
	goalD = 0.0;
	goalA = 0.0;
	xErrorOld = 0.0;
	wErrorOld = 0.0;
	counter = 0;

}

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
	float xError = goalD - distanceTravelled; // or the other way around
	float pwmX = (kP_X * xError) + (kD_X * (xError - xErrorOld)) / 0.01;
	xErrorOld = xError;

	// w_controller
	float wError = angleTravelled - goalA;
	float pwmW = (kP_W * wError) + (kD_W * (wError - wErrorOld)) / 0.01; // why divide by 0.01?
	wErrorOld = wError;

	// Limit motor PWM
	pwmX = limitPwm(pwmX);
	pwmW = limitPwm(pwmW);

	// Calculate right and left PWM value
	float rightSpeed = pwmX + pwmW;
	float leftSpeed = pwmX - pwmW;

	// Increase counter only if rat isn't moving considerably
	if (rightSpeed < minSpeed && leftSpeed < minSpeed) {
		counter++;
	}

	// Move motors
	MotorR_PWM_Set(rightSpeed);
	MotorL_PWM_Set(leftSpeed);

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
/*Recommended: return 1 if the PID is sufficiently close to the goal for a long enough amount of time, else return 0.*/
int8_t PID_Done(void) {

	if(counter > threshold) {
		return 1;

	} else {
		return 0;
	}

}

/* Limits PWM between -1 to 1 */
float limitPwm(float pwm) {

	if(pwm > maxSpeed) {
		return maxSpeed;

	} else if(pwm < -maxSpeed) {
		return -maxSpeed;

	} else {
		return pwm;
	}

}
