/*
 * controller.c
 *
 *  Created on: Oct 26, 2019
 *      Author: Caleb Terrill
 */

#include "main.h"
#include "controller.h"
#include "pid.h"

const int xRatio = 10000; // This number corresponds to the number of encoder ticks needed to go forward one cell.
const int wRatio = 5000; // Number of encoder ticks needed to turn 90 degrees.

/*Would recommend you implement this function so that Move_Cells(1) will
 * move your rat 1 cell forward.*/
void Move_Cells(int8_t n) {
	/*You will have to set the distance and angle goals of your PID,
	 * and wait for the error to become sufficiently small before exiting this function.*/

	PID_Reset();
	int xCounts = n * xRatio;
	PID_Set_GoalD(xCounts);

	while(!PID_Done()) {
		// just waiting for it to finish
	}

}

/*Would recommend you implement this function so that Turn(1) will
 * turn your rat 90 degrees in your positive rotation direction.*/
void Turn(int8_t n) {
	/*You will have to set the distance and angle goals of your PID,
	* and wait for the error to become sufficiently small before exiting this function.*/

	PID_Reset();
	int wCounts = n * wRatio;
	PID_Set_GoalD(wCounts);

	while(!PID_Done()) {
		// just waiting for it to finish
	}

}
