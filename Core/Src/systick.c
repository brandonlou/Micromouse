/*
 * systick.c
 *
 *  Created on: Oct 26, 2019
 *      Author: Caleb Terrill
 */

#include "main.h"
#include "pid.h"

/* Will be called every millisecond */
void SysTick_Function(void) {

	/* Do whatever you wish to be done every millisecond. Maybe... update your PID? */
	PID_Update();

}
