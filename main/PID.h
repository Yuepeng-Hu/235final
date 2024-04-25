#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
// #ifndef PID_CONTROLLER_H
// #define PID_CONTROLLER_H

typedef struct {

  	/* Controller gains */
  	float Kp[2];
  	float Ki[2];
  	float Kd[2];

	/* Derivative low-pass filter time constant */
	// float tau;

	/* Output limits */
	// float limMin;
	// float limMax;
	
	/* Integrator limits */
	// float limMinInt;
	// float limMaxInt;

	/* Sample time (in seconds) */
	float dt;

	/* Controller "memory" */
	float integrator[2];
	float prevError[2];			/* Required for integrator */
	float differentiator[2];
	// float prevMeasurement[2];		/* Required for differentiator */

	/* Controller output */
	float out[2];

} PIDController;

void PID_Init(PIDController *pid);
void PID_Update(PIDController *pid, float setpoint[], float measurement[]);

// #endif
