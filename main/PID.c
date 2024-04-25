#include "PID.h"

void PID_Init(PIDController *pid) {
	pid->integrator[0] = 0.0f;
  	pid->integrator[1] = 0.0f;
	pid->prevError[0]  = 0.0f;
  	pid->prevError[1]  = 0.0f;
	pid->differentiator[0]  = 0.0f;
  	pid->differentiator[1]  = 0.0f;
  	pid->out[0] = 0.0f;
  	pid->out[1] = 0.0f;
	// pid->prevMeasurement = 0.0f;
}

void PID_Update(PIDController *pid, float setpoint[], float measurement[]) { //setpoint and measurement are 1x3: x, y, angle
	float error[3] = {setpoint[0] - measurement[0], setpoint[1] - measurement[1], setpoint[2] - measurement[2]};
	float proportional[2];
	float error_position = sqrt(error[0]*error[0] + error[1]*error[1]);
	float error_angle = atan2(error[1], error[0]) - measurement[2];

	if (error_angle > M_PI) {
		error_angle -= 2 * M_PI;
	} else if (error_angle < -M_PI) {
		error_angle += 2 * M_PI;
	}

	// proportional
	float proportional[2] = {pid->Kp[0] * error_position, pid->Kp[1] * error_angle};
	// integrator
	pid->integrator[0] += pid->Ki[0] * error_position * pid->dt;
	pid->integrator[1] += pid->Ki[1] * error_angle * pid->dt;

	/* Anti-wind-up via integrator clamping */
	// if (pid->integrator > pid->limMaxInt) {

	//     pid->integrator = pid->limMaxInt;

	// } else if (pid->integrator < pid->limMinInt) {

	//     pid->integrator = pid->limMinInt;

	// }

	pid->differentiator[0] = pid->Kd[0] * (error_position - pid->prevError[0]) / pid->dt;
	pid->differentiator[1] = pid->Kd[1] * (error_angle - pid->prevError[1]) / pid->dt;

	/*
	* Compute output and apply limits
	*/
	// pid->out = proportional + pid->integrator + pid->differentiator;

	// if (pid->out > pid->limMax) {

	//     pid->out = pid->limMax;

	// } else if (pid->out < pid->limMin) {

	//     pid->out = pid->limMin;

	// }

	pid->out[0] = proportional[0] + pid->integrator[0] + pid->differentiator[0];
	pid->out[0] = proportional[1] + pid->integrator[1] + pid->differentiator[1];

	pid->prevError[0] = error_position;
	pid->prevError[1] = error_angle;
}
