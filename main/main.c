#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "string.h"
#include "PID.h"
#include <projdefs.h>

#define RXp2 7
#define TXp2 8
#define BUF_SIZE 1024
#define WIDTH 0.01
// Proportional Gains
#define KP_POSITION 0.1
#define KP_ANGLE    0.05
// Integral Gains
#define KI_POSITION 0.01
#define KI_ANGLE    0.005
// Derivative Gains
#define KD_POSITION 0.05
#define KD_ANGLE    0.025
#define TIME_STEP 0.1
// Number of received variables
#define NUM_VARIABLES 2


void get_wheel_velocity(double v, double w, double *v_left, double *v_right) {
	// Calculate the left wheel velocity
	*v_left = v + (WIDTH * w / 2.0);
	// Calculate the right wheel velocity
	*v_right = v - (WIDTH * w / 2.0);
}

void app_main(void)
{
	uart_config_t uart_config = {
		.baud_rate = 115200,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_APB,
	};

	// Configure the UART1 parameters
	uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
	uart_param_config(UART_NUM_1, &uart_config);
	uart_set_pin(UART_NUM_1, RXp2, TXp2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

	// Initialize PID controller
	PIDController pid;
	PID_Init(&pid);
	// Kp
	pid.Kp[0] = KP_POSITION;
	pid.Kp[1] = KP_ANGLE;
	// Ki
	pid.Ki[0] = KI_POSITION;
	pid.Ki[1] = KI_ANGLE;
	// Kd
	pid.Kd[0] = KD_POSITION;
	pid.Kd[1] = KD_ANGLE;
	// time step
	pid.dt = TIME_STEP;

	// Buffer for input data
	uint8_t* data = (uint8_t*) malloc(BUF_SIZE);
	// Initialize setpoints and left and right wheel velocity
	float setpoint[NUM_VARIABLES], v_left, v_right;
	while (1) {
		int len = uart_read_bytes(UART_NUM_1, data, 8, pdMS_TO_TICKS(20));
		if (len == 8) {
		memcpy(setpoint, data, sizeof(data));

		PID_Update(&pid, setpoint, pid.out);
		get_wheel_velocity(pid.out[0], pid.out[1], &v_left, &v_right);
		}
	}
	free(data);
}
