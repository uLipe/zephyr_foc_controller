/*
 * Copyright (c) 2022 Linaro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <arm_math.h>
#include <string.h>
#include <zephyr.h>
#include <device.h>
#include <drivers/i2c.h>
#include <drivers/pwm.h>
#include <drivers/gpio.h>
#include <sys/printk.h>
#include "foc_svm.h"
#include "foc_controller.h"

/** AS5600 I2C Encoder constants */
#define AS5600_SLAVE_ADDR 0x36
#define AS5600_ANGLE_REGISTER_H 0x0E
#define AS5600_PULSES_PER_REVOLUTION 4096.0f
#define AS5600_READING_MASK 0xFFF 

/* PWM frequency about 16KHz */
#define PWM_PERIOD_NSEC 62500

const static float encoder_to_degrees_ratio = (360.0f) / AS5600_PULSES_PER_REVOLUTION;

static foc_regulation_callback_t user_callback;
static float pole_pairs;
static bool foc_start = false;

static struct {
	uint16_t raw;
	uint16_t zero_offset;
} encoder_reading;

/* devices used to implement the foc actuation and feeback */
static const struct device *inverter_pwm = DEVICE_DT_GET(DT_NODELABEL(inverter_pwm));
static const struct device *encoder_i2c = DEVICE_DT_GET(DT_NODELABEL(i2c1));
static struct gpio_dt_spec enable_u = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(inverter_enable),gpios,0);
static struct gpio_dt_spec enable_v = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(inverter_enable),gpios,1);
static struct gpio_dt_spec enable_w = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(inverter_enable),gpios,2);

/* Rotor sensor measurement thread. */
static void foc_driver_rotor_position_sample_thread(void *arg1);
K_THREAD_DEFINE(foc_tid, 4096,
		foc_driver_rotor_position_sample_thread, NULL, NULL, NULL,
		-1, 0, 0);

static uint16_t foc_driver_read_encoder(void)
{
	uint16_t raw;
	uint8_t read_data[2] = {0,0};
	uint8_t angle_reg = AS5600_ANGLE_REGISTER_H;

	/* gets the raw position from encoder */
	int err = i2c_write_read(encoder_i2c, 
						AS5600_SLAVE_ADDR,
						&angle_reg,
						1,
						&read_data,
						sizeof(read_data));

	if(err) {
		return 0xFFFF;
	}

	raw = ((uint16_t)read_data[0] << 8) | read_data[1];

	/* wrap to its maximum value */
	if(raw > AS5600_PULSES_PER_REVOLUTION - 1) {
		raw = AS5600_PULSES_PER_REVOLUTION - 1 ;
	}

	return raw;
}

static void foc_driver_get_rotor_position(struct foc_feeback_sensor_data *fdbk)
{
	fdbk->motor_pole_pairs = pole_pairs;

	/* update the encoder reading */
	uint16_t counts = foc_driver_read_encoder();

	/* keeps a ZOH (Zero Order Hold) by just accepting valid measurements to 
	 * reduce position reading distortion
	 */
	if(counts <= (AS5600_PULSES_PER_REVOLUTION - 1)) {
		/* valid readings, update the encoder */
		encoder_reading.raw = counts;
	}

	/* make encoder measures to radians for mech and electrical angle */
	fdbk->rotor_position = (float)((encoder_reading.raw - encoder_reading.zero_offset) &
						 AS5600_READING_MASK) * encoder_to_degrees_ratio;
	fdbk->e_rotor_position = fdbk->motor_pole_pairs * fdbk->rotor_position;
}

static void foc_set_pwms(float dc_a, float dc_b, float dc_c) 
{
	/* saturate the duties */
	if(dc_a > 1.0f) {
		dc_a = 1.0f;
	} else if (dc_a < 0.0f) {
		dc_a = 0.0f;
	}

	if(dc_b > 1.0f) {
		dc_b = 1.0f;
	} else if (dc_b < 0.0f) {
		dc_b = 0.0f;
	}

	if(dc_c > 1.0f) {
		dc_c = 1.0f;
	} else if (dc_c < 0.0f) {
		dc_c = 0.0f;
	}

	/* scale the duty cicle to nanoseconds */
	dc_a *= PWM_PERIOD_NSEC - 1;
	dc_b *= PWM_PERIOD_NSEC - 1;
	dc_c *= PWM_PERIOD_NSEC - 1;

	pwm_set(inverter_pwm, 1 , PWM_PERIOD_NSEC, dc_a, 0);
	pwm_set(inverter_pwm, 2 , PWM_PERIOD_NSEC, dc_b, 0);
	pwm_set(inverter_pwm, 3 , PWM_PERIOD_NSEC, dc_c, 0);
}

static void foc_rotor_align(void) 
{
	float sine;
	float cosine;
	float alpha;
	float beta;
	float inverter_duties[3];

	/* we want a voltage vector with 90 degree of phase */
	arm_sin_cos_f32(90, &sine, &cosine);

	/* use only 20% of the current capacity to avoid heating*/
	arm_inv_park_f32(0.0f,
					0.2f,
					&alpha,
					&beta,
					sine,
					cosine);

	/* convert the alpha-beta frame into SVPWM signals */
	foc_svm_set(alpha, 
			beta,
			&inverter_duties[0],
			&inverter_duties[1],
			&inverter_duties[2]);

	/* set the voltage vector to the inverter and... */
	foc_set_pwms(inverter_duties[0], inverter_duties[1], inverter_duties[2]);

	k_sleep(K_MSEC(500));
}

static int foc_driver_set_encoder_offset(void)
{
	encoder_reading.zero_offset = foc_driver_read_encoder() & AS5600_READING_MASK;
	return 0;
}

/* actual FoC algorithm implementation */
static void foc_controller_do_process(struct foc_controller_payload *foc_data)
{
	struct foc_command_data *command = &foc_data->cmd;
	struct foc_feeback_sensor_data *feed = &foc_data->state;
	float sine;
	float cosine;
	float inverter_duties[3];

	arm_sin_cos_f32(feed->e_rotor_position, &sine, &cosine);

	arm_inv_park_f32(command->voltage_d,
					command->voltage_q,
					&command->voltage_alpha,
					&command->voltage_beta,
					sine,
					cosine);

	/* convert the alpha-beta frame into SVPWM signals */
	foc_svm_set(command->voltage_alpha, 
			command->voltage_beta,
			&inverter_duties[0],
			&inverter_duties[1],
			&inverter_duties[2]);

	/* set the voltage vector to the inverter and... */
	foc_set_pwms(inverter_duties[0], inverter_duties[1], inverter_duties[2]);
}

static void foc_initialize_driver(void)
{
	/* wait the controller to be configured */
	while(!foc_start) {
		k_sleep(K_MSEC(10));
	}

	i2c_configure(encoder_i2c, I2C_MODE_MASTER | I2C_SPEED_FAST);
	foc_set_pwms(0.0f, 0.0f, 0.0f);

	gpio_pin_configure_dt(&enable_u, GPIO_OUTPUT);
	gpio_pin_configure_dt(&enable_v, GPIO_OUTPUT);
	gpio_pin_configure_dt(&enable_w, GPIO_OUTPUT);

	gpio_pin_set_dt(&enable_u, 1);
	gpio_pin_set_dt(&enable_v, 1);
	gpio_pin_set_dt(&enable_w, 1);

}

static void foc_driver_rotor_position_sample_thread(void *arg1)
{	
	struct foc_controller_payload foc_data = {.cmd.voltage_d = 0.0f, .cmd.voltage_d = 0.0f};

	foc_initialize_driver();

	/* perform rotor alignment procedure */
	foc_rotor_align();
	foc_driver_set_encoder_offset();

	for(;;) {
		foc_driver_get_rotor_position(&foc_data.state);

		if(user_callback) {
			user_callback(&foc_data.cmd);
		}

		foc_controller_do_process(&foc_data);
	}
}

int foc_controller_start(float motor_pole_pairs, foc_regulation_callback_t cb)
{
	user_callback = cb;
	pole_pairs = motor_pole_pairs;

	encoder_reading.raw = 0;
	encoder_reading.zero_offset = 0;
	foc_start = true;

	return 0;
}
