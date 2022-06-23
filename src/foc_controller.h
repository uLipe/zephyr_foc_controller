/*
 * Copyright (c) 2022 Linaro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef FOC_CONTROLLER_H__
#define FOC_CONTROLLER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/kernel.h>

/**
 * @brief FoC rotor position sensor data:
 */
struct foc_feeback_sensor_data {
	float motor_pole_pairs;
	float e_rotor_position;
	float rotor_position;
};

/**
 * @brief FoC voltages flow data:
 */
struct foc_command_data {
	float voltage_q;
	float voltage_d;
	float voltage_alpha;
	float voltage_beta;
	float voltage_u;
	float voltage_v;
};

/**
 * @brief FoC data payload descriptor:
 */
struct foc_controller_payload {
	uint32_t timestamp;
	struct foc_feeback_sensor_data state;
	struct foc_command_data cmd;
};

/**
 * @brief FoC measurement reception callback:
 */
typedef void (*foc_regulation_callback_t) (struct foc_command_data *cmd);

/**
 * @brief Initializes the foc driver.
 * 
 * @param cb user callback to be executed each measurement from motor rotor position.
 * 
 * @param dc_link_voltage inverter dc bus voltage in volts.
 *
 * @param motor_pole_pairs number of motor pole pairs.
 * 
 * @return int  0 on success, otherwise a negative error code.
 * 
 */
int foc_controller_start(float motor_pole_pairs, foc_regulation_callback_t cb);

#ifdef __cplusplus
}
#endif

#endif /* STEP_SHELL_DRIVER_H_ */
