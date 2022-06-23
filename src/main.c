/*
 * Copyright (c) 2022 Linaro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>
#include "foc_controller.h"

#define SAMPLE_MOTOR_POLE_PAIRS 	7
//#define SAMPLE_MOTOR_POLE_PAIRS 	11

static float motor_vq;
static float motor_vd;

void on_foc_regulation(struct foc_command_data * cmd)
{
	/* just update the desired voltage, data publish is handled by
	 * by the driver automatically
	 */
	cmd->voltage_q = motor_vq;
	cmd->voltage_d = motor_vd;
}

void main(void)
{	
	int rc = 0;

	motor_vq = 0.0f;
	motor_vd = 0.0f;

	/* initialize the foc driver first */
	rc = foc_controller_start(SAMPLE_MOTOR_POLE_PAIRS,
							on_foc_regulation);
	if (rc) {
		printk("foc controller init failed!\n");
		return;
	}
}

static int foc_cmd_set(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 3) {
		return -EINVAL;
	}

	motor_vq = strtof(argv[1], NULL);
	motor_vd = strtof(argv[2], NULL);
	return 0;
}

/* Subcommand array for "step_foc" (level 1). */
SHELL_STATIC_SUBCMD_SET_CREATE(
	foc,
	SHELL_CMD(set, NULL, "Set voltages to control the motor", foc_cmd_set),
	SHELL_SUBCMD_SET_END
	);

/* Root command "step_foc" (level 0). */
SHELL_CMD_REGISTER(zephyr_foc, &foc, "Zephyr RTOS FoC sample commands", NULL);
