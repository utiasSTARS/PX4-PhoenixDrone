/*
 * ts_loadcell_test.cpp
 *
 *  Created on: Jul 21, 2017
 *      Author: yilun
 */

#include "ts_loadcell_test.h"

bool test_should_exit = false;

void ts_usage()
{
	errx(1,
		 "usage:\n"
		 "\n");
}

void arm_disarm_construct_msg(struct actuator_armed_s* msg, bool arm)
{
	msg->timestamp = hrt_absolute_time();
	msg->armed = arm ? true : false;
	msg->prearmed = true;
	msg->ready_to_arm = true;
	msg->lockdown = false;
	msg->manual_lockdown = false;
	msg->force_failsafe = false;
	msg->in_esc_calibration_mode = false;
}



int loadcell_test_main(int argc, char *argv[])
{
	const char *verb = argv[1];


	if (!strcmp(verb, "motor-ramp")) {
		int pid = px4_task_spawn_cmd("ts_motor_ramp_test",
									SCHED_DEFAULT,
									SCHED_PRIORITY_DEFAULT,
									1024,
									motor_test,
									(argv && argc > 2) ? (char *const *) &argv[2] : (char *const *) NULL);
		if (pid <= 0) {
			errx(1, "failed to start motor ramp test");
			exit(0);
		}
	}

	if (!strcmp(verb, "motor-step")) {
		int pid = px4_task_spawn_cmd("ts_motor_step_test",
									SCHED_DEFAULT,
									SCHED_PRIORITY_DEFAULT,
									1024,
									motor_step_test,
									(argv && argc > 2) ? (char *const *) &argv[2] : (char *const *) NULL);
		if (pid <= 0) {
			errx(1, "failed to start motor step test");
			exit(0);
		}
	}

	if (!strcmp(verb, "step")) {
		int pid = px4_task_spawn_cmd("ts_loadcell_step_test",
									SCHED_DEFAULT,
									SCHED_PRIORITY_DEFAULT,
									1024,
									step_test,
									(argv && argc > 2) ? (char *const *) &argv[2] : (char *const *) NULL);
		if (pid <= 0) {
			errx(1, "failed to start ts loadcell step test");
			exit(0);
		}
	}

	if (!strcmp(verb, "servo-step")) {
		int pid = px4_task_spawn_cmd("ts_loadcell_servo_step_test",
									SCHED_DEFAULT,
									SCHED_PRIORITY_DEFAULT,
									1024,
									servo_step_test,
									(argv && argc > 2) ? (char *const *) &argv[2] : (char *const *) NULL);
		if (pid <= 0) {
			errx(1, "failed to start ts loadcell servo step test");
			exit(0);
		}
	}

	if (!strcmp(verb, "stop")) {
		test_should_exit = true;
		exit(0);
	}

	return 0;
}

