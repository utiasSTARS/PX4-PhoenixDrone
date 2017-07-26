/*
 * ts_loadcell_test.h
 *
 *  Created on: Jul 24, 2017
 *      Author: yilun
 */

#ifndef SYSTEMCMDS_TS_LOADCELL_TEST_TS_LOADCELL_TEST_H_
#define SYSTEMCMDS_TS_LOADCELL_TEST_TS_LOADCELL_TEST_H_

#include <px4_config.h>
#include <px4_getopt.h>
#include <px4_tasks.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#include <arch/board/board.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/ts_actuator_controls.h>
#include <uORB/topics/actuator_armed.h>

#include <lib/LoopTimer/LoopTimer.h>
#include "systemlib/systemlib.h"
#include "systemlib/err.h"

extern "C" __EXPORT int loadcell_test_main(int argc, char* argv[]);
void ts_usage(void);
void arm_disarm_construct_msg(struct actuator_armed_s* msg, bool arm);
int motor_test(int argc, char*arv[]);
int motor_step_test(int argc, char*argv[]);
int servo_step_test(int argc, char*argv[]);
int step_test(int argc, char *argv[]);

extern bool test_should_exit;

#define PI 3.1415926f
#define NUM_SPD_LEVEL 20.f
#define ANGLE_DELTA 2.f
#define NUM_AGL_LEVEL 100.f/ANGLE_DELTA


#endif /* SYSTEMCMDS_TS_LOADCELL_TEST_TS_LOADCELL_TEST_H_ */
