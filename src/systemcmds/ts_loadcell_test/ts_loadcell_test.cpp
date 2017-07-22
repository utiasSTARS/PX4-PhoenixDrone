/*
 * ts_loadcell_test.c
 *
 *  Created on: Jul 21, 2017
 *      Author: yilun
 */



#include <px4_config.h>
#include <px4_getopt.h>
#include <px4_tasks.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <arch/board/board.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/ts_actuator_controls.h>
#include <uORB/topics/actuator_armed.h>

#include <lib/LoopTimer/LoopTimer.h>
#include "systemlib/systemlib.h"
#include "systemlib/err.h"

#define PI 3.1415926f

extern "C" __EXPORT int loadcell_test_main(int argc, char* argv[]);
void ts_usage(void);
int test(int argc, char *argv[]);

void ts_usage()
{
	errx(1,
		 "usage:\n"
		 "\n");
}


int test(int argc, char *argv[])
{
	orb_advert_t _actuator_test_pub = NULL;
	struct ts_actuator_controls_s msg;

	LoopTimer loopTimer(1000);//Loop Period 1ms

	/* init all actuators to zero */
	msg.timestamp = hrt_absolute_time();
	msg.control[0] = msg.control[1] = msg.control[2] = msg.control[3] = 0;
	_actuator_test_pub = orb_advertise(ORB_ID(ts_actuator_controls_0), &msg);

	/* Arm the system */
	struct actuator_armed_s aa;

	aa.armed = true;
	aa.lockdown = false;
	aa.manual_lockdown = false;
	aa.force_failsafe = false;

	orb_advert_t actuator_armed_pub = orb_advertise(ORB_ID(actuator_armed), &aa);

	uint64_t test_init = hrt_absolute_time();
	float test_elapsed = 0;

	while(test_elapsed < 10.f){ //Test for 10 seconds
		loopTimer.wait();
		test_elapsed = hrt_elapsed_time(&test_init)/1e6f;
		msg.timestamp = hrt_absolute_time();
		msg.control[ts_actuator_controls_s::INDEX_DEGREE_LEFT] = 30 * sin(2 * PI * test_elapsed / 4.f);
		msg.control[ts_actuator_controls_s::INDEX_DEGREE_RIGHT] = -30 * sin(2 * PI * test_elapsed / 4.f);
		orb_publish(ORB_ID(ts_actuator_controls_0), _actuator_test_pub, &msg);
	}




	/* Disarm the system after finish */
	msg.timestamp = hrt_absolute_time();
	msg.control[0] = msg.control[1] = msg.control[2] = msg.control[3] = 0;
	orb_publish(ORB_ID(ts_actuator_controls_0), _actuator_test_pub, &msg);
	usleep(1000);
	aa.armed = false;
	aa.lockdown = false;
	aa.manual_lockdown = false;
	aa.force_failsafe = false;
	orb_publish(ORB_ID(actuator_armed), actuator_armed_pub, &aa);
	orb_unadvertise(actuator_armed_pub);
	orb_unadvertise(_actuator_test_pub);

	return 0;
}


int loadcell_test_main(int argc, char *argv[])
{
	const char *verb = argv[1];

	if (!strcmp(verb, "start")) {
		int pid = px4_task_spawn_cmd("ts_loadcell_test",
									SCHED_DEFAULT,
									SCHED_PRIORITY_DEFAULT,
									1024,
									test,
									NULL);
		if (pid <= 0) {
			errx(1, "failed to start ts loadcell test");
			exit(0);
		}

	}



	return 0;
}

