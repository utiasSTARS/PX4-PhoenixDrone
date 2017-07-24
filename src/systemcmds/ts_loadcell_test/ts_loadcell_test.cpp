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
#include <math.h>

#include <arch/board/board.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/ts_actuator_controls.h>
#include <uORB/topics/actuator_armed.h>

#include <lib/LoopTimer/LoopTimer.h>
#include "systemlib/systemlib.h"
#include "systemlib/err.h"

#define PI 3.1415926f
#define NUM_SPD_LEVEL 20.f
#define ANGLE_DELTA 2.f
#define NUM_AGL_LEVEL 100.f/ANGLE_DELTA

extern "C" __EXPORT int loadcell_test_main(int argc, char* argv[]);
void ts_usage(void);
void arm_disarm_construct_msg(struct actuator_armed_s* msg, bool arm);
int test(int argc, char *argv[]);
int motor_test(int argc, char*arv[]);
int step_test(int argc, char *argv[]);
static bool test_should_exit = false;

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

int step_test(int argc, char *argv[])
{
	test_should_exit = false;
	orb_advert_t _actuator_test_pub = NULL;
	struct ts_actuator_controls_s msg;

	LoopTimer loopTimer(500000);//Loop Period 0.5s

	/* init all actuators to zero */
	msg.timestamp = hrt_absolute_time();
	msg.control[0] = msg.control[1] = msg.control[2] = msg.control[3] = 0;
	_actuator_test_pub = orb_advertise(ORB_ID(ts_actuator_controls_0), &msg);

	/* Arm the system */
	struct actuator_armed_s aa;

	arm_disarm_construct_msg(&aa, true);

	orb_advert_t actuator_armed_pub = orb_advertise(ORB_ID(actuator_armed), &aa);



	for (unsigned i = 0; i < NUM_SPD_LEVEL; i ++) {
		msg.control[ts_actuator_controls_s::INDEX_RPM_LEFT] = (i + 1) / NUM_SPD_LEVEL;
		msg.control[ts_actuator_controls_s::INDEX_RPM_RIGHT] = (i + 1) / NUM_SPD_LEVEL;
		//Publish to let motor spin to desired target first
		orb_publish(ORB_ID(ts_actuator_controls_0), _actuator_test_pub, &msg);
		usleep(500000);//sleep 500ms waiting for motor response
		for (unsigned j = 0; j <= NUM_AGL_LEVEL; j ++) {
			float angle = -50.f + j *ANGLE_DELTA;
			msg.control[ts_actuator_controls_s::INDEX_DEGREE_LEFT] = (i % 2) ? angle : -1.f * angle;
			msg.control[ts_actuator_controls_s::INDEX_DEGREE_RIGHT] = (i % 2) ? angle : -1.f * angle;
			msg.timestamp = hrt_absolute_time();
			if (test_should_exit) goto stop;
			orb_publish(ORB_ID(ts_actuator_controls_0), _actuator_test_pub, &msg);
			loopTimer.wait();
		}
	}



	/* Disarm the system after finish */
stop:
	msg.timestamp = hrt_absolute_time();
	msg.control[0] = msg.control[1] = msg.control[2] = msg.control[3] = 0;
	orb_publish(ORB_ID(ts_actuator_controls_0), _actuator_test_pub, &msg);
	usleep(1000);

	arm_disarm_construct_msg(&aa, false);
	orb_publish(ORB_ID(actuator_armed), actuator_armed_pub, &aa);

	orb_unadvertise(actuator_armed_pub);
	orb_unadvertise(_actuator_test_pub);

	return 0;
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

	while(test_elapsed < 30.f){ //Test for 10 seconds
		loopTimer.wait();
		test_elapsed = hrt_elapsed_time(&test_init)/1e6f;
		msg.timestamp = hrt_absolute_time();
		msg.control[ts_actuator_controls_s::INDEX_DEGREE_LEFT] = 50 * sin(2 * PI * test_elapsed / 60.f);
		msg.control[ts_actuator_controls_s::INDEX_DEGREE_RIGHT] = -50 + 100 * test_elapsed / 30.f;// -50 * sin(2 * PI * test_elapsed / 40.f);
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

int motor_test(int argc, char*arv[])
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

	while(test_elapsed < 20.f){ //Test for 20 seconds
		loopTimer.wait();
		test_elapsed = hrt_elapsed_time(&test_init)/1e6f;
		msg.timestamp = hrt_absolute_time();
		msg.control[ts_actuator_controls_s::INDEX_RPM_LEFT] = 1.f - (fabsf(test_elapsed - 10.f) / 10.f);
		msg.control[ts_actuator_controls_s::INDEX_RPM_RIGHT] = 0;
		msg.control[ts_actuator_controls_s::INDEX_DEGREE_LEFT] = 0;
		msg.control[ts_actuator_controls_s::INDEX_DEGREE_RIGHT] = 0;
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

	if (!strcmp(verb, "motor-calibration")) {
		int pid = px4_task_spawn_cmd("ts_motor_test",
				SCHED_DEFAULT,
				SCHED_PRIORITY_DEFAULT,
				1024,
				motor_test,
				NULL);
		if (pid <= 0) {
			errx(1, "failed to start motor test");
			exit(0);
		}
	}

	if (!strcmp(verb, "step")) {
		int pid = px4_task_spawn_cmd("ts_loadcell_step_test",
									SCHED_DEFAULT,
									SCHED_PRIORITY_DEFAULT,
									1024,
									step_test,
									NULL);
		if (pid <= 0) {
			errx(1, "failed to start ts loadcell step test");
			exit(0);
		}
	}

	if (!strcmp(verb, "stop")) {
		test_should_exit = true;
		exit(0);
	}

	return 0;
}

