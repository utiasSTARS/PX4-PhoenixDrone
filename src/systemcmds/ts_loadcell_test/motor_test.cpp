/*
 * motor_test.cpp
 *
 *  Created on: Jul 24, 2017
 *      Author: yilun
 */
#include "ts_loadcell_test.h"

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


