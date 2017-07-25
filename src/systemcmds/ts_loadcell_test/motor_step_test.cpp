/*
 * motor_step_test.cpp
 *
 *  Created on: Jul 24, 2017
 *      Author: yilun
 */
#include "ts_loadcell_test.h"

int motor_step_test(int argc, char*argv[])
{
	if (!argv) errx(1, "Please provide enabled channels");
	bool left_en = false;
	bool right_en = false;
	//printf("%s %s", argv[0], argv[1]);
	if (!strcmp(argv[1], "left")) left_en = true;
	else if (!strcmp(argv[1], "right")) right_en = true;
	else if (!strcmp(argv[1], "both")) left_en = right_en = true;
	else{
		warnx("not recognized command exiting...");
		exit(0);
	}
	test_should_exit = false;
	orb_advert_t _actuator_test_pub = NULL;
	struct ts_actuator_controls_s msg;

	LoopTimer loopTimer(1000000);//Loop Period 1s

	/* init all actuators to zero */
	msg.timestamp = hrt_absolute_time();
	msg.control[0] = msg.control[1] = msg.control[2] = msg.control[3] = 0;
	_actuator_test_pub = orb_advertise(ORB_ID(ts_actuator_controls_0), &msg);

	/* Arm the system */
	struct actuator_armed_s aa;

	arm_disarm_construct_msg(&aa, true);

	orb_advert_t actuator_armed_pub = orb_advertise(ORB_ID(actuator_armed), &aa);

	msg.control[ts_actuator_controls_s::INDEX_RPM_LEFT] = left_en ? 0.05f : 0.0f;
	msg.control[ts_actuator_controls_s::INDEX_RPM_RIGHT] = right_en ? 0.05f : 0.0f;
	orb_publish(ORB_ID(ts_actuator_controls_0), _actuator_test_pub, &msg);
	usleep(1000000);//wait 1ms for the pwm state to get from ramp into on

	for (unsigned i = 0; i < NUM_SPD_LEVEL * 2; i ++) {
		if (i < NUM_SPD_LEVEL){
			msg.control[ts_actuator_controls_s::INDEX_RPM_LEFT] = left_en ? (i + 1) / NUM_SPD_LEVEL : 0.0f;
			msg.control[ts_actuator_controls_s::INDEX_RPM_RIGHT] = right_en ? (i + 1) / NUM_SPD_LEVEL : 0.0f;
		}
		else {
			msg.control[ts_actuator_controls_s::INDEX_RPM_LEFT] = left_en ? (NUM_SPD_LEVEL * 2 - i - 1) / NUM_SPD_LEVEL : 0.0f;
			msg.control[ts_actuator_controls_s::INDEX_RPM_RIGHT] = right_en ? (NUM_SPD_LEVEL * 2 - i - 1) / NUM_SPD_LEVEL : 0.0f;
		}
		msg.control[ts_actuator_controls_s::INDEX_DEGREE_LEFT] = 0.0f;
		msg.control[ts_actuator_controls_s::INDEX_DEGREE_RIGHT] = 0.0f;
		orb_publish(ORB_ID(ts_actuator_controls_0), _actuator_test_pub, &msg);
		loopTimer.wait();
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




