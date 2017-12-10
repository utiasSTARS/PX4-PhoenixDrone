/**
 * @file ts_att_control_main.cpp
 * Tailsitter attitude controller.
 *
 * @author Xintong Du	<xintong.du@mail.utoronto.ca>
 *
 */
#include <iostream>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <systemlib/px4_macros.h>
#include <lib/mathlib/mathlib.h>
#include "ts_path_planner.h"

/**
 * Tailsitter attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int ts_path_planner_main(int argc, char *argv[]);

namespace ts_path_planner
{
TailsitterPathPlanner	*g_planner;
}

TailsitterPathPlanner::TailsitterPathPlanner():
		_task_should_exit(false),
		_planner_task(-1),
		_setpoint_updated(false),
		_control_mode_updated(false),
		_v_control_mode_pub(nullptr),
		_position_setpoint_pub(nullptr),
		_control_mode{},
		_pos_sp_triplet{}
{
}

TailsitterPathPlanner::~TailsitterPathPlanner()
{
	if (_planner_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_planner_task);
				break;
			}
		} while (_planner_task != -1);
	}

	ts_path_planner::g_planner = nullptr;
}


int
TailsitterPathPlanner::start(){

	ASSERT(_planner_task == -1);

	_planner_task = px4_task_spawn_cmd("ts_att_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1500,
					   (px4_main_t)&TailsitterPathPlanner::task_main_trampoline,
					   nullptr);

	if (_planner_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

void
TailsitterPathPlanner::task_main_trampoline(int argc, char *argv[])
{
	ts_path_planner::g_planner->task_main();
}

void
TailsitterPathPlanner::task_main()
{
	_reset_control_mode();

	while(!_task_should_exit){
		if (_setpoint_updated){

			_publish_setpoint();
			_setpoint_updated = false;
		}
		_publish_control_mode();
	}
}

void
TailsitterPathPlanner::_publish_setpoint()
{
	if (_position_setpoint_pub != nullptr) {
		orb_publish(ORB_ID(position_setpoint_triplet), _position_setpoint_pub, &_pos_sp_triplet);

	} else {
		_position_setpoint_pub = orb_advertise(ORB_ID(position_setpoint_triplet), &_pos_sp_triplet);
	}
}

void
TailsitterPathPlanner::_publish_control_mode()
{
	if(_v_control_mode_pub != nullptr){
		_control_mode.timestamp = hrt_absolute_time();
		orb_publish(ORB_ID(offboard_control_mode), _v_control_mode_pub, &_control_mode);
	}
	else{
		_v_control_mode_pub = orb_advertise(ORB_ID(offboard_control_mode), &_control_mode);
	}
}

void
TailsitterPathPlanner::update_pos_setpoint(int argc, char*argv[]){

	if(argc < 2){
		warnx("usage: ts_path_planner pub {att|acc}");
	}
	else{
		if(!strcmp(argv[0], "acc")){
			if (_control_mode.ignore_acceleration_force){

				_control_mode.ignore_acceleration_force = false;
				_control_mode.ignore_position = true;
			}



			_pos_sp_triplet.previous = _pos_sp_triplet.current;
			_pos_sp_triplet.current.valid = true;
			_pos_sp_triplet.current.position_valid = false;
			_pos_sp_triplet.current.velocity_valid = false;
			_pos_sp_triplet.current.alt_valid = false;
			_pos_sp_triplet.current.yawspeed_valid = false;

			_pos_sp_triplet.current.acceleration_valid =  true;
			_pos_sp_triplet.current.yaw_valid = true;
			PX4_INFO("Command Acc: %s, %s, %s, %s", argv[1],
													argv[2],
													argv[3],
													argv[4]);
			_pos_sp_triplet.current.a_x = strtof(argv[1], 0);
			_pos_sp_triplet.current.a_y = strtof(argv[2], 0);
			_pos_sp_triplet.current.a_z = strtof(argv[3], 0);
			_pos_sp_triplet.current.yaw = strtof(argv[4], 0);
			usleep(1e6);
			_setpoint_updated = true;

		}

		if(!strcmp(argv[0], "pos")){
			if (_control_mode.ignore_position){

				_control_mode.ignore_position = false;
				_control_mode.ignore_acceleration_force = true;
			}



			_pos_sp_triplet.previous = _pos_sp_triplet.current;
			_pos_sp_triplet.current.valid = true;
			_pos_sp_triplet.current.position_valid = true;
			_pos_sp_triplet.current.velocity_valid = false;
			_pos_sp_triplet.current.alt_valid = true;
			_pos_sp_triplet.current.yawspeed_valid = false;

			_pos_sp_triplet.current.acceleration_valid =  false;
			_pos_sp_triplet.current.yaw_valid = true;
			PX4_INFO("Command Pos: %s, %s, %s, %s", argv[1],
													argv[2],
													argv[3],
													argv[4]);
			_pos_sp_triplet.current.x = strtof(argv[1], 0);
			_pos_sp_triplet.current.y = strtof(argv[2], 0);
			_pos_sp_triplet.current.z = strtof(argv[3], 0);
			_pos_sp_triplet.current.yaw = strtof(argv[4], 0);
			usleep(1e6);
			_setpoint_updated = true;

		}
	}
}
void
TailsitterPathPlanner::update_control_mode(int argc, char* argv[])
{
	_reset_control_mode();
	if(!strcmp(argv[0], "acc"))
		_control_mode.ignore_acceleration_force = false;
	if(!strcmp(argv[0], "pos"))
		_control_mode.ignore_position = false;
}


void
TailsitterPathPlanner::_reset_control_mode()
{
	_control_mode.ignore_thrust = true;
	_control_mode.ignore_attitude = true;
	_control_mode.ignore_bodyrate = true;
	_control_mode.ignore_position = false;
	_control_mode.ignore_velocity = true;
	_control_mode.ignore_acceleration_force = true;
	_control_mode.ignore_alt_hold = true;
}

int ts_path_planner_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: ts_path_planner {start|stop|status|set}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (ts_path_planner::g_planner != nullptr) {
			warnx("already running");
			return 1;
		}

		ts_path_planner::g_planner = new TailsitterPathPlanner;

		if (ts_path_planner::g_planner == nullptr) {
			warnx("alloc failed");
			return 1;
		}


		if (OK != ts_path_planner::g_planner->start()) {
			delete ts_path_planner::g_planner;
			ts_path_planner::g_planner = nullptr;
			warnx("start failed");
			return 1;
		}
		warnx("Planner started");

		return 0;
	}


	if (!strcmp(argv[1], "stop")) {
		if (ts_path_planner::g_planner == nullptr) {
			warnx("not running");
			return 1;
		}

		delete ts_path_planner::g_planner;
		ts_path_planner::g_planner = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (ts_path_planner::g_planner) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	if(!strcmp(argv[1], "pub")){
		if(ts_path_planner::g_planner == nullptr){
			warnx("not running");
			return 1;
		}
		else{
			ts_path_planner::g_planner->update_pos_setpoint(argc-2, argv+2);

			}

		return 0;
	}

	if(!strcmp(argv[1], "control")){
		if(ts_path_planner::g_planner == nullptr){
			warnx("not running");
			return 1;
		}
		else{
			ts_path_planner::g_planner->update_control_mode(argc-2, argv+2);

			}

		return 0;
	}

	warnx("unrecognized command");
	return 1;
}
