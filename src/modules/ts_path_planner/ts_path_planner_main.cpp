/**
 * @file ts_att_control_main.cpp
 * Tailsitter attitude controller.
 *
 * @author Xintong Du	<xintong.du@mail.utoronto.ca>
 *
 */
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
#include "systemlib/param/param.h"
#include <lib/mathlib/mathlib.h>
#include <platforms/px4_workqueue.h>
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
		_circle_traj_generator(-1),
		_setpoint_updated(false),
		_control_mode_updated(false),
		_v_control_mode_pub(nullptr),
		_position_setpoint_pub(nullptr),
		_params_sub(-1),
		_local_pos_sub(-1),
		_v_lpos_sp_sub(-1),
		_control_mode{},
		_pos_sp_triplet{},
		_local_pos{},
		_local_pos_sp{},
		_work{},
		_looptimer(2e4),
		_looptimer_circle(2e4)
{
	_params.cruise_speed_max.zero();
	_params.cruise_speed = 0;
	_param_handles.z_cruise_speed = param_find("TS_CRUISE_MAX_Z");
	_param_handles.xy_cruise_speed = param_find("TS_CRUISE_MAX_XY");
	_param_handles.cruise_speed = param_find("TS_CRUISE_SPEED");
	params_update(true);

	_waypoint.start_time = hrt_absolute_time();
	_waypoint.end_point.zero();
	_waypoint.direction.zero();
	_waypoint.start_point.zero();
	_waypoint.velocity.zero();
	_waypoint.yaw = 0;

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

	_planner_task = px4_task_spawn_cmd("ts_path_planner",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1000,
					   (px4_main_t)&TailsitterPathPlanner::task_main_trampoline,
					   nullptr);

	if (_planner_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

void
TailsitterPathPlanner::star_generator_trampoline(int argc, char *argv[])
{
	ts_path_planner::g_planner->star_generator_main();
}

void
TailsitterPathPlanner::task_main_trampoline(int argc, char *argv[])
{
	ts_path_planner::g_planner->task_main();
}

void
TailsitterPathPlanner::task_main()
{
	reset_control_mode();
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_v_lpos_sp_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
	work_queue(HPWORK, &_work, (worker_t)&TailsitterPathPlanner::publish_control_mode_trampoline, this, 0);
	while(!_task_should_exit){
		_looptimer.wait();
		params_update(true);
		poll_subscriptions();
		if (_setpoint_updated){

				float dt = (hrt_absolute_time() - _waypoint.start_time)/1e6f;
				math::Vector<3> next_point = _waypoint.start_point + _waypoint.direction * dt * _waypoint.speed;
				math::Vector<3> velocity = _waypoint.velocity;

				if((next_point - _waypoint.end_point).length()< 0.01f){
					_setpoint_updated = false;
					next_point = _waypoint.end_point;
					velocity.zero();
				}

				_pos_sp_triplet.timestamp = hrt_absolute_time();
				_pos_sp_triplet.previous = _pos_sp_triplet.current;
				_pos_sp_triplet.current.valid = true;
				_pos_sp_triplet.current.position_valid = true;
				_pos_sp_triplet.current.velocity_valid = true;
				_pos_sp_triplet.current.velocity_frame = position_setpoint_s::VELOCITY_FRAME_LOCAL_NED;
				_pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_OFFBOARD;
				_pos_sp_triplet.current.alt_valid = false;
				_pos_sp_triplet.current.yawspeed_valid = false;
//				velocity.zero();
//				velocity.normalize();
//				velocity = velocity * 0.3f;
				_pos_sp_triplet.current.acceleration_valid =  false;
				_pos_sp_triplet.current.yaw_valid = true;
				_pos_sp_triplet.current.x = next_point(0);
				_pos_sp_triplet.current.y = next_point(1);
				_pos_sp_triplet.current.z = next_point(2);
				_pos_sp_triplet.current.vx = velocity(0);
				_pos_sp_triplet.current.vy = velocity(1);
				_pos_sp_triplet.current.vz = velocity(2);
				_pos_sp_triplet.current.yaw = _waypoint.yaw;
				_pos_sp_triplet.current.timestamp = hrt_absolute_time();
//				printf("Next point %f, %f, %f\n", (double) next_point(0),(double) next_point(1),(double) next_point(2));
//				printf("Velocity %f, %f, %f\n", (double) velocity(0),(double) velocity(1),(double) velocity(2));


			publish_setpoint();
		}
	}

}


void
TailsitterPathPlanner::publish_setpoint()
{
	if (_position_setpoint_pub != nullptr) {
		orb_publish(ORB_ID(position_setpoint_triplet), _position_setpoint_pub, &_pos_sp_triplet);

	} else {
		_position_setpoint_pub = orb_advertise(ORB_ID(position_setpoint_triplet), &_pos_sp_triplet);
	}
}

void
TailsitterPathPlanner::publish_control_mode_trampoline(void *arg)
{
	TailsitterPathPlanner *dev = reinterpret_cast<TailsitterPathPlanner *>(arg);
	dev->publish_control_mode();
}

void
TailsitterPathPlanner::publish_control_mode()
{
	if(_v_control_mode_pub != nullptr){
		_control_mode.timestamp = hrt_absolute_time();
		orb_publish(ORB_ID(offboard_control_mode), _v_control_mode_pub, &_control_mode);
	}
	else{
		_v_control_mode_pub = orb_advertise(ORB_ID(offboard_control_mode), &_control_mode);
	}
	work_queue(HPWORK, &_work, (worker_t)&TailsitterPathPlanner::publish_control_mode_trampoline, this, USEC2TICK(1e5));
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
			usleep(2e6);
			//_setpoint_updated = true;
			publish_setpoint();
			//publish_setpoint();

		}

		if(!strcmp(argv[0], "pos")){
			if (_control_mode.ignore_position){
				_control_mode.ignore_position = false;
				_control_mode.ignore_acceleration_force = true;
			}
			publish_waypoint(strtof(argv[1], 0), 
							 strtof(argv[2], 0),
							 strtof(argv[3], 0), 
							 strtof(argv[4], 0) / 180.f * 3.14f);
		}

		if(!strcmp(argv[0], "circle")) {
			if(argc != 7) {
				PX4_WARN("Require 7 parameters for circle");
				return;
			}
			float centerX = strtof(argv[1], 0);
			float centerY = strtof(argv[2], 0);
			float radius = strtof(argv[3], 0);
			float zAmplitude = strtof(argv[4], 0);
			float yaw = strtof(argv[5], 0);
			float revs = strtof(argv[6], 0);
			circle_trajectory(centerX, centerY, radius, zAmplitude, yaw/180.f*3.14159f, revs*6.28f);
		}

		if (!strcmp(argv[0], "star")) {
			if (argc != 3) { PX4_WARN("Require x parametres for star traj"); return;}

			_circle_traj_generator = px4_task_spawn_cmd("star_trajectory_generator",
							   SCHED_DEFAULT,
							   SCHED_PRIORITY_MAX - 5,
							   1000,
							   (px4_main_t)&TailsitterPathPlanner::star_generator_trampoline,
							   nullptr);
		}
	}
}

/*
 * Publishes the next position setpoint given circle parameters.
 * centerX, centerY are in NED coordinates.
 * Rotates rev radians
 * Assumes the tailsitter is already on the circumference of the circle.
 * Goes clockwise.
 */
void TailsitterPathPlanner::circle_trajectory(float centerX, float centerY,
		float radius, float zAmplitude, float yaw, float revs) {
	float speed = _params.cruise_speed;
	float w = speed / radius;
	math::Vector<3> centerVector(centerX, centerY, _local_pos_sp.z);
	math::Vector<3> radiusVector(radius, 0, 0); // (cos(wt), sin(wt)), t = 0
	// "current" desired position of TS
	// This vector will rotate over time.
	math::Vector<3> desiredPosition = centerVector + radiusVector;
	// First head to start
	publish_waypoint(desiredPosition(0), desiredPosition(1), desiredPosition(2), yaw);
	//while(_setpoint_updated) {
		usleep(5e6);
	//}; // wait until ts is at location
	// Add 5s wait to allow it to stabilize
	usleep(5e6);
	float t = 0; // time in seconds
	hrt_abstime start_time = hrt_absolute_time();
	while(w*t < revs) {
		_looptimer_circle.wait();
		t = hrt_elapsed_time(&start_time)/1e6f;
		// Rotate the radius vector based on the elapsed time and desired speed and re-compute the currentDesiredVector
		poll_subscriptions();
		radiusVector(0) = radius*(float)cos(w*t);
		radiusVector(1) = radius*(float)sin(w*t);
		radiusVector(2) = zAmplitude*(float)sin(w*t);
		desiredPosition = centerVector + radiusVector;
		math::Vector<3> velocity(-w*(float)sin(w*t), w*(float)cos(w*t), 0);
		if(w*t >= revs){
			velocity.zero();
		}
		_pos_sp_triplet.timestamp = hrt_absolute_time();
		_pos_sp_triplet.previous = _pos_sp_triplet.current;
		_pos_sp_triplet.current.valid = true;
		_pos_sp_triplet.current.position_valid = true;
		_pos_sp_triplet.current.velocity_valid = true;
		_pos_sp_triplet.current.velocity_frame = position_setpoint_s::VELOCITY_FRAME_LOCAL_NED;
		_pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_OFFBOARD;
		_pos_sp_triplet.current.alt_valid = false;
		_pos_sp_triplet.current.yawspeed_valid = false;
//				velocity.zero();
//				velocity.normalize();
//				velocity = velocity * 0.3f;
		_pos_sp_triplet.current.acceleration_valid =  false;
		_pos_sp_triplet.current.yaw_valid = true;
		_pos_sp_triplet.current.x = desiredPosition(0);
		_pos_sp_triplet.current.y = desiredPosition(1);
		_pos_sp_triplet.current.z = desiredPosition(2);
		_pos_sp_triplet.current.vx = velocity(0);
		_pos_sp_triplet.current.vy = velocity(1);
		_pos_sp_triplet.current.vz = velocity(2);
		_pos_sp_triplet.current.yaw = yaw + w*t;
		_pos_sp_triplet.current.timestamp = hrt_absolute_time();
//				printf("Next point %f, %f, %f\n", (double) next_point(0),(double) next_point(1),(double) next_point(2));
//				printf("Velocity %f, %f, %f\n", (double) velocity(0),(double) velocity(1),(double) velocity(2));
		publish_setpoint();
	}
}

/* Accepts (x,y,z) in NED frame and yaw in degrees and triggers publishing positoin_setpoint_triplet
 */
void TailsitterPathPlanner::publish_waypoint(float x, float y, float z, float yaw) {
	_waypoint.start_point(0) = _local_pos_sp.x;
	_waypoint.start_point(1) = _local_pos_sp.y;
	_waypoint.start_point(2) = _local_pos_sp.z;
	_waypoint.end_point(0) = x;
	_waypoint.end_point(1) = y;
	_waypoint.end_point(2) = z;
	math::Vector<3> direction = _waypoint.end_point - _waypoint.start_point;
	direction.normalize();
	_waypoint.direction = direction;
	_waypoint.yaw = yaw;
	math::Vector<3> velocity = direction * _params.cruise_speed;
	for (int i=0; i<3; i++){
		if (velocity(i) > _params.cruise_speed_max(i) || velocity(i) < -_params.cruise_speed_max(i)){
			float scale = _params.cruise_speed_max(i) / velocity(i);
			scale = scale > 0 ? scale:-scale;
			velocity = velocity * scale;
		}
	}
	_waypoint.speed = velocity.length();
	_waypoint.velocity = velocity;
	_waypoint.start_time = hrt_absolute_time();
	_setpoint_updated = true;
}

void
TailsitterPathPlanner::update_control_mode(int argc, char* argv[])
{
	reset_control_mode();
	if(!strcmp(argv[0], "acc"))
		_control_mode.ignore_acceleration_force = false;
	if(!strcmp(argv[0], "pos"))
		_control_mode.ignore_position = false;
}


void
TailsitterPathPlanner::reset_control_mode()
{
	_control_mode.ignore_thrust = true;
	_control_mode.ignore_attitude = true;
	_control_mode.ignore_bodyrate = true;
	_control_mode.ignore_position = false;
	_control_mode.ignore_velocity = true;
	_control_mode.ignore_acceleration_force = true;
	_control_mode.ignore_alt_hold = true;
}
void
TailsitterPathPlanner::params_update(bool force)
{
	bool updated;
		struct parameter_update_s param_upd;

		orb_check(_params_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(parameter_update), _params_sub, &param_upd);
		}

		if (updated || force) {
			float v;
			param_get(_param_handles.z_cruise_speed, &v);
			_params.cruise_speed_max(2) = v;
			param_get(_param_handles.xy_cruise_speed, &v);
			_params.cruise_speed_max(0) = v;
			_params.cruise_speed_max(1) = v;
			param_get(_param_handles.cruise_speed, &_params.cruise_speed);

		}
}

void
TailsitterPathPlanner::poll_subscriptions()
{
	bool updated;
	orb_check(_local_pos_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
	}

	orb_check(_v_lpos_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_local_position_setpoint), _v_lpos_sp_sub, &_local_pos_sp);
	}

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
		PX4_INFO("Planner started");

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
