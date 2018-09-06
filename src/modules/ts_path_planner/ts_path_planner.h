/*
 * ts_rate_control.h
 *
 *  Created on: Nov 26, 2017
 *      Author: tracy
 */

#ifndef MODULES_TS_PATH_PLANNER_H_
#define MODULES_TS_PATH_PLANNER_H_


#include <math.h>
#include <lib/mathlib/mathlib.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/position_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <platforms/px4_workqueue.h>
#include <lib/LoopTimer/LoopTimer.h>


class __EXPORT TailsitterPathPlanner{
public:
	TailsitterPathPlanner();
	~TailsitterPathPlanner();

	int start();
	void update_pos_setpoint(int argc, char *argv[]);
	void update_control_mode(int argc, char* argv[]);

private:
	bool _task_should_exit;
	int  _planner_task;
	int  _circle_traj_generator;
	bool _setpoint_updated;
	bool _control_mode_updated;

	orb_advert_t _v_control_mode_pub;
	orb_advert_t _position_setpoint_pub;

	int	_params_sub;
	int _local_pos_sub;
	int _v_lpos_sp_sub;

	struct offboard_control_mode_s		_control_mode;
	struct position_setpoint_triplet_s 	_pos_sp_triplet;
	struct vehicle_local_position_s		_local_pos;
	struct vehicle_local_position_setpoint_s _local_pos_sp;
	struct work_s						_work;

	struct{
		math::Vector<3>	cruise_speed_max;
		float			cruise_speed;
	}_params;

	struct{
		param_t	z_cruise_speed;
		param_t xy_cruise_speed;
		param_t cruise_speed;
	}_param_handles;

	struct{
		uint64_t start_time;
		math::Vector<3> end_point;
		math::Vector<3> start_point;
		math::Vector<3> direction;
		math::Vector<3> velocity;
		float speed;
		float	yaw;
	}_waypoint;

	LoopTimer _looptimer;
	LoopTimer _looptimer_circle;

	static void	task_main_trampoline(int argc, char *argv[]);
	void task_main();
	static void star_generator_trampoline(int argc, char*argv[]);
	void star_generator_main();
	void publish_setpoint();
	void publish_control_mode();
	void publish_waypoint(float x, float y, float z, float yaw);
	void circle_trajectory(float centerX, float centerY, float radius,
			float zAmplitude, float yaw, float revs);
	static void publish_control_mode_trampoline(void *arg);
	void reset_control_mode();
	void params_update(bool force_update);
	void poll_subscriptions();
};


#endif /* MODULES_TS_PATH_PLANNER_H_ */
