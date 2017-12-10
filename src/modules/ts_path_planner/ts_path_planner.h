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
	bool _setpoint_updated;
	bool _control_mode_updated;

	orb_advert_t _v_control_mode_pub;
	orb_advert_t _position_setpoint_pub;

	struct offboard_control_mode_s		_control_mode;
	struct position_setpoint_triplet_s 	_pos_sp_triplet;

	static void	task_main_trampoline(int argc, char *argv[]);
	void task_main();
	void _publish_setpoint();
	void _publish_control_mode();
	void _reset_control_mode();
};


#endif /* MODULES_TS_PATH_PLANNER_H_ */
