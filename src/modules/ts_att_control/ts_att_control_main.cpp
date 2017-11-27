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
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/fw_virtual_rates_setpoint.h>
#include <uORB/topics/mc_virtual_rates_setpoint.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/mc_att_ctrl_status.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/actuator_outputs.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/circuit_breaker.h>
#include <lib/mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <lib/tailsitter_recovery/tailsitter_recovery.h>
#include "ts_rate_control.h"

/**
 * Tailsitter attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int ts_att_control_main(int argc, char *argv[]);

#define YAW_DEADZONE	0.05f
#define MIN_TAKEOFF_THRUST    1.0f
#define TPA_RATE_LOWER_LIMIT 0.05f
#define MANUAL_THROTTLE_MAX_MULTICOPTER	0.9f
#define ATTITUDE_TC_DEFAULT 0.2f

#define AXIS_INDEX_ROLL 0
#define AXIS_INDEX_PITCH 1
#define AXIS_INDEX_YAW 2
#define AXIS_COUNT 3

class TailsitterAttitudeControl
{
public:
	/**
	 * Constructor
	 */
	TailsitterAttitudeControl();

	/**
	 * Destructor, also kills the main task
	 */
	~TailsitterAttitudeControl();

	/**
	 * Start the tailsitter attitude control task.
	 *
	 * @return		OK on success.
	 */
	int		start();

private:

	bool	_task_should_exit;		/**< if true, task_main() should exit */
	int		_control_task;			/**< task handle */

	int		_ctrl_state_sub;		/**< control state subscription */
	int		_v_att_sp_sub;			/**< vehicle attitude setpoint subscription */
	int		_v_rates_sp_sub;		/**< vehicle rates setpoint subscription */
	int		_v_control_mode_sub;	/**< vehicle control mode subscription */
	int		_params_sub;			/**< parameter updates subscription */
	int		_manual_control_sp_sub;	/**< manual control setpoint subscription */
	int		_armed_sub;				/**< arming status subscription */
	int 	_motor_limits_sub;		/**< motor limits subscription */
	int 	_battery_status_sub;	/**< battery status subscription */

	orb_advert_t	_v_rates_sp_pub;		/**< rate setpoint publication */
	orb_advert_t	_actuators_0_pub;		/**< attitude actuator controls publication */
	orb_advert_t	_controller_status_pub;	/**< controller status publication */
	orb_advert_t	_actuator_outputs_pub;	/**<actuator outputs publication */
	orb_id_t _rates_sp_id;	/**< pointer to correct rates setpoint uORB metadata structure */
	orb_id_t _actuators_id;	/**< pointer to correct actuator controls0 uORB metadata structure */

	bool		_actuators_0_circuit_breaker_enabled;	/**< circuit breaker to suppress output */

	struct control_state_s				_ctrl_state;		/**< control state */
	struct vehicle_attitude_setpoint_s	_v_att_sp;			/**< vehicle attitude setpoint */
	struct vehicle_rates_setpoint_s		_v_rates_sp;		/**< vehicle rates setpoint */
	struct manual_control_setpoint_s	_manual_control_sp;	/**< manual control setpoint */
	struct vehicle_control_mode_s		_v_control_mode;	/**< vehicle control mode */
	struct actuator_controls_s			_actuators;			/**< actuator controls */
	struct actuator_armed_s				_armed;				/**< actuator arming status */
	struct multirotor_motor_limits_s	_motor_limits;		/**< motor limits */
	struct mc_att_ctrl_status_s 		_controller_status; /**< controller status */
	struct battery_status_s				_battery_status;	/**< battery status */
	struct actuator_outputs_s			_actuator_outputs;  /**< actuator outputs */

	TailsitterRateControl*	_ts_rate_control;
	union {
		struct {
			uint16_t motor_pos	: 1; // 0 - true when any motor has saturated in the positive direction
			uint16_t motor_neg	: 1; // 1 - true when any motor has saturated in the negative direction
			uint16_t roll_pos	: 1; // 2 - true when a positive roll demand change will increase saturation
			uint16_t roll_neg	: 1; // 3 - true when a negative roll demand change will increase saturation
			uint16_t pitch_pos	: 1; // 4 - true when a positive pitch demand change will increase saturation
			uint16_t pitch_neg	: 1; // 5 - true when a negative pitch demand change will increase saturation
			uint16_t yaw_pos	: 1; // 6 - true when a positive yaw demand change will increase saturation
			uint16_t yaw_neg	: 1; // 7 - true when a negative yaw demand change will increase saturation
			uint16_t thrust_pos	: 1; // 8 - true when a positive thrust demand change will increase saturation
			uint16_t thrust_neg	: 1; // 9 - true when a negative thrust demand change will increase saturation
		} flags;
		uint16_t value;
	} _saturation_status;

	perf_counter_t	_loop_perf;			/**< loop performance counter */
	perf_counter_t	_controller_latency_perf;

	math::Vector<3>		_rates_prev;	/**< angular rates on previous step */
	math::Vector<3>		_rates_sp_prev; /**< previous rates setpoint */
	math::Vector<3>		_rates_sp;		/**< angular rates setpoint */
	math::Vector<3>		_rates_int;		/**< angular rates integral error */
	float				_thrust_sp;		/**< thrust setpoint */
	math::Vector<3>		_att_control;	/**< attitude control vector */

	math::Matrix<3, 3>  _I;				/**< identity matrix */
	math::Matrix<3, 3>  _J;				/**< Moment of Inertia */

	struct {
		param_t roll_p;
		param_t roll_i;
		param_t roll_d;
		param_t roll_integ_lim;
		param_t roll_tc;
		param_t roll_rate_tc;

		param_t pitch_p;
		param_t pitch_i;
		param_t pitch_d;
		param_t pitch_integ_lim;
		param_t pitch_tc;
		param_t pitch_rate_tc;

		param_t tpa_breakpoint_p;
		param_t tpa_breakpoint_i;
		param_t tpa_breakpoint_d;
		param_t tpa_rate_p;
		param_t tpa_rate_i;
		param_t tpa_rate_d;

		param_t yaw_p;
		param_t yaw_i;
		param_t yaw_d;
		param_t yaw_integ_lim;
		param_t yaw_tc;
		param_t yaw_rate_tc;

		param_t roll_max;
		param_t pitch_max;
		param_t yaw_max;

		param_t roll_rate_max;
		param_t pitch_rate_max;
		param_t yaw_rate_max;

		param_t	j_xx;
		param_t	j_yy;
		param_t	j_zz;

		param_t lift_constant;
		param_t drag_constant;
		param_t thrust_constant;
		param_t momt_constant_r;
		param_t momt_constant_p;
		param_t momt_constant_y;



	}		_params_handles;		/**< handles for interesting parameters */

	struct {
		math::Vector<3> att_p;					/**< P gain for angular error */
		math::Vector<3> att_i;
		math::Vector<3> att_d;
		math::Vector<3> att_int_lim;			/**< integrator state limit for rate loop */
		math::Vector<3> att_tc;
		math::Vector<3> rate_tc;				/**< time constant for angular rate error */
		math::Vector<3> att_max;
		math::Vector<3> rate_max;
		math::Vector<3> momt_const;				/** Momentum constants roll, pith, yaw **/

		float tpa_breakpoint_p;				/**< Throttle PID Attenuation breakpoint */
		float tpa_breakpoint_i;				/**< Throttle PID Attenuation breakpoint */
		float tpa_breakpoint_d;				/**< Throttle PID Attenuation breakpoint */
		float tpa_rate_p;					/**< Throttle PID Attenuation slope */
		float tpa_rate_i;					/**< Throttle PID Attenuation slope */
		float tpa_rate_d;					/**< Throttle PID Attenuation slope */

		float drag_const;
		float lift_const;
		float thrust_const;

	}		_params;


	/**
	 * Update our local parameter cache.
	 */
	int			parameters_update();

	/**
	 * Check for parameter update and handle it.
	 */
	void		parameter_update_poll();

	/**
	 * Check for changes in vehicle control mode.
	 */
	void		vehicle_control_mode_poll();

	/**
	 * Check for changes in manual inputs.
	 */
	void		vehicle_manual_poll();

	/**
	 * Check for attitude setpoint updates.
	 */
	void		vehicle_attitude_setpoint_poll();

	/**
	 * Check for rates setpoint updates.
	 */
	void		vehicle_rates_setpoint_poll();

	/**
	 * Check for arming status updates.
	 */
	void		arming_status_poll();

	/**
	 * Attitude controller.
	 */
	void		control_attitude(float dt);

	/**
	 * Attitude rates controller.
	 */
	void		control_attitude_rates(float dt);

	/**
	 * Throttle PID attenuation.
	 */
	math::Vector<3> pid_attenuations(float tpa_breakpoint, float tpa_rate);

	/**
	 * Check for vehicle motor limits status.
	 */
	void		vehicle_motor_limits_poll();

	/**
	 * Check for battery status updates.
	 */
	void		battery_status_poll();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main attitude control task.
	 */
	void		task_main();
};

namespace ts_att_control
{

TailsitterAttitudeControl	*g_control;
}

TailsitterAttitudeControl::TailsitterAttitudeControl() :

	_task_should_exit(false),
	_control_task(-1),

	/* subscriptions */
	_ctrl_state_sub(-1),
	_v_att_sp_sub(-1),
	_v_control_mode_sub(-1),
	_params_sub(-1),
	_manual_control_sp_sub(-1),
	_armed_sub(-1),

	/* publications */
	_v_rates_sp_pub(nullptr),
	_actuators_0_pub(nullptr),
	_controller_status_pub(nullptr),
	_actuator_outputs_pub(nullptr),
	_rates_sp_id(0),
	_actuators_id(0),

	_actuators_0_circuit_breaker_enabled(false),

	_ctrl_state{},
	_v_att_sp{},
	_v_rates_sp{},
	_manual_control_sp{},
	_v_control_mode{},
	_actuators{},
	_armed{},
	_motor_limits{},
	_controller_status{},
	_battery_status{},
	_ts_rate_control(nullptr),
	_saturation_status{},
	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "ts_att_control")),
	_controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency"))

{
	_ts_rate_control = new TailsitterRateControl;
	_params.att_p.zero();
	_params.att_i.zero();
	_params.att_d.zero();
	_params.att_int_lim.zero();
	_params.att_tc.zero();
	_params.rate_tc.zero();
	_params.att_max.zero();
	_params.rate_max.zero();
	_params.momt_const.zero();
	_rates_prev.zero();
	_rates_sp.zero();
	_rates_sp_prev.zero();
	_rates_int.zero();
	_thrust_sp = 0.0f;
	_att_control.zero();

	_I.identity();
	_J.identity();

	_params_handles.roll_p			= 	param_find("TS_ROLL_P");
	_params_handles.roll_i			=	param_find("TS_ROLL_I");
	_params_handles.roll_d			=	param_find("TS_ROLL_D");
	_params_handles.roll_integ_lim	=	param_find("TS_ROLL_INT_LIM");
	_params_handles.roll_tc			= 	param_find("TS_ROLL_TC");
	_params_handles.roll_rate_tc	=	param_find("TS_ROLL_RATE_TC");
	_params_handles.roll_max		=	param_find("TS_ROLL_MAX");
	_params_handles.roll_rate_max	= 	param_find("TS_ROLL_RATE_MAX");

	_params_handles.pitch_p			= 	param_find("TS_PITCH_P");
	_params_handles.pitch_i			=	param_find("TS_PITCH_I");
	_params_handles.pitch_d			=	param_find("TS_PITCH_D");
	_params_handles.pitch_integ_lim	=	param_find("TS_PITCH_INT_LIM");
	_params_handles.pitch_tc		= 	param_find("TS_PITCH_TC");
	_params_handles.pitch_rate_tc	=	param_find("TS_PITCH_RATE_TC");
	_params_handles.pitch_max		=	param_find("TS_PITCH_MAX");
	_params_handles.pitch_rate_max	= 	param_find("TS_PITCH_RATE_MX");

	_params_handles.yaw_p			= 	param_find("TS_YAW_P");
	_params_handles.yaw_i			=	param_find("TS_YAW_I");
	_params_handles.yaw_d			=	param_find("TS_YAW_D");
	_params_handles.yaw_integ_lim	=	param_find("TS_YAW_INT_LIM");
	_params_handles.yaw_tc			=	param_find("TS_YAW_TC");
	_params_handles.yaw_rate_tc		=	param_find("TS_YAW_RATE_TC");
	_params_handles.yaw_max			=	param_find("TS_YAW_MAX");
	_params_handles.yaw_rate_max	= 	param_find("TS_YAW_RATE_MAX");

	_params_handles.tpa_breakpoint_p 	= 	param_find("TS_TPA_BREAK_P");
	_params_handles.tpa_breakpoint_i 	= 	param_find("TS_TPA_BREAK_I");
	_params_handles.tpa_breakpoint_d 	= 	param_find("TS_TPA_BREAK_D");
	_params_handles.tpa_rate_p	 	= 	param_find("TS_TPA_RATE_P");
	_params_handles.tpa_rate_i	 	= 	param_find("TS_TPA_RATE_I");
	_params_handles.tpa_rate_d	 	= 	param_find("TS_TPA_RATE_D");

	_params_handles.j_xx			=	param_find("TS_J_XX");
	_params_handles.j_yy			=	param_find("TS_J_YY");
	_params_handles.j_zz			=	param_find("TS_J_ZZ");

	_params_handles.lift_constant 	=	param_find("TS_LIFT_CONST");
	_params_handles.drag_constant	=	param_find("TS_DRAG_CONST");
	_params_handles.thrust_constant =	param_find("TS_THRUST_CONST");
	_params_handles.momt_constant_r =	param_find("TS_MOMT_CONST_R");
	_params_handles.momt_constant_p =	param_find("TS_MOMT_CONST_P");
	_params_handles.momt_constant_y =	param_find("TS_MOMT_CONST_Y");

	/* fetch initial parameter values */
	parameters_update();


}

TailsitterAttitudeControl::~TailsitterAttitudeControl()
{
	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	ts_att_control::g_control = nullptr;
}

int
TailsitterAttitudeControl::parameters_update()
{
	float v;

	float roll_tc, pitch_tc, yaw_tc;

	param_get(_params_handles.roll_tc, &roll_tc);
	param_get(_params_handles.pitch_tc, &pitch_tc);
	param_get(_params_handles.pitch_tc, &yaw_tc);
	_params.att_tc(0) = roll_tc;
	_params.att_tc(1) = pitch_tc;
	_params.att_tc(2) = yaw_tc;

	/* roll gains */
	param_get(_params_handles.roll_p, &v);
	_params.att_p(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc);
	param_get(_params_handles.roll_i, &v);
	_params.att_i(0) = v;
	param_get(_params_handles.roll_d, &v);
	_params.att_d(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc);
	param_get(_params_handles.roll_integ_lim, &v);
	_params.att_int_lim(0) = v;
	param_get(_params_handles.roll_rate_tc, &v);
	_params.rate_tc(0) = v;
	param_get(_params_handles.roll_max, &v);
	_params.att_max(0) = v;
	param_get(_params_handles.roll_rate_max, &v);
	_params.rate_max(0) = v;

	/* pitch gains */
	param_get(_params_handles.pitch_p, &v);
	_params.att_p(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
	param_get(_params_handles.pitch_i, &v);
	_params.att_i(1) = v;
	param_get(_params_handles.pitch_d, &v);
	_params.att_d(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
	param_get(_params_handles.pitch_integ_lim, &v);
	_params.att_int_lim(1) = v;
	param_get(_params_handles.pitch_rate_tc, &v);
	_params.rate_tc(1) = v;
	param_get(_params_handles.pitch_max, &v);
	_params.att_max(1) = v;
	param_get(_params_handles.pitch_rate_max, &v);
	_params.rate_max(1) = v;

	/* yaw gains */
	param_get(_params_handles.yaw_p, &v);
	_params.att_p(2) = v * (ATTITUDE_TC_DEFAULT / yaw_tc);
	param_get(_params_handles.yaw_i, &v);
	_params.att_i(2) = v;
	param_get(_params_handles.yaw_d, &v);
	_params.att_d(2) = v * (ATTITUDE_TC_DEFAULT / yaw_tc);
	param_get(_params_handles.yaw_integ_lim, &v);
	_params.att_int_lim(2) = v;
	param_get(_params_handles.yaw_rate_tc, &v);
	_params.rate_tc(2) = v;
	param_get(_params_handles.yaw_max, &v);
	_params.att_max(2) = v;
	param_get(_params_handles.yaw_rate_max, &v);
	_params.rate_max(2) = v;

	param_get(_params_handles.tpa_breakpoint_p, &_params.tpa_breakpoint_p);
	param_get(_params_handles.tpa_breakpoint_i, &_params.tpa_breakpoint_i);
	param_get(_params_handles.tpa_breakpoint_d, &_params.tpa_breakpoint_d);
	param_get(_params_handles.tpa_rate_p, &_params.tpa_rate_p);
	param_get(_params_handles.tpa_rate_i, &_params.tpa_rate_i);
	param_get(_params_handles.tpa_rate_d, &_params.tpa_rate_d);

	_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled("CBRK_RATE_CTRL", CBRK_RATE_CTRL_KEY);

	param_get(_params_handles.j_xx, &_J(0,0));
	param_get(_params_handles.j_yy, &_J(1,1));
	param_get(_params_handles.j_zz, &_J(2,2));

	param_get(_params_handles.drag_constant, &v);
	_params.drag_const = v;
	param_get(_params_handles.lift_constant, &v);
	_params.lift_const = v;
	param_get(_params_handles.thrust_constant, &v);
	_params.thrust_const = v;

	param_get(_params_handles.momt_constant_r, &v);
	_params.momt_const(0) = v;
	param_get(_params_handles.momt_constant_p, &v);
	_params.momt_const(1) = v;
	param_get(_params_handles.momt_constant_y, &v);
	_params.momt_const(2) = v;

	return OK;
}

void
TailsitterAttitudeControl::parameter_update_poll()
{
	bool updated;

	/* Check if parameters have changed */
	orb_check(_params_sub, &updated);

	if (updated) {
		struct parameter_update_s param_update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
		parameters_update();
	}
}

void
TailsitterAttitudeControl::vehicle_control_mode_poll()
{
	bool updated;

	/* Check if vehicle control mode has changed */
	orb_check(_v_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _v_control_mode_sub, &_v_control_mode);
	}
}

void
TailsitterAttitudeControl::vehicle_manual_poll()
{
	bool updated;

	/* get pilots inputs */
	orb_check(_manual_control_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, &_manual_control_sp);
	}
}

void
TailsitterAttitudeControl::vehicle_attitude_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_att_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_sub, &_v_att_sp);
	}
}

void
TailsitterAttitudeControl::vehicle_rates_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_rates_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &_v_rates_sp);
	}
}

void
TailsitterAttitudeControl::arming_status_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_armed_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
	}
}


void
TailsitterAttitudeControl::vehicle_motor_limits_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_motor_limits_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(multirotor_motor_limits), _motor_limits_sub, &_motor_limits);
		_saturation_status.value = _motor_limits.saturation_status;
	}
}

void
TailsitterAttitudeControl::battery_status_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_battery_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(battery_status), _battery_status_sub, &_battery_status);
	}
}

/**
 * Attitude controller.
 * Input: 'vehicle_attitude_setpoint' topics (depending on mode)
 * Output: '_rates_sp' vector, '_thrust_sp'
 */
void
TailsitterAttitudeControl::control_attitude(float dt)
{
	vehicle_attitude_setpoint_poll();

	_thrust_sp = _v_att_sp.thrust;

	/* construct attitude setpoint rotation matrix */
	math::Quaternion q_sp(_v_att_sp.q_d[0], _v_att_sp.q_d[1], _v_att_sp.q_d[2], _v_att_sp.q_d[3]);
	math::Matrix<3, 3> R_sp = q_sp.to_dcm();

	/* get current rotation matrix from control state quaternions */
	math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
	math::Matrix<3, 3> R = q_att.to_dcm();

	/* all input data is ready, run controller itself */

	/* try to move thrust vector shortest way, because yaw response is slower than roll/pitch */
	math::Vector<3> R_z(R(0, 2), R(1, 2), R(2, 2));
	math::Vector<3> R_sp_z(R_sp(0, 2), R_sp(1, 2), R_sp(2, 2));

	/* axis and sin(angle) of desired rotation */
	math::Vector<3> e_R = R.transposed() * (R_z % R_sp_z);

	/* calculate angle error */
	float e_R_z_sin = e_R.length();
	float e_R_z_cos = R_z * R_sp_z;

	/* calculate weight for yaw control */
	float yaw_w = R_sp(2, 2) * R_sp(2, 2);

	/* calculate rotation matrix after roll/pitch only rotation */
	math::Matrix<3, 3> R_rp;

	if (e_R_z_sin > 0.0f) {
		/* get axis-angle representation */
		float e_R_z_angle = atan2f(e_R_z_sin, e_R_z_cos);
		math::Vector<3> e_R_z_axis = e_R / e_R_z_sin;

		e_R = e_R_z_axis * e_R_z_angle;

		/* cross product matrix for e_R_axis */
		math::Matrix<3, 3> e_R_cp;
		e_R_cp.zero();
		e_R_cp(0, 1) = -e_R_z_axis(2);
		e_R_cp(0, 2) = e_R_z_axis(1);
		e_R_cp(1, 0) = e_R_z_axis(2);
		e_R_cp(1, 2) = -e_R_z_axis(0);
		e_R_cp(2, 0) = -e_R_z_axis(1);
		e_R_cp(2, 1) = e_R_z_axis(0);

		/* rotation matrix for roll/pitch only rotation */
		R_rp = R * (_I + e_R_cp * e_R_z_sin + e_R_cp * e_R_cp * (1.0f - e_R_z_cos));

	} else {
		/* zero roll/pitch rotation */
		R_rp = R;
	}

	/* R_rp and R_sp has the same Z axis, calculate yaw error */
	math::Vector<3> R_sp_x(R_sp(0, 0), R_sp(1, 0), R_sp(2, 0));
	math::Vector<3> R_rp_x(R_rp(0, 0), R_rp(1, 0), R_rp(2, 0));
	e_R(2) = atan2f((R_rp_x % R_sp_x) * R_sp_z, R_rp_x * R_sp_x) * yaw_w;

	if (e_R_z_cos < 0.0f) {
		/* for large thrust vector rotations use another rotation method:
		 * calculate angle and axis for R -> R_sp rotation directly */
		math::Quaternion q_error;
		q_error.from_dcm(R.transposed() * R_sp);
		math::Vector<3> e_R_d = q_error(0) >= 0.0f ? q_error.imag()  * 2.0f : -q_error.imag() * 2.0f;

		/* use fusion of Z axis based rotation and direct rotation */
		float direct_w = e_R_z_cos * e_R_z_cos * yaw_w;
		e_R = e_R * (1.0f - direct_w) + e_R_d * direct_w;
	}

	/* calculate angular rates setpoint */
	_rates_sp = _params.att_p.emult(e_R);

	/* limit rates */
	for (int i = 0 ; i < 3; i++){
		_rates_sp(i) = math::constrain(_rates_sp(i), -_params.rate_max(i), _params.rate_max(i));
	}

	/* feed forward yaw setpoint rate */
	//_rates_sp(2) += _v_att_sp.yaw_sp_move_rate * yaw_w * _params.yaw_ff;
}

/*
 * Throttle PID attenuation
 * Function visualization available here https://www.desmos.com/calculator/gn4mfoddje
 * Input: 'tpa_breakpoint', 'tpa_rate', '_thrust_sp'
 * Output: 'pidAttenuationPerAxis' vector
 */
math::Vector<3>
TailsitterAttitudeControl::pid_attenuations(float tpa_breakpoint, float tpa_rate)
{
	/* throttle pid attenuation factor */
	float tpa = 1.0f - tpa_rate * (fabsf(_v_rates_sp.thrust) - tpa_breakpoint) / (1.0f - tpa_breakpoint);
	tpa = fmaxf(TPA_RATE_LOWER_LIMIT, fminf(1.0f, tpa));

	math::Vector<3> pidAttenuationPerAxis;
	pidAttenuationPerAxis(AXIS_INDEX_ROLL) = tpa;
	pidAttenuationPerAxis(AXIS_INDEX_PITCH) = tpa;
	pidAttenuationPerAxis(AXIS_INDEX_YAW) = 1.0;

	return pidAttenuationPerAxis;
}

/*
 * Attitude rates controller.
 * Input: '_rates_sp' vector, '_thrust_sp'
 * Output: '_att_control' vector
 */
void
TailsitterAttitudeControl::control_attitude_rates(float dt)
{

	/* current body angular rates */
	math::Vector<3> rates;
	rates(0) = _ctrl_state.roll_rate;
	rates(1) = _ctrl_state.pitch_rate;
	rates(2) = _ctrl_state.yaw_rate;

	/* angular rates error */
	math::Vector<3> rates_err = _rates_sp - rates;

	math::Vector<3> att_control1 = (_J * rates_err).edivide(_params.rate_tc);
	math::Vector<3> att_control2 = rates % (_J * rates);

	_att_control = att_control1 + att_control2;

	for (int i=0; i<3; i++){
		float diff = _att_control(i) - _params.rate_max(i);
		if(diff > 0){
			warnx("Rate %d hit upper bound", i, (double) _params.rate_max(i));
			_att_control(i) = _params.rate_max(i);
		}

		diff = _att_control(i) + _params.rate_max(i);

		if(diff < 0){
			warnx("Rate %d hits lower bound %f", i, (double) -_params.rate_max(i));
			_att_control(i) = -_params.rate_max(i);
		}
	}
//	/* update integral only if motors are providing enough thrust to be effective */
//	if (_thrust_sp > MIN_TAKEOFF_THRUST) {
//		for (int i = AXIS_INDEX_ROLL; i < AXIS_COUNT; i++) {
//			// Check for positive control saturation
//			bool positive_saturation =
//				((i == AXIS_INDEX_ROLL) && _saturation_status.flags.roll_pos) ||
//				((i == AXIS_INDEX_PITCH) && _saturation_status.flags.pitch_pos) ||
//				((i == AXIS_INDEX_YAW) && _saturation_status.flags.yaw_pos);
//
//			// Check for negative control saturation
//			bool negative_saturation =
//				((i == AXIS_INDEX_ROLL) && _saturation_status.flags.roll_neg) ||
//				((i == AXIS_INDEX_PITCH) && _saturation_status.flags.pitch_neg) ||
//				((i == AXIS_INDEX_YAW) && _saturation_status.flags.yaw_neg);
//
//			// prevent further positive control saturation
//			if (positive_saturation) {
//				rates_err(i) = math::min(rates_err(i), 0.0f);
//
//			}
//
//			// prevent further negative control saturation
//			if (negative_saturation) {
//				rates_err(i) = math::max(rates_err(i), 0.0f);
//
//			}
//
//			// Perform the integration using a first order method and do not propaate the result if out of range or invalid
//			float rate_i = _rates_int(i) + _params.rate_i(i) * rates_err(i) * dt;
//
//			if (PX4_ISFINITE(rate_i) && rate_i > -_params.rate_int_lim(i) && rate_i < _params.rate_int_lim(i)) {
//				_rates_int(i) = rate_i;
//
//			}
//		}
//	}

//	/* explicitly limit the integrator state */
//	for (int i = AXIS_INDEX_ROLL; i < AXIS_COUNT; i++) {
//		_rates_int(i) = math::constrain(_rates_int(i), -_params.rate_int_lim(i), _params.rate_int_lim(i));
//
//	}
}

void
TailsitterAttitudeControl::task_main_trampoline(int argc, char *argv[])
{
	ts_att_control::g_control->task_main();
}

void
TailsitterAttitudeControl::task_main()
{

	/*
	 * do subscriptions
	 */
	_v_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_v_rates_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
	_ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
	_v_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	_motor_limits_sub = orb_subscribe(ORB_ID(multirotor_motor_limits));
	_battery_status_sub = orb_subscribe(ORB_ID(battery_status));

	/* Topic id */
	_rates_sp_id = ORB_ID(vehicle_rates_setpoint);
	_actuators_id = ORB_ID(actuator_controls_0);
	/* initialize parameters cache */
	parameters_update();

	/* wakeup source: vehicle attitude */
	px4_pollfd_struct_t fds[1];

	fds[0].fd = _ctrl_state_sub;
	fds[0].events = POLLIN;

	while (!_task_should_exit) {

		/* wait for up to 100ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("mc att ctrl: poll error %d, %d", pret, errno);
			/* sleep a bit before next try */
			usleep(100000);
			continue;
		}

		perf_begin(_loop_perf);

		/* run controller on attitude changes */
		if (fds[0].revents & POLLIN) {
			static uint64_t last_run = 0;
			float dt = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too small (< 2ms) and too large (> 20ms) dt's */
			if (dt < 0.002f) {
				dt = 0.002f;

			} else if (dt > 0.02f) {
				dt = 0.02f;
			}

			/* copy attitude and control state topics */
			orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);

			/* check for updates in other topics */
			parameter_update_poll();
			vehicle_control_mode_poll();
			arming_status_poll();
			vehicle_manual_poll();
			vehicle_motor_limits_poll();
			battery_status_poll();

			if (_v_control_mode.flag_control_attitude_enabled) {


				control_attitude(dt);

				/* publish attitude rates setpoint */
				_v_rates_sp.roll = _rates_sp(0);
				_v_rates_sp.pitch = _rates_sp(1);
				_v_rates_sp.yaw = _rates_sp(2);
				_v_rates_sp.thrust = _thrust_sp;
				_v_rates_sp.timestamp = hrt_absolute_time();

				if (_v_rates_sp_pub != nullptr) {
					orb_publish(_rates_sp_id, _v_rates_sp_pub, &_v_rates_sp);

				} else if (_rates_sp_id) {
					_v_rates_sp_pub = orb_advertise(_rates_sp_id, &_v_rates_sp);
				}

				//}

			} else {
				/* attitude controller disabled, poll rates setpoint topic */
				if (_v_control_mode.flag_control_manual_enabled) {
					/* manual rates control - ACRO mode */
					_rates_sp = math::Vector<3>(_manual_control_sp.y, -_manual_control_sp.x,
								    _manual_control_sp.r);//.emult(_params.acro_rate_max);
					_thrust_sp = math::min(_manual_control_sp.z, MANUAL_THROTTLE_MAX_MULTICOPTER);

					/* publish attitude rates setpoint */
					_v_rates_sp.roll = _rates_sp(0);
					_v_rates_sp.pitch = _rates_sp(1);
					_v_rates_sp.yaw = _rates_sp(2);
					_v_rates_sp.thrust = _thrust_sp;
					_v_rates_sp.timestamp = hrt_absolute_time();

					if (_v_rates_sp_pub != nullptr) {
						orb_publish(_rates_sp_id, _v_rates_sp_pub, &_v_rates_sp);

					} else if (_rates_sp_id) {
						_v_rates_sp_pub = orb_advertise(_rates_sp_id, &_v_rates_sp);
					}

				} else {
					/* attitude controller disabled, poll rates setpoint topic */
					vehicle_rates_setpoint_poll();
					_rates_sp(0) = _v_rates_sp.roll;
					_rates_sp(1) = _v_rates_sp.pitch;
					_rates_sp(2) = _v_rates_sp.yaw;
					_thrust_sp = _v_rates_sp.thrust;
				}
			}

			if (_v_control_mode.flag_control_rates_enabled) {
				control_attitude_rates(dt);

				/* publish actuator controls */
				_actuators.control[0] = (PX4_ISFINITE(_att_control(0))) ? _att_control(0) : 0.0f;
				_actuators.control[1] = (PX4_ISFINITE(_att_control(1))) ? _att_control(1) : 0.0f;
				_actuators.control[2] = (PX4_ISFINITE(_att_control(2))) ? _att_control(2) : 0.0f;
				_actuators.control[3] = (PX4_ISFINITE(_thrust_sp)) ? _thrust_sp : 0.0f;
				_actuators.control[7] = _v_att_sp.landing_gear;
				_actuators.timestamp = hrt_absolute_time();
				_actuators.timestamp_sample = _ctrl_state.timestamp;


				_controller_status.roll_rate_integ = _rates_int(0);
				_controller_status.pitch_rate_integ = _rates_int(1);
				_controller_status.yaw_rate_integ = _rates_int(2);
				_controller_status.timestamp = hrt_absolute_time();

				if (!_actuators_0_circuit_breaker_enabled) {
					if (_actuators_0_pub != nullptr) {

						orb_publish(_actuators_id, _actuators_0_pub, &_actuators);
						perf_end(_controller_latency_perf);

					} else if (_actuators_id) {
						_actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
					}

				}

				/* publish controller status */
				if (_controller_status_pub != nullptr) {
					orb_publish(ORB_ID(mc_att_ctrl_status), _controller_status_pub, &_controller_status);

				} else {
					_controller_status_pub = orb_advertise(ORB_ID(mc_att_ctrl_status), &_controller_status);
				}
			}

			float outputs[4];
			memset(outputs, 0, sizeof(outputs));
			/* run mixer for tailsitter */
			if(_armed.armed){
				math::Vector<3> momentum_ref;
				momentum_ref.zero();
				for(int i=0; i<3; i++){
					momentum_ref(i) = _actuators.control[i];
				}
				_ts_rate_control->mix(_actuators.control[3], momentum_ref, outputs);
			}
			_actuator_outputs.noutputs = 6;
			_actuator_outputs.timestamp = hrt_absolute_time();
			_actuator_outputs.output[0] = (PX4_ISFINITE(outputs[0])) ? outputs[0] : 0.0f;
			_actuator_outputs.output[1] = (PX4_ISFINITE(outputs[1])) ? outputs[1] : 0.0f;
			_actuator_outputs.output[4] = (PX4_ISFINITE(outputs[2])) ? outputs[2] : 0.0f;
			_actuator_outputs.output[5] = (PX4_ISFINITE(outputs[3])) ? outputs[3] : 0.0f;

			if (_controller_status_pub != nullptr) {
				orb_publish(ORB_ID(ts_actuator_outputs_virtual), _actuator_outputs_pub, &_actuator_outputs);

			} else {
				_actuator_outputs_pub = orb_advertise(ORB_ID(ts_actuator_outputs_virtual), &_actuator_outputs);
			}


			if (_v_control_mode.flag_control_termination_enabled) {
				if (false) {

					_rates_sp.zero();
					_rates_int.zero();
					_thrust_sp = 0.0f;
					_att_control.zero();


					/* publish actuator controls */
					_actuators.control[0] = 0.0f;
					_actuators.control[1] = 0.0f;
					_actuators.control[2] = 0.0f;
					_actuators.control[3] = 0.0f;
					_actuators.timestamp = hrt_absolute_time();
					_actuators.timestamp_sample = _ctrl_state.timestamp;

					if (!_actuators_0_circuit_breaker_enabled) {
						if (_actuators_0_pub != nullptr) {

							orb_publish(_actuators_id, _actuators_0_pub, &_actuators);
							perf_end(_controller_latency_perf);

						} else if (_actuators_id) {
							_actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
						}
					}

					_controller_status.roll_rate_integ = _rates_int(0);
					_controller_status.pitch_rate_integ = _rates_int(1);
					_controller_status.yaw_rate_integ = _rates_int(2);
					_controller_status.timestamp = hrt_absolute_time();

					/* publish controller status */
					if (_controller_status_pub != nullptr) {
						orb_publish(ORB_ID(mc_att_ctrl_status), _controller_status_pub, &_controller_status);

					} else {
						_controller_status_pub = orb_advertise(ORB_ID(mc_att_ctrl_status), &_controller_status);
					}

					/* publish attitude rates setpoint */
					_v_rates_sp.roll = _rates_sp(0);
					_v_rates_sp.pitch = _rates_sp(1);
					_v_rates_sp.yaw = _rates_sp(2);
					_v_rates_sp.thrust = _thrust_sp;
					_v_rates_sp.timestamp = hrt_absolute_time();

					if (_v_rates_sp_pub != nullptr) {
						orb_publish(_rates_sp_id, _v_rates_sp_pub, &_v_rates_sp);

					} else if (_rates_sp_id) {
						_v_rates_sp_pub = orb_advertise(_rates_sp_id, &_v_rates_sp);
					}
				}
			}



		}

		perf_end(_loop_perf);
	}

	_control_task = -1;
	return;
}

int
TailsitterAttitudeControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("ts_att_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1500,
					   (px4_main_t)&TailsitterAttitudeControl::task_main_trampoline,
					   nullptr);
	warnx("Started ts att control\n");
	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int ts_att_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: ts_att_control {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (ts_att_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}
		warnx("Creating Pointer\n");
		ts_att_control::g_control = new TailsitterAttitudeControl;

		if (ts_att_control::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}
		warnx("Starting ts att control\n");

		if (OK != ts_att_control::g_control->start()) {
			delete ts_att_control::g_control;
			ts_att_control::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (ts_att_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete ts_att_control::g_control;
		ts_att_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (ts_att_control::g_control) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}
