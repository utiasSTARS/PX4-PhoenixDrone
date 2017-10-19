/*
 * mixer_tailsitter.cpp
 *
 *  Created on: Jun 26, 2017
 *      Author: yilun
 */

#include <px4_config.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <math.h>
#include <lib/mathlib/mathlib.h>

#include <px4iofirmware/protocol.h>
#include <drivers/drv_pwm_output.h>

#include "mixer.h"

#define debug(fmt, args...)	do { } while(0)
//#define debug(fmt, args...)	do { printf("[mixer] " fmt "\n", ##args); } while(0)
//#include <debug.h>
//#define debug(fmt, args...)	syslog(fmt "\n", ##args)

#define NAN_VALUE	(0.0f/0.0f)

namespace
{

float constrain(float val, float min, float max)
{
	return (val < min) ? min : ((val > max) ? max : val);
}

float normalize(float val, float val_min, float val_max, float nor_min, float nor_max)
{
	if (!(val >= val_min && val <= val_max && nor_min < nor_max)) return NAN_VALUE;
	return nor_max * (val - val_min)/(val_max - val_min) + nor_min * (val_max - val)/(val_max - val_min);
}

} // anonymous namespace

TailsitterMixer::TailsitterMixer(ControlCallback control_cb,
		uintptr_t cb_handle,
		mixer_ts_s *mixer_info):
				Mixer(control_cb, cb_handle),
				_mixer_info(*mixer_info),
				_delta_out_max(0),
				_rotor_controller_init(false),
				_last_control_timestamp(0),
				_pi_integrals(0,0),
				_curr_omegas(0,0),
				_curr_omegas_valid(false)

{
}

TailsitterMixer::~TailsitterMixer()
{
}

TailsitterMixer *
TailsitterMixer::from_text(Mixer::ControlCallback control_cb,
		uintptr_t cb_handle, const char *buf, unsigned &buflen)
{
	mixer_ts_s mixer_info;
	int s[6];
	int used;
	/* enforce that the mixer ends with a new line */
	if (!string_well_formed(buf, buflen)) {
		return nullptr;
	}

	if (sscanf(buf, "T: %d %d %d %d %d %d%n", &s[0], &s[1], &s[2], &s[3], &s[4], &s[5], &used) != 6) {
		debug("tailsitter parse failed on '%s'", buf);
		return nullptr;
	}

	mixer_info.rads_max = s[0]/1.f;
	mixer_info.deg_min = s[1]/100.f;
	mixer_info.deg_max = s[2]/100.f;
	mixer_info.k_w2 = s[3]/1.f;
	mixer_info.k_w = s[4]/1.f;
	mixer_info.k_c = s[5]/1.f;

	//TODO: Write parser to setup parameters
	TailsitterMixer *tm = new TailsitterMixer(
			control_cb,
			cb_handle,
			&mixer_info);

	if (tm != nullptr){
		debug("loaded ts mixer.");
	}
	else {
		debug("could not allocate memory for ts mixer.");
	}
	return tm;
}

void
TailsitterMixer::set_max_delta_out_once(float delta_out_max)
{
	_delta_out_max = delta_out_max;
}

void
TailsitterMixer::init_rotor_controller(){

	_pi_integrals = {0,0};
	_rotor_controller_init = true;
}

unsigned
TailsitterMixer::mix(float *outputs, unsigned space, uint16_t *status_reg)
{
	float rads_left  = constrain(get_control(0, 0), 0, _mixer_info.rads_max);
	float rads_right = constrain(get_control(0, 1), 0, _mixer_info.rads_max);

	float elv_left  = constrain(get_control(0, 2), _mixer_info.deg_min, _mixer_info.deg_max);
	float elv_right = constrain(get_control(0, 3), _mixer_info.deg_min, _mixer_info.deg_max);

	// replace static mapping with a pi controller
	/*
	float mot_left  = _mixer_info.k_w2 * rads_left * rads_left
				+ _mixer_info.k_w * rads_left + _mixer_info.k_c;

	float mot_right = _mixer_info.k_w2 * rads_right * rads_right
				+ _mixer_info.k_w * rads_right + _mixer_info.k_c;
	*/
	float mot_left = 0;
	float mot_right = 0;
	float dt;
	if(!_rotor_controller_init){
		printf("Rotor controller not initialized!");
	}
	else{
		if (_last_control_timestamp == 0){
			_last_control_timestamp = hrt_absolute_time();
		}
		else if (_curr_omegas_valid){
			dt = hrt_elapsed_time(&_last_control_timestamp);

			if (dt < 0.05f * _mixer_info.control_interval)
					dt = 0.05f * _mixer_info.control_interval;
			else if (dt > 10.0f * _mixer_info.control_interval)
				dt = 10.0f * _mixer_info.control_interval;

			dt /= 1e6f;
			float omega_desired[2] = {rads_left, rads_right};
			float* pwm_outputs[2] = {&mot_left, &mot_right};
			_rotor_control(dt, omega_desired, pwm_outputs);

			_last_control_timestamp = hrt_absolute_time();
		}
	}


	outputs[0] = constrain(mot_left, -1.f, 1.f);
	outputs[1] = constrain(mot_right, -1.f, 1.f);

	outputs[2] = 0;//Not being used, will be set to NaN in tsfmu cycle
	outputs[3] = 0;

	//Elevon deflections are linear, just normalize between -1, 1
	outputs[4] = normalize(elv_left , _mixer_info.deg_min, _mixer_info.deg_max, -1.f, 1.f);
	outputs[5] = normalize(elv_right, _mixer_info.deg_min, _mixer_info.deg_max, -1.f, 1.f);


	return 6;
}

void
TailsitterMixer::groups_required(uint32_t &groups)
{
	/* Subscribe group 0 and 1 from ts_actuator_controls*/
	groups |= (1 << 0);
	groups |= (1 << 1);
}

void
TailsitterMixer::set_curr_omega(float* omegas)
{
	_curr_omegas.set(omegas);
}

void
TailsitterMixer::set_curr_omega_valid(bool valid){
	_curr_omegas_valid = valid;
}

void
TailsitterMixer::_rotor_control(float dt, float* omega_desired, float** pwm_outputs)
{
	/* control pwm to track desired rotor angular velocity*/
	math::Vector<2> k_c(_mixer_info.k_c, _mixer_info.k_c);
	math::Vector<2> pwm_ff =  _curr_omegas.emult(_curr_omegas) * _mixer_info.k_w2 +
							  _curr_omegas * _mixer_info.k_w +
							  k_c;
	//pwm_ff(0) = pwm_ff(0) +  _mixer_info.k_c;
	//pwm_ff(1) = pwm_ff(1) +  _mixer_info.k_c;

	math::Vector<2> desired;
	math::Vector<2> pwm_p;
	math::Vector<2> pwm_i;

	desired.set(omega_desired);
	math::Vector<2> error = desired - _curr_omegas;
	_pi_integrals += _pi_integrals + error * dt;

	for(int i=0; i<2; i++){
		_pi_integrals(i) = math::constrain(_pi_integrals(i), -_mixer_info.integral_lim, _mixer_info.integral_lim);
	}

	pwm_p = error * _mixer_info.k_p;
	pwm_i = _pi_integrals *_mixer_info.k_i;

	for(int i=0; i<2; i++){
		pwm_p(i) = math::constrain(pwm_p(i), -_mixer_info.p_term_lim, _mixer_info.p_term_lim);
		pwm_i(i) = math::constrain(pwm_i(i), -_mixer_info.int_term_lim, _mixer_info.int_term_lim);
	}


	math::Vector<2> outputs = pwm_ff + pwm_p + pwm_i;
	*pwm_outputs[0] = outputs(0);
	*pwm_outputs[1] = outputs(1);
}


