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
#include <drivers/drv_hrt.h>

#include <px4iofirmware/protocol.h>
#include <drivers/drv_pwm_output.h>

#include "mixer.h"

#define debug(fmt, args...)	do { } while(0)
//#define debug(fmt, args...)	do { printf("[mixer] " fmt "\n", ##args); } while(0)
//#include <debug.h>
//#define debug(fmt, args...)	syslog(fmt "\n", ##args)

#define NAN_VALUE	(0.0f/0.0f)
#define TIMEOUT_MS 	10

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
				_prev_mixer_info{0},
				_delta_out_max(0),
				_rotor_controller_init(false),
				_last_control_timestamp(0),
				_pi_integrals(0,0),
				_curr_omegas(0,0),
				_curr_omegas_valid(false),
				_sem_mixer{0}

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
	mixer_info.k_w2[0] = s[3]/1.f;
	mixer_info.k_w2[1] = s[3]/1.f;
	mixer_info.k_w[0] = s[4]/1.f;
	mixer_info.k_w[1] = s[4]/1.f;
	mixer_info.k_c[0] = s[5]/1.f;
	mixer_info.k_c[1] = s[5]/1.f;

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
	memcpy(&_prev_mixer_info, &_mixer_info, sizeof(_mixer_info));
	sem_init(&_sem_mixer, 0, 1);
}

void
TailsitterMixer::clear_integral(int motor_index){
	_pi_integrals(motor_index) = 0;
}

void
TailsitterMixer::update_mixer_info(mixer_ts_s *mixer_info)
{
	struct timespec time;
	(void) clock_gettime (0, &time);
	time.tv_nsec += TIMEOUT_MS * 1000 * 1000;
	if (time.tv_nsec >= 1000 * 1000 * 1000)
	{
		time.tv_sec++;
		time.tv_nsec -= 1000 * 1000 * 1000;
	}

	int ret = 1;
	while(ret !=0){
		ret = sem_timedwait(&_sem_mixer,&time);

		if(ret == 0){
			memcpy(&_mixer_info, mixer_info, sizeof(*mixer_info));
			_report_mixer_info();
		}
		sem_post(&_sem_mixer);
	}

}

void
TailsitterMixer::update_mixer_info(float volt){
	float volt_scale = volt / _mixer_info.calib_volt;
	_mixer_info.k_p /= volt_scale;
	_mixer_info.k_i /= volt_scale;
	for(int i=0; i<2; i++){
		_mixer_info.k_w2[i] /= volt_scale;
		_mixer_info.k_w[i]  /= volt_scale;
		_mixer_info.k_c[i]   = _mixer_info.k_c[i] / volt_scale +
							   (1 - volt_scale) / volt_scale / 500.0f * 1500.f;
	}

}


void
TailsitterMixer::_report_mixer_info(){

	if(_prev_mixer_info.k_i - _mixer_info.k_i > 0.0000001f ||  _prev_mixer_info.k_i - _mixer_info.k_i <  -0.0000001f){
		printf("Ki changed from %.6f to %.6f\n",(double) _prev_mixer_info.k_i, (double) _mixer_info.k_i);
	}

	if(_prev_mixer_info.k_p - _mixer_info.k_p > 0.0000001f || _prev_mixer_info.k_p - _mixer_info.k_p <  -0.0000001f){
		printf("Kp changed from %.6f to %.6f\n",(double) _prev_mixer_info.k_p, (double) _mixer_info.k_p);
	}

	if(_prev_mixer_info.p_term_lim - _mixer_info.p_term_lim > 0.0000001f ||  _prev_mixer_info.p_term_lim - _mixer_info.p_term_lim <  -0.0000001f){
		printf("P term lim changed from %.6f to %.6f\n",(double) _prev_mixer_info.p_term_lim, (double) _mixer_info.p_term_lim);
	}

	if(_prev_mixer_info.int_term_lim - _mixer_info.int_term_lim > 0.0000001f ||  _prev_mixer_info.int_term_lim - _mixer_info.int_term_lim <  -0.0000001f){
		printf("I term lim changed from %.6f to %.6f\n",(double) _prev_mixer_info.int_term_lim, (double) _mixer_info.int_term_lim);
	}

	if(_prev_mixer_info.integral_lim - _mixer_info.integral_lim > 0.0000001f ||  _prev_mixer_info.integral_lim - _mixer_info.integral_lim <  -0.0000001f){
		printf("Integral lim changed from %.6f to %.6f\n",(double) _prev_mixer_info.integral_lim, (double) _mixer_info.integral_lim);
	}

	memcpy(&_prev_mixer_info, &_mixer_info, sizeof(_mixer_info));
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
	float mot_left = -1.f;
	float mot_right = -1.f;
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

	//printf("%.2f, %.2f\n", (double) mot_left, (double) mot_right);
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
	math::Vector<2> k_w2;
	math::Vector<2> k_w;
	math::Vector<2> k_c;
	math::Vector<2> desired;
	math::Vector<2> pwm_ff;
	math::Vector<2> pwm_p;
	math::Vector<2> pwm_i;
	math::Vector<2> pwm_p_cons;
	math::Vector<2> pwm_i_cons;

	int sem_value;
	sem_getvalue(&_sem_mixer, &sem_value);
	if(sem_value <= 0)
		printf("start waiting");

	sem_wait(&_sem_mixer);
	k_w2.set(_mixer_info.k_w2);
	k_w.set(_mixer_info.k_w);
	k_c.set(_mixer_info.k_c);
	desired.set(omega_desired);


	pwm_ff =  (_curr_omegas.emult(_curr_omegas)).emult(k_w2) +
							  	  	_curr_omegas.emult(k_w) +
								  	  	  	  	 	   k_c;

	pwm_ff = (desired.emult(desired)).emult(k_w2) +
							  desired.emult(k_w) +
										  	k_c;
	pwm_ff = pwm_ff / 12.5f * 12.2f;

	math::Vector<2> error = desired - _curr_omegas;
	_pi_integrals = _pi_integrals + error * dt;

	for(int i=0; i<2; i++){
		math::Vector<2> prev_pi_integrals = _pi_integrals;
		_pi_integrals(i) = math::constrain(_pi_integrals(i), -_mixer_info.integral_lim, _mixer_info.integral_lim);
		if (i == 1 && prev_pi_integrals(i) > _pi_integrals(i))
			printf("Motor %d Integral %.2f hits upper bound\n", i , (double) prev_pi_integrals(i));
		else if (i == 1 && _pi_integrals(i) > prev_pi_integrals(i))
			printf("Motor %d Integral %.2f hits lower bound\n", i , (double) prev_pi_integrals(i));
	}

	pwm_p = error * _mixer_info.k_p;
	//printf("%.2f, %.2f\n", (double) error(0), (double) _curr_omegas(0));
	pwm_i = _pi_integrals *_mixer_info.k_i;

	for(int i=0; i<2; i++){
		pwm_p_cons(i) = math::constrain(pwm_p(i), -_mixer_info.p_term_lim, _mixer_info.p_term_lim);
		pwm_i_cons(i) = math::constrain(pwm_i(i), -_mixer_info.int_term_lim, _mixer_info.int_term_lim);
		if(i == 1 && pwm_p_cons(i) < pwm_p(i))
			printf("Motor %d P term %.2f hits upper bound\n", i, (double) pwm_p(i));
		else if(i == 1 && pwm_p_cons(i) > pwm_p(i))
			printf("Motor %d P term %.2f hits lower bound\n", i, (double) pwm_p(i));
		if (i == 1 && pwm_i_cons(i) < pwm_i(i))
			printf("Motor %d I term %.2f hits upper bound\n", i, (double) pwm_i(i));
		else if (i == 1 && pwm_i_cons(i) > pwm_i(i))
			printf("Motor %d I term %.2f hits lower bound\n", i, (double) pwm_i(i));

	}



	math::Vector<2> outputs = pwm_ff; //+ pwm_p_cons + pwm_i_cons;
	*pwm_outputs[0] = outputs(0);
	*pwm_outputs[1] = outputs(1);

	sem_post(&_sem_mixer);

	//printf("%.2f,%.2f,%.2f,%.2f\n", (double) outputs(0), (double) error(0),(double) _curr_omegas(0),(double) _curr_omegas(1));
}


