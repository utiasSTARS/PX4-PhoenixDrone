/*
 * ts_mixer.cpp
 *
 *      author Xintong Du <xintong.du@mail.utoronto.ca>
 */
#include <math.h>
#include <lib/mathlib/mathlib.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include "ts_rate_control.h"

TailsitterRateControl::TailsitterRateControl():
_actuator_max{}
{
	param_t param_handle;
	float v;
	param_handle = param_find("TS_LIFT_CONST");
	param_get(param_handle, &v);
	_kl = v;

	param_handle = param_find("TS_DRAG_CONST");
	param_get(param_handle, &v);
	_kd = v;

	param_handle = param_find("TS_THRUST_CONST");
	param_get(param_handle, &v);
	_kt = v;

	param_handle = param_find("TS_MOMT_CONST_R");
	param_get(param_handle, &v);
	_l = v;

	param_handle = param_find("TS_MOMT_CONST_P");
	param_get(param_handle, &v);
	_kp = v;

	param_handle = param_find("TS_MOMT_CONST_Y");
	param_get(param_handle, &v);
	_km = v;

	param_handle = param_find("TS_MOT_MAX");
	param_get(param_handle, &v);
	_actuator_max(0) = v;
	_actuator_max(1) = v;

	param_handle = param_find("TS_SERVO_MAX");
	param_get(param_handle, &v);
	_actuator_max(2) = v;
	_actuator_max(3) = v;

}

TailsitterRateControl::~TailsitterRateControl(){

}

void TailsitterRateControl::mix(float fa, math::Vector<3> momentum_ref, float* actuator_outputs){
	math::Vector<4> outputs;
	outputs.zero();
	get_omega2_l(fa, momentum_ref, &outputs(0));
	get_omega2_r(fa, momentum_ref, &outputs(1));
	get_delta_r(fa, momentum_ref, &outputs(2));
	get_delta_l(fa, momentum_ref, &outputs(3));

	for(int i=0; i< 4; i++){
		if (i < 2){
			if(outputs(i) < 0){
				outputs(i) = 0;
				warnx("Negative omeg^2");
				warnx("fa: %f, momentum_ref: %f, %f, %f\n", (double) fa, (double) momentum_ref(0), (double) momentum_ref(1), (double) momentum_ref(2));
			}

			outputs(i) = sqrt(outputs(i));
			outputs(i) = outputs(i) / 800.0f;
		}
		else{
			outputs(i) = outputs(i) / (float) M_PI * 180.0f;
		}
	}


	for(int i=0; i<4; i++){
		outputs(i) = math::constrain(outputs(i), -_actuator_max(i), _actuator_max(i));
		actuator_outputs[i] = outputs(i);
	}
}

void TailsitterRateControl::get_omega2_l(float fa, math::Vector<3> momentum_ref, float* omega2_l){
	float mx = momentum_ref(0);
	*omega2_l = (mx + 2*fa*_l)/(2*_kt*_l);
}

void
TailsitterRateControl::get_omega2_r(float fa, math::Vector<3> momentum_ref, float* omega2_r){
	float mx = momentum_ref(0);
	*omega2_r = -(mx - 2*fa*_l)/(2*_kt*_l);
}

void TailsitterRateControl::get_delta_r(float fa, math::Vector<3> momentum_ref, float* delta_r){
	float mx = momentum_ref(0);
	float my = momentum_ref(1);
	float mz = momentum_ref(2);
	*delta_r = (_kl*_kt*my*_l*_l - _kp*_kt*mz*_l + _km*_kp*_kt*mx)/(_kl*_kp*_l*(mx - 2*fa*_l));
}

void TailsitterRateControl::get_delta_l(float fa, math::Vector<3> momentum_ref, float* delta_l){
	float mx = momentum_ref(0);
	float my = momentum_ref(1);
	float mz = momentum_ref(2);
	*delta_l = -(_kl*_kt*my*_l*_l + _kp*_kt*mz*_l - _km*_kp*_kt*mx)/(2*fa*_kl*_kp*_l*_l + _kl*_kp*mx*_l);
}






