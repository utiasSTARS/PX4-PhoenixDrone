/*
 * ts_rate_control.h
 *
 *  Created on: Nov 26, 2017
 *      Author: tracy
 */

#ifndef MODULES_TS_ATT_CONTROL_TS_RATE_CONTROL_H_
#define MODULES_TS_ATT_CONTROL_TS_RATE_CONTROL_H_


#include <math.h>
#include <lib/mathlib/mathlib.h>


class __EXPORT TailsitterRateControl{
public:
	TailsitterRateControl();
	~TailsitterRateControl();

	void mix(float fa, math::Vector<3> momentum_ref, float* actuator_outputs);

private:
	float _kp;	/* pitch constant */
	float _kd;	/* drag constant */
	float _kl;	/* lift constant */
	float _km;	/* motor constant */
	float _l;	/* thrust torque arm  */
	float _kt;	/* thrust constant */

	math::Vector<4> _actuator_max; /* maximum value of actuator outputs */

	void get_omega2_r(float fa, math::Vector<3> momentum_ref, float* omega2_r);
	void get_omega2_l(float fa, math::Vector<3> momentum_ref, float* omega2_l);
	void get_delta_r(float fa, math::Vector<3> momentum_ref, float* delta_r);
	void get_delta_l(float fa, math::Vector<3> momentum_ref, float* delta_l);
};


#endif /* MODULES_TS_ATT_CONTROL_TS_RATE_CONTROL_H_ */
