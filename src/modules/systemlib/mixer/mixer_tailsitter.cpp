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
				_delta_out_max(0)
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

unsigned
TailsitterMixer::mix(float *outputs, unsigned space, uint16_t *status_reg)
{
	float rads_left  = constrain(get_control(0, 0), 0, _mixer_info.rads_max);
	float rads_right = constrain(get_control(0, 1), 0, _mixer_info.rads_max);

	float elv_left  = constrain(get_control(0, 2), _mixer_info.deg_min, _mixer_info.deg_max);
	float elv_right = constrain(get_control(0, 3), _mixer_info.deg_min, _mixer_info.deg_max);

	float mot_left  = _mixer_info.k_w2 * rads_left * rads_left
				+ _mixer_info.k_w * rads_left + _mixer_info.k_c;

	float mot_right = _mixer_info.k_w2 * rads_right * rads_right
				+ _mixer_info.k_w * rads_right + _mixer_info.k_c;

	outputs[0] = constrain(mot_left, -1.f, 1.f);
	outputs[1] = constrain(mot_right, -1.f, 1.f);

	outputs[2] = 0;//Not being used, will be set to NaN in tsfmu cycle
	outputs[3] = 0;

	//Elevon deflections are linear, just normalize between -1, 1
	outputs[4] = -normalize(elv_left , _mixer_info.deg_min, _mixer_info.deg_max, -1.f, 1.f);
	outputs[5] = normalize(elv_right, _mixer_info.deg_min, _mixer_info.deg_max, -1.f, 1.f)+0.12f;


	return 6;
}

void
TailsitterMixer::groups_required(uint32_t &groups)
{
	/* Subscribe group 0 and 1 from ts_actuator_controls*/
	groups |= (1 << 0);
	groups |= (1 << 1);
}
