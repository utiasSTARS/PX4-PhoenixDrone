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

//namespace
//{
//
//float constrain(float val, float min, float max)
//{
//	return (val < min) ? min : ((val > max) ? max : val);
//}
//
//}

TailsitterMixer::TailsitterMixer(ControlCallback control_cb,
		uintptr_t cb_handle,
		mixer_ts_s *mixer_info):
				Mixer(control_cb, cb_handle),
				_mixer_info(*mixer_info)
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


unsigned
TailsitterMixer::mix(float *outputs, unsigned space, uint16_t *status_reg)
{
	return 3;
}

void
TailsitterMixer::groups_required(uint32_t &groups)
{
	/* XXX for now, hardcoded to indexes 0-3 in control group zero */
	groups |= (1 << 0);
}
