/*
 * actuator_test.c
 *
 *  Created on: Jun 29, 2017
 *      Author: yilun
 */


#include <px4_config.h>
#include <px4_getopt.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <arch/board/board.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/actuator_controls.h>


#include "systemlib/systemlib.h"
#include "systemlib/err.h"

__EXPORT int actuator_test_main(int argc, char* argv[]);
void usage(void);
static orb_advert_t _actuator_test_pub = NULL;
static struct actuator_controls_s msg;
void usage()
{
	errx(1,
		 "usage:\n"
		 "actuator_test <channel> <normalized value>\n");
}


int actuator_test_main(int argc, char *argv[])
{
	if (argc < 3){
		usage();
		return 1;
	}

	int channel = (int)strtol(argv[1], NULL, 0);
	float value = (float)strtof(argv[2], NULL);


	msg.timestamp = hrt_absolute_time();
	//if (chanel == 4) channel = 5;//Go around the thrust index to avoid arming issue
	msg.control[channel - 1] = value;

	if (_actuator_test_pub != NULL){
		orb_publish(ORB_ID(actuator_controls_2), _actuator_test_pub, &msg);
	}
	else{
		_actuator_test_pub = orb_advertise(ORB_ID(actuator_controls_2), &msg);
	}

	return 0;

}
