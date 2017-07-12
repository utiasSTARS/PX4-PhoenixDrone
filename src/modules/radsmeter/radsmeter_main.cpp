/*
 * radsmeter_main.cpp
 *
 *  Created on: Jun 12, 2017
 *      Author: yilun
 */


#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <drivers/drv_hrt.h>
#include <unistd.h>
#include <board_config.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/topics/esc_rads.h>
/****************
 * Pin Definition
 */
#define GPIO_LMOTOR_RPM_FB (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTE|GPIO_PIN11)

/***************
 * Parameters
 */
#define RADS_FILTER_CONSTANT 	0.2f
#define NUM_POLES 7
#define NUM_SYNC_PER_CYCLE 3
#define TIMEOUT_MS 10

/***********************************************
 * Function Declarations
 */
extern "C" __EXPORT int radsmeter_main(int argc, char *argv[]);
int radsmeterDaemon(void);
int StartDaemon (void);
int StopDaemon (void);
int PrintStatus(void);
void UpdateTimeDiffUp (void);
void UpdateTimeDiffDown (void);
int PublishRads (void);
int callback(int irq, FAR void *context);

/*****************
 * Global Variables
 */
orb_advert_t	_esc_rads_pub;
sem_t _sem_loop_timer;
static volatile bool thread_running = false;
static volatile uint64_t timeMeasUp[2], timeMeasDown[2];
static volatile uint8_t timeIdxUp, timeIdxDown = 0;
static volatile float timeDiffFilt = 0.0f;
static volatile float timeDiff = 0.0f;
int _radsmeter_pid = 0;
struct krads_data{
	uint64_t time;
	float krads;
	float krads_raw;
}_krads;

struct esc_rads_s esc_rads_msg;
int radsmeterDaemon(void)
{
	stm32_configgpio(GPIO_LMOTOR_RPM_FB);
	stm32_gpiosetevent(GPIO_LMOTOR_RPM_FB, true, false, false, callback);

	sem_init (&_sem_loop_timer, 0, 0);
	struct timespec time;
	while (thread_running){
		/* Setup a timedwait for TIMEOUT_MS:
			 * If we time out publish krads=0.0
			 */
		(void) clock_gettime (CLOCK_REALTIME, &time);
		time.tv_nsec += TIMEOUT_MS * 1000 * 1000;
		if (time.tv_nsec >= 1000 * 1000 * 1000)
		{
			time.tv_sec++;
			time.tv_nsec -= 1000 * 1000 * 1000;
		}
		int ret = sem_timedwait (&_sem_loop_timer, &time);

		if (ret == 0)
		{
			esc_rads_msg.timestamp = hrt_absolute_time();
			esc_rads_msg.rads_filtered[0] = 2 * 3.1415926f * 1000000.f / (NUM_POLES * NUM_SYNC_PER_CYCLE * timeDiffFilt);
			esc_rads_msg.rads_raw[0] = 2 * 3.1415926f * 1000000.f / (NUM_POLES * NUM_SYNC_PER_CYCLE * timeDiff);
			PublishRads();
		}
		/* timeout */
		else
		{
			//printf("timeout!\n");
			timeDiffFilt = 0.0f;
			timeDiff = 0.0f;
			esc_rads_msg.timestamp = hrt_absolute_time();
			esc_rads_msg.rads_filtered[0] = 0.0f;
			esc_rads_msg.rads_raw[0] = 0.0f;
			PublishRads();
		}

	}
	return 0;
}

int PublishRads ()
{
	if (_esc_rads_pub != nullptr) {
		return orb_publish(ORB_ID(esc_rads), _esc_rads_pub, &esc_rads_msg);

	} else {
		_esc_rads_pub = orb_advertise(ORB_ID(esc_rads), &esc_rads_msg);
	}
	return 0;
}

int StartDaemon (void)
{
	if (thread_running) {
		PX4_INFO("Radsmeter already running.");
		return 0;
	}

	thread_running = true;
	_radsmeter_pid = px4_task_spawn_cmd("radsmeter",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 1,
					   1000,
					   (px4_main_t)radsmeterDaemon,
					   nullptr);
	if (_radsmeter_pid != 0) PX4_INFO("Radsmeter started.");
	return 0;
}

int StopDaemon (void)
{
	if (!thread_running){
		PX4_INFO("Radsmeter not running.");
		return 0;
	}
	thread_running = false;
	exit(0);
	return 0;
}

int PrintStatus()
{
	printf("krads: %.3f\n", (double) esc_rads_msg.rads_filtered[0]);
	return 0;

}

void
UpdateTimeDiffUp (void)
{
	timeMeasUp[timeIdxUp] = hrt_absolute_time();
	timeDiffFilt = timeDiffFilt
			+ RADS_FILTER_CONSTANT
					* ((float) (timeMeasUp[timeIdxUp] - timeMeasUp[(timeIdxUp + 1) % 2])
							- timeDiffFilt);
	timeDiff = (float) (timeMeasUp[timeIdxUp] - timeMeasUp[(timeIdxUp + 1) % 2]);
	timeIdxUp = (timeIdxUp + 1) % 2;
}


void
UpdateTimeDiffDown (void)
{
	timeMeasDown[timeIdxDown] = hrt_absolute_time();
	timeDiffFilt = timeDiffFilt
			+ RADS_FILTER_CONSTANT
					* ((float) (timeMeasDown[timeIdxDown]
							- timeMeasDown[(timeIdxDown + 1) % 2]) - timeDiffFilt);
	timeDiff = (float) (timeMeasUp[timeIdxUp] - timeMeasUp[(timeIdxUp + 1) % 2]);
	timeIdxDown = (timeIdxDown + 1) % 2;
}

int
callback (int irq, FAR void *context)
{
	if (stm32_gpioread (GPIO_LMOTOR_RPM_FB))
		{
			UpdateTimeDiffUp();
		}
	else
		{
			UpdateTimeDiffDown();
		}

	int svalue;
	sem_getvalue (&_sem_loop_timer, &svalue);
	if (svalue < 0)
		sem_post (&_sem_loop_timer);

	return 0;
}

int
radsmeter_main(int argc, char *argv[])
{
	if (argc < 2) {
			PX4_WARN("usage: radsmeter {start|stop|status}");
			return 1;
		}

		if (!strcmp(argv[1], "start")) {
			return StartDaemon();
		}

		if (!strcmp(argv[1], "stop")) {
			return StopDaemon();
		}

		if (!strcmp(argv[1], "status")) {
			return PrintStatus();
		}

	PX4_INFO("radsmeter started.");
	return 0;

}

