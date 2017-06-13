/*
 * radsmeter-nuttx.cpp
 *
 *  Created on: Jun 12, 2017
 *      Author: yilun
 */


#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <stdbool.h>
#include <poll.h>
#include <pthread.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <cmath>
#include "stm32.h"
#include <nuttx/fs/nxffs.h>
#include <systemlibrary/systemlib.h>
#include <arch/board/idsc_highrestimer.h>
#include <nuttx/shared/shared.h>
#include "shared/krads.h"
#include "systemlibrary/convenience.h"
#include "shared/Publisher.h"
#include "parameter/ParameterServer.h"
#include "parameter/DefaultParameters.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define GPIO_MOTOR_0 GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN4
#define RADS_FILTER_CONSTANT 	0.2f
#define NUM_POLES 7
#define TIMEOUT_MS 10
/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
extern "C" __EXPORT int
radsmeter_main (int argc, char *argv[]);
int
radsmeterDaemon (int argc, char *argv[]);
int
callback (int irq, FAR void *context);
/****************************************************************************
 * Private Data
 ****************************************************************************/
sem_t _sem_loop_timer;
static volatile bool thread_running = false; /**< Daemon status flag */
static volatile uint64_t timeMeasUp[2], timeMeasDown[2];
static volatile uint8_t timeIdxUp, timeIdxDown = 0;
static volatile float timeDiffFilt = 0.0f;
static volatile float timeDiff = 0.0f;
struct krads_data _krads;
Parameters::OnboardParameters<Parameters::Radsmeter::NUM_PARAMS> _params("radsmeter");
/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: radsmeter_daemon
 ****************************************************************************/
int
radsmeterDaemon (int argc, char *argv[])
{

	stm32_configgpio (GPIO_MOTOR_0);
	stm32_gpiosetevent (GPIO_MOTOR_0, true, false, false, callback);

	sem_init (&_sem_loop_timer, 0, 0);

	Publisher kradsPub (KRADS_NAME, KRADS_SIZE);
	kradsPub.Init ();

	struct timespec time;

	while (thread_running)
		{
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
					_krads.time = hrt_absolute_time ();
					_krads.krads = 2 * M_PI * 1000.f / (NUM_POLES * timeDiffFilt);
					_krads.krads_raw = 2 * M_PI * 1000.f / (NUM_POLES * timeDiff);
					kradsPub.Publish ((uint8_t*) &_krads);
				}
			/* timeout */
			else
				{
					//printf("timeout!\n");
					timeDiffFilt = 0.0f;
					timeDiff = 0.0f;
					_krads.time = hrt_absolute_time ();
					_krads.krads = 0.0f;
					_krads.krads_raw = 0.0f;
					kradsPub.Publish ((uint8_t*) &_krads);
				}
		}

	return 0;
}

/****************************************************************************
 * Name: StartDaemon
 ****************************************************************************/

int
StartDaemon (void)
{
	if (thread_running)
		{
			printf ("Thread already running\n");
			return 0;
		}

	thread_running = true;


	_params.ReadFromFile();
	if (!_params.GetReadSuccess())
	{
		printf("Couldn't read %s! Setting hard defaults: flag = ", _params.GetName());
		SetDefaultRadsmeterParameters(_params);
		int flag = _params.WriteToFile();
		printf("%d\n", flag);
	}
	// start task
	task_spawn_cmd ("radsmeter_daemon", SCHED_DEFAULT, SCHED_PRIORITY_MAX - 1,
									1000, radsmeterDaemon,
									NULL);
	return 0;
}

/****************************************************************************
 * Name: StopDaemon
 ****************************************************************************/

int
StopDaemon (void)
{
	if (!thread_running)
		return 0;
	thread_running = false;
	sem_post (&_sem_loop_timer);
	exit (0);
	return 0;
}

/****************************************************************************
 * Name: Echo
 ****************************************************************************/

int
Echo ()
{
	clearscreen ();
	while (true)
		{

			movecursor ();

			printf ("krads: %.3f\n", _krads.krads);
			echosleep ();
		}
	return 0;
}

/****************************************************************************
 * Name: UpdateTimeDiffUp
 ****************************************************************************/

void
UpdateTimeDiffUp (void)
{
	timeMeasUp[timeIdxUp] = hrt_absolute_time ();
	timeDiffFilt = timeDiffFilt
			+ _params.Get (Parameters::Radsmeter::FILTERCONSTANT)
					* ((float) (timeMeasUp[timeIdxUp] - timeMeasUp[(timeIdxUp + 1) % 2])
							- timeDiffFilt);
	timeDiff = (float) (timeMeasUp[timeIdxUp] - timeMeasUp[(timeIdxUp + 1) % 2]);
	timeIdxUp = (timeIdxUp + 1) % 2;
}

/****************************************************************************
 * Name: UpdateTimeDiffDown
 ****************************************************************************/

void
UpdateTimeDiffDown (void)
{
	timeMeasDown[timeIdxDown] = hrt_absolute_time ();
	timeDiffFilt = timeDiffFilt
			+ _params.Get (Parameters::Radsmeter::FILTERCONSTANT)
					* ((float) (timeMeasDown[timeIdxDown]
							- timeMeasDown[(timeIdxDown + 1) % 2]) - timeDiffFilt);
	timeDiff = (float) (timeMeasUp[timeIdxUp] - timeMeasUp[(timeIdxUp + 1) % 2]);
	timeIdxDown = (timeIdxDown + 1) % 2;
}

/****************************************************************************
 * Name: callback
 ****************************************************************************/

int
callback (int irq, FAR void *context)
{
	if (stm32_gpioread (GPIO_MOTOR_0))
		{
			UpdateTimeDiffUp ();
		}
	else
		{
			UpdateTimeDiffDown ();
		}

	int svalue;
	sem_getvalue (&_sem_loop_timer, &svalue);
	if (svalue < 0)
		sem_post (&_sem_loop_timer);

	return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: radsmeter_main
 ****************************************************************************/

int
radsmeter_main (int argc, char *argv[])
{
	if (!strcmp (argv[1], "start"))
		return StartDaemon ();
	if (!strcmp (argv[1], "stop"))
		return StopDaemon ();
	if (!strcmp (argv[1], "echo"))
		return Echo ();

	return 0;
}

