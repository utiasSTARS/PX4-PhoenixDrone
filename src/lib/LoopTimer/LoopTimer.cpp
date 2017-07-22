/*
 * LoopTimer.cpp
 *
 *  Created on: Jul 21, 2017
 *      Author: yilun
 */

#include "LoopTimer.h"
#include <cstring>

void OnTimer(void *p) {
	sem_t* sem_loop_timer = (sem_t*) p;
	int svalue;
	sem_getvalue(sem_loop_timer, &svalue);
	if (svalue < 0)
		sem_post(sem_loop_timer);
}

//***************************************************************************
// Public Functions
//***************************************************************************

LoopTimer::LoopTimer(uint32_t period_us) :
		_period_us(period_us) {
	sem_init(&_sem_loop_timer, 0, 0);

	memset(&_ol_tick_call, 0, sizeof(_ol_tick_call));
	hrt_call_every(&_ol_tick_call, period_us, period_us, &OnTimer,
			&_sem_loop_timer);

}

LoopTimer::~LoopTimer() {
	/* cancel the call to OnTimer */
	hrt_cancel(&_ol_tick_call);
	/* post one more time the semaphore to release sem_wait */
	int svalue;
	sem_getvalue(&_sem_loop_timer, &svalue);
	if (svalue < 0)
		sem_post(&_sem_loop_timer);
	/* destroy semaphore */
	sem_destroy(&_sem_loop_timer);
}

void LoopTimer::wait(void) {
	sem_wait(&_sem_loop_timer);
}
