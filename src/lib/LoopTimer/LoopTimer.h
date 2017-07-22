/*
 * LoopTimer.h
 *
 *  Created on: Jul 21, 2017
 *      Author: yilun
 */

#ifndef LIB_LOOPTIMER_LOOPTIMER_H_
#define LIB_LOOPTIMER_LOOPTIMER_H_

#include <semaphore.h>
#include <drivers/drv_hrt.h>

void OnTimer(void *p);

class LoopTimer {
public:
	LoopTimer(uint32_t period_us); /* period is in microseconds */
	~LoopTimer();
	void wait(void);

private:
	sem_t _sem_loop_timer;
	struct hrt_call _ol_tick_call;
	const uint32_t _period_us;
};



#endif /* LIB_LOOPTIMER_LOOPTIMER_H_ */
