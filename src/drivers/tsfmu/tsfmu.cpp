/*
 * tsfmu.cpp
 *
 *  Created on: Jun 29, 2017
 *      Author: yilun
 */

#include <math.h>
#include <px4_config.h>
#include <px4_tasks.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <float.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <lib/mathlib/mathlib.h>

#include <nuttx/arch.h>

#include <platforms/px4_workqueue.h>

#include <drivers/device/device.h>
#include <drivers/device/i2c.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_input_capture.h>
#include <drivers/drv_gpio.h>
#include <drivers/drv_hrt.h>

#include <board_config.h>

#include <systemlib/px4_macros.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <systemlib/mixer/mixer.h>
#include <systemlib/pwm_limit/pwm_limit.h>
#include <systemlib/board_serial.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>
#include <drivers/drv_mixer.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_input_capture.h>


#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/ts_actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/esc_rads.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/safety.h>


#include <systemlib/circuit_breaker.h>

#define TSFMU_DEV_PATH "/dev/tsfmu"

#define PWM_SERVO_MIN 900
#define PWM_SERVO_MAX 2100
#define SCHEDULE_INTERVAL	2000	/**< The schedule interval in usec (500 Hz) */
#define NAN_VALUE	(0.0f/0.0f)		/**< NaN value for throttle lock mode */
#define BUTTON_SAFETY	px4_arch_gpioread(GPIO_BTN_SAFETY)
#define CYCLE_COUNT 10			/* safety switch must be held for 1 second to activate */
#define RPM_CH_LEFT 2
#define RPM_CH_RIGHT 3
#define NUM_POLES 7
#define NUM_SYNC_PER_CYCLE 3
#define TIMER_PSC 8
#define PI 3.14159f
#define RADS_FILTER_CONSTANT 0.8f
#define TIMEOUT_MS 200
#define CH_TIMEOUT_MS 100
#define MED_FIL_ENTRY 5
#define CALIB_NUM_INTVL 50

/*
 * Define the various LED flash sequences for each system state.
 */
#define LED_PATTERN_FMU_OK_TO_ARM 		0x0003		/**< slow blinking			*/
#define LED_PATTERN_FMU_REFUSE_TO_ARM 	0x5555		/**< fast blinking			*/
#define LED_PATTERN_IO_ARMED 			0x5050		/**< long off, then double blink 	*/
#define LED_PATTERN_FMU_ARMED 			0x5500		/**< long off, then quad blink 		*/
#define LED_PATTERN_IO_FMU_ARMED 		0xffff		/**< constantly on			*/

#if !defined(BOARD_HAS_PWM)
#  error "board_config.h needs to define BOARD_HAS_PWM"
#endif



class TSFMU : public device::CDev
{
public:
	TSFMU();
	~TSFMU();

	int	ioctl(file *filp, int cmd, unsigned long arg);
	virtual ssize_t	write(file *filp, const char *buffer, size_t len);

	virtual int	init();

	int		set_mode();

	int		set_pwm_alt_rate(unsigned rate);
	int		set_pwm_alt_channels(uint32_t channels);

	static int	set_i2c_bus_clock(unsigned bus, unsigned clock_hz);

	static void	capture_trampoline(void *context, uint32_t chan_index,
					   hrt_abstime edge_time, uint32_t edge_state,
					   uint32_t overflow);

	int cancel_hp_work();
	int	start_hp_work();
	void radsmeter_stop();
	uint32_t show_pwm_mask();

	void print_info();
	void perf_counter_reset();
	void calibrate(int argc, char* argv[]);
	void update_mixer_params(int argc, char *argv[]);
private:

	bool _report_lock = true;
	bool _got_first_cmd = false;


	hrt_abstime _cycle_timestamp = 0;
	hrt_abstime _last_safety_check = 0;
	hrt_abstime _time_last_mix = 0;

	static const unsigned _max_actuators = 6; //DIRECT_PWM_OUTPUT_CHANNELS; //defined in board_config.h
	static const unsigned _max_timers = 4;

	unsigned	_pwm_default_rate;
	unsigned	_pwm_alt_rate;
	uint32_t	_pwm_alt_rate_channels;
	unsigned	_current_update_rate;
	struct work_s	_work;
	int		_vehicle_cmd_sub;
	int		_armed_sub;
	int		_param_sub;
	int		_esc_rads_sub;
	orb_advert_t	_outputs_pub;
	unsigned	_num_outputs;
	int		_class_instance;

	struct esc_rads_s  _esc_rads_msg;
	volatile bool	_initialized;
	volatile bool   _rotor_controller_init;
	bool		_throttle_armed;
	bool		_pwm_on;
	uint32_t	_pwm_mask;
	bool		_pwm_initialized;

	//MixerGroup	*_mixers;
	TailsitterMixer *_ts_mixer;
	mixer_ts_s	_mixer_info;


	uint32_t	_groups_required;
	uint32_t	_groups_subscribed;


	unsigned	_poll_fds_num;
	/*Tailsitter actuator subscriptions*/
	int		_ts_control_subs[ts_actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	ts_actuator_controls_s _ts_controls[ts_actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	orb_id_t	_ts_control_topics[ts_actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	pollfd  _poll_fds[ts_actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];


	static pwm_limit_t	_pwm_limit;
	static actuator_armed_s	_armed;
	float _outputs[_max_actuators];
	uint16_t	_failsafe_pwm[_max_actuators];
	uint16_t	_disarmed_pwm[_max_actuators];
	uint16_t	_min_pwm[_max_actuators];
	uint16_t	_max_pwm[_max_actuators];
	uint16_t	_trim_pwm[_max_actuators];
	uint16_t	_reverse_pwm_mask;
	unsigned	_num_failsafe_set;
	unsigned	_num_disarmed_set;
	bool		_safety_off;
	bool		_safety_disabled;
	orb_advert_t		_to_safety;
	orb_advert_t      _to_mixer_status; 	///< mixer status flags

	float _mot_t_max;	// maximum rise time for motor (slew rate limiting)
	float _thr_mdl_fac;	// thrust to pwm modelling factor

	/* Motor RPM Pulse Detection */
	bool _rads_task_should_exit;
	int _rads_task;
	sem_t _sem_timer;
	volatile uint64_t _last_edge_l;
	volatile uint64_t _last_edge_r;
	volatile uint64_t _current_edge_l;
	volatile uint64_t _current_edge_r;
	volatile uint8_t _timeIdx_l;
	volatile uint8_t _timeIdx_r;
	volatile uint64_t _time_l[MED_FIL_ENTRY];
	volatile uint64_t _time_r[MED_FIL_ENTRY];
	volatile uint64_t _timeDiff_l;
	volatile uint64_t _timeDiff_r;
	volatile uint64_t _timeDiff_l_fil;
	volatile uint64_t _timeDiff_r_fil;
	volatile float _rads_l;
	volatile float _rads_r;
	volatile float _rads_l_raw;
	volatile float _rads_r_raw;
	orb_advert_t	_rads_pub;

	/* PWM Calibration */
	bool	_is_calib;
	float	_outputs_calib[4];
	float	_esc_rads_calib[2][10];
	int 	_esc_rads_curr_indx_calib;
	float   _esc_rads_avg_calib[2];
	sem_t 	_sem_timer_calib;


	perf_counter_t	_ctl_latency;
	perf_counter_t  _capture_cb;

	/*Debug Only, to be deleted */
	volatile uint64_t ridi_timediffs[5];
	volatile uint8_t ridi_idx;

	static bool	arm_nothrottle()
	{
		return ((_armed.prearmed && !_armed.armed) || _armed.in_esc_calibration_mode);
	}

	static void	cycle_trampoline(void *arg);
	void		cycle();
	void		work_start();
	void		work_stop();

	static int ts_control_callback(uintptr_t handle,
					 uint8_t control_group,
					 uint8_t control_index,
					 float &input);
	void 		reset_rads_meas(int ch);
	static void rads_task_main_trampoline(int argc, char *argv[]);
	void 		rads_task_main(void);
	void		capture_callback(uint32_t chan_index,
					 hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow);
	void		subscribe();
	void 		ts_subscribe();
	int		set_pwm_rate(unsigned rate_map, unsigned default_rate, unsigned alt_rate);
	int		pwm_ioctl(file *filp, int cmd, unsigned long arg);
	void		publish_pwm_outputs(uint16_t *values, size_t numvalues);
	void		update_pwm_out_state(bool on);
	void		pwm_output_set(unsigned i, unsigned value);
	uint64_t	check_anomaly(volatile uint64_t * array, unsigned size, uint64_t newest);

	struct GPIOConfig {
		uint32_t	input;
		uint32_t	output;
		uint32_t	alt;
	};

	static const GPIOConfig	_gpio_tab[];
	static const unsigned	_ngpio;

	void		gpio_reset(void);
	void		sensor_reset(int ms);
	void		peripheral_reset(int ms);
	void		gpio_set_function(uint32_t gpios, int function);
	void		gpio_write(uint32_t gpios, int function);
	uint32_t	gpio_read(void);
	int		gpio_ioctl(file *filp, int cmd, unsigned long arg);

	int		capture_ioctl(file *filp, int cmd, unsigned long arg);

	/* do not allow to copy due to ptr data members */
	TSFMU(const TSFMU &);
	TSFMU operator=(const TSFMU &);

	void safety_check_button(void);
	void flash_safety_button(void);

};

const TSFMU::GPIOConfig TSFMU::_gpio_tab[] =	BOARD_FMU_GPIO_TAB;

const unsigned		TSFMU::_ngpio = arraySize(TSFMU::_gpio_tab);
pwm_limit_t		TSFMU::_pwm_limit;
actuator_armed_s	TSFMU::_armed = {};

namespace
{

TSFMU	*g_fmu;

} // namespace

TSFMU::TSFMU() :
	CDev("tsfmu", TSFMU_DEV_PATH),
	_pwm_default_rate(50),
	_pwm_alt_rate(50),
	_pwm_alt_rate_channels(0),
	_current_update_rate(0),
	_work{},
	_vehicle_cmd_sub(-1),
	_armed_sub(-1),
	_param_sub(-1),
	_esc_rads_sub(-1),
	_outputs_pub(nullptr),
	_num_outputs(0),
	_class_instance(0),
	_esc_rads_msg{},
	_initialized(false),
	_rotor_controller_init(false),
	_throttle_armed(false),
	_pwm_on(false),
	_pwm_mask(0),
	_pwm_initialized(false),
	_ts_mixer(nullptr),
	_mixer_info{0},
	_groups_required(0),
	_groups_subscribed(0),
	_poll_fds_num(0),
	_ts_control_subs{ -1},
	_outputs{0},
	_failsafe_pwm{0},
	_disarmed_pwm{0},
	_reverse_pwm_mask(0),
	_num_failsafe_set(0),
	_num_disarmed_set(0),
	_safety_off(false),
	_safety_disabled(false),
	_to_safety(nullptr),
	_to_mixer_status(nullptr),
	_mot_t_max(0.0f),
	_thr_mdl_fac(0.0f),
	_rads_task_should_exit(false),
	_rads_task(-1),
	_sem_timer{0},
	_last_edge_l(0),
	_last_edge_r(0),
	_current_edge_l(0),
	_current_edge_r(0),
	_timeIdx_l(0),
	_timeIdx_r(0),
	_time_l{0},
	_time_r{0},
	_timeDiff_l(0),
	_timeDiff_r(0),
	_timeDiff_l_fil(0),
	_timeDiff_r_fil(0),
	_rads_l(0.f),
	_rads_r(0.f),
	_rads_l_raw(0.f),
	_rads_r_raw(0.f),
	_rads_pub(nullptr),
	_is_calib(false),
	_outputs_calib{0},
	_esc_rads_calib{0},
	_esc_rads_curr_indx_calib(0),
	_esc_rads_avg_calib{0},
	_sem_timer_calib{0},
	_ctl_latency(perf_alloc(PC_ELAPSED, "ctl_lat")),
	_capture_cb(perf_alloc(PC_ELAPSED, "capture callback")),
	ridi_timediffs{0},
	ridi_idx(0)
{
	/* force output rates for TS on PixRacer */
	_pwm_default_rate = 200;//Digital Servos seem to be okay with 200Hz PWM Signal
	_pwm_alt_rate = 500;//ESCs use 500Hz PWM Signal
	_pwm_alt_rate_channels = 0x0f;//Channel 1,2,3,4 have alt rate
	_pwm_mask = 0x33;//Enable Channel 1,2,5,6
	_pwm_initialized = false;


	for (unsigned i = 0; i < _max_actuators; i++) {
		_min_pwm[i] = PWM_DEFAULT_MIN;
		_max_pwm[i] = PWM_DEFAULT_MAX;
		_trim_pwm[i] = PWM_DEFAULT_TRIM;
		_failsafe_pwm[i] = 950;
		_disarmed_pwm[i] = 900;
	}

	//Special PWM_MIN and PWM_MAX for the servos
	for (unsigned i = 4; i <= 5; i++) {
		_min_pwm[i] = PWM_SERVO_MIN;
		_max_pwm[i] = PWM_SERVO_MAX;
		_failsafe_pwm[i] = (PWM_SERVO_MIN + PWM_SERVO_MAX) / 2;
		_disarmed_pwm[i] = 500;
	}

	_num_disarmed_set = 4;
	_num_failsafe_set = 4;

	_ts_control_topics[0] = ORB_ID(ts_actuator_controls_0);
	_ts_control_topics[1] = ORB_ID(ts_actuator_controls_1);


	memset(_ts_controls, 0, sizeof(_ts_controls));
	memset(_poll_fds, 0, sizeof(_poll_fds));

	_mixer_info.rads_max = 1000.f;
	_mixer_info.deg_max = 50.f;
	_mixer_info.deg_min = -50.f;
	_mixer_info.k_c[0] = -1.136835f;
	_mixer_info.k_c[1] = -1.240911466216619;
	_mixer_info.k_w2[0] = -0.000000030499f;//1.f;
	_mixer_info.k_w2[1] = -0.000000019738231f;
	_mixer_info.k_w[0] = 0.001752517190f;//1.f;
	_mixer_info.k_w[1] = 0.001705341665494f;
	_mixer_info.k_p = 0.005f;//0.002f;
	_mixer_info.k_i = 0;//0.1647;//0.0262f;
	_mixer_info.int_term_lim = 50000.f;
	_mixer_info.p_term_lim = 1000.f;
	_mixer_info.control_interval = SCHEDULE_INTERVAL;
	_mixer_info.integral_lim = 10000.f;
	_mixer_info.calib_volt = 12.5f;


	_ts_mixer = new TailsitterMixer(ts_control_callback, (uintptr_t)_ts_controls, &_mixer_info);
	_ts_mixer->groups_required(_groups_required);

	// Safely initialize armed flags.
	_armed.armed = false;
	_armed.prearmed = false;
	_armed.ready_to_arm = false;
	_armed.lockdown = false;
	_armed.force_failsafe = false;
	_armed.in_esc_calibration_mode = false;


	// If there is no safety button, disable it on boot.
#ifndef GPIO_BTN_SAFETY
	_safety_off = true;
#endif

	/* only enable this during development */
	_debug_enabled = false;

	sem_init(&_sem_timer, 0, 0);
	sem_init(&_sem_timer_calib, 0, 1);
}

TSFMU::~TSFMU()
{
	if (_initialized) {
		/* tell the task we want it to go away */
		work_stop();

		int i = 10;

		do {
			/* wait 50ms - it should wake every 100ms or so worst-case */
			usleep(50000);
			i--;

		} while (_initialized && i > 0);
	}

	if (_rads_task != -1) {
		_rads_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_rads_task);
				break;
			}
		} while (_rads_task != -1);
	}

	/* clean up the alternate device node */
	unregister_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH, _class_instance);

	/* clean up subscribed and published topics */
	orb_unadvertise(_outputs_pub);
	orb_unadvertise(_to_safety);
	orb_unadvertise(_to_mixer_status);
	orb_unadvertise(_rads_pub);
	orb_unsubscribe(_vehicle_cmd_sub);
	orb_unsubscribe(_armed_sub);
	orb_unsubscribe(_param_sub);


	perf_free(_ctl_latency);
	perf_free(_capture_cb);

	g_fmu = nullptr;
}

uint32_t
TSFMU::show_pwm_mask()
{
	return _pwm_mask;
}

int
TSFMU::init()
{
	int ret;

	ASSERT(!_initialized);

	/* do regular cdev init */
	ret = CDev::init();

	if (ret != OK) {
		errx(1, "CDev init failed.");
		return ret;
	}

	// XXX best would be to register / de-register the device depending on modes

	/* try to claim the generic PWM output device node as well - it's OK if we fail at this */
	_class_instance = register_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH);

	if (_class_instance == CLASS_DEVICE_PRIMARY) {
		/* lets not be too verbose */
	} else if (_class_instance < 0) {
		warnx("FAILED registering class device");
	}

	_safety_disabled = circuit_breaker_enabled("CBRK_IO_SAFETY", CBRK_IO_SAFETY_KEY);

	work_start();

	/*Input Capture settings for reading motor RPM pulses */
	up_input_capture_set(RPM_CH_LEFT, Both, 0, &capture_trampoline, this);
	up_input_capture_set(RPM_CH_RIGHT, Both, 0, &capture_trampoline, this);


	return OK;
}

void
TSFMU::safety_check_button(void)
{
#ifdef GPIO_BTN_SAFETY
	static int counter = 0;
	/*
	 * Debounce the safety button, change state if it has been held for long enough.
	 *
	 */
	bool safety_button_pressed = BUTTON_SAFETY;

	/*
	 * Keep pressed for a while to arm.
	 *
	 * Note that the counting sequence has to be same length
	 * for arming / disarming in order to end up as proper
	 * state machine, keep ARM_COUNTER_THRESHOLD the same
	 * length in all cases of the if/else struct below.
	 */
	if (safety_button_pressed && !_safety_off) {

		if (counter < CYCLE_COUNT) {
			counter++;

		} else if (counter == CYCLE_COUNT) {
			/* switch to armed state */
			_safety_off = true;
			counter++;
		}

	} else if (safety_button_pressed && _safety_off) {

		if (counter < CYCLE_COUNT) {
			counter++;

		} else if (counter == CYCLE_COUNT) {
			/* change to disarmed state and notify the FMU */
			_safety_off = false;
			counter++;
		}

	} else {
		counter = 0;
	}

#endif
}

void
TSFMU::flash_safety_button()
{
#ifdef GPIO_BTN_SAFETY

	/* Select the appropriate LED flash pattern depending on the current arm state */
	uint16_t pattern = LED_PATTERN_FMU_REFUSE_TO_ARM;

	/* cycle the blink state machine at 10Hz */
	static int blink_counter = 0;

	if (_safety_off) {
		if (_armed.armed) {
			pattern = LED_PATTERN_IO_FMU_ARMED;

		} else {
			pattern = LED_PATTERN_IO_ARMED;
		}

	} else if (_armed.armed) {
		pattern = LED_PATTERN_FMU_ARMED;

	} else {
		pattern = LED_PATTERN_FMU_OK_TO_ARM;

	}

	/* Turn the LED on if we have a 1 at the current bit position */
	px4_arch_gpiowrite(GPIO_LED_SAFETY, !(pattern & (1 << blink_counter++)));

	if (blink_counter > 15) {
		blink_counter = 0;
	}

#endif
}



int
TSFMU::set_pwm_rate(uint32_t rate_map, unsigned default_rate, unsigned alt_rate)
{
	PX4_DEBUG("set_pwm_rate %x %u %u", rate_map, default_rate, alt_rate);

	for (unsigned pass = 0; pass < 2; pass++) {
		for (unsigned group = 0; group < _max_timers; group++) {

			// get the channel mask for this rate group
			uint32_t mask = up_pwm_servo_get_rate_group(group);

			if (mask == 0) {
				continue;
			}

			// all channels in the group must be either default or alt-rate
			uint32_t alt = rate_map & mask;

			if (pass == 0) {
				// preflight
				if ((alt != 0) && (alt != mask)) {
					warn("rate group %u mask %x bad overlap %x", group, mask, alt);
					// not a legal map, bail
					return -EINVAL;
				}

			} else {
				// set it - errors here are unexpected
				if (alt != 0) {
					if (up_pwm_servo_set_rate_group_update(group, _pwm_alt_rate) != OK) {
						warn("rate group set alt failed");
						return -EINVAL;
					}

				} else {
					if (up_pwm_servo_set_rate_group_update(group, _pwm_default_rate) != OK) {
						warn("rate group set default failed");
						return -EINVAL;
					}
				}
			}
		}
	}

	_pwm_alt_rate_channels = rate_map;
	_pwm_default_rate = default_rate;
	_pwm_alt_rate = alt_rate;

	return OK;
}

int
TSFMU::set_pwm_alt_rate(unsigned rate)
{
	return set_pwm_rate(_pwm_alt_rate_channels, _pwm_default_rate, rate);
}

int
TSFMU::set_pwm_alt_channels(uint32_t channels)
{
	return set_pwm_rate(channels, _pwm_default_rate, _pwm_alt_rate);
}

int
TSFMU::set_i2c_bus_clock(unsigned bus, unsigned clock_hz)
{
	return device::I2C::set_bus_clock(bus, clock_hz);
}

void
TSFMU::subscribe()
{
	_poll_fds_num = 0;

	for (unsigned i = 0; i < ts_actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++)
	{
		//Subscribe to TS actuator control topics
		_ts_control_subs[i] = orb_subscribe(_ts_control_topics[i]);

		//Set up Polling
		if (_ts_control_subs[i] > 0) {
			_poll_fds[_poll_fds_num].fd = _ts_control_subs[i];
			_poll_fds[_poll_fds_num].events = POLLIN;
			_poll_fds_num++;
			_groups_subscribed |= (1<<i);
		}
	}
}




void
TSFMU::publish_pwm_outputs(uint16_t *values, size_t numvalues)
{
	actuator_outputs_s outputs = {};
	outputs.noutputs = numvalues;
	outputs.timestamp = hrt_absolute_time();

	for (size_t i = 0; i < _max_actuators; ++i) {
		outputs.output[i] = i < numvalues ? (float)values[i] : 0;
	}

	if (_outputs_pub == nullptr) {
		int instance = _class_instance;
		_outputs_pub = orb_advertise_multi(ORB_ID(actuator_outputs), &outputs, &instance, ORB_PRIO_DEFAULT);

	} else {
		orb_publish(ORB_ID(actuator_outputs), _outputs_pub, &outputs);
	}
}


void
TSFMU::work_start()
{
	ASSERT(_rads_task == -1);

	/* start radsmeter task */
	_rads_task = px4_task_spawn_cmd("radsmeter",
					SCHED_DEFAULT,
					SCHED_PRIORITY_MAX - 1,
					1000,
					(px4_main_t)&TSFMU::rads_task_main_trampoline,
					nullptr);

	if (_rads_task < 0) {
		warn("task start failed");
	}
	else PX4_INFO("radsmeter started.");

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&TSFMU::cycle_trampoline, this, 0);
}

void
TSFMU::cycle_trampoline(void *arg)
{
	TSFMU *dev = reinterpret_cast<TSFMU *>(arg);

	dev->cycle();
}

void
TSFMU::reset_rads_meas(int ch)
{
	if (ch == RPM_CH_LEFT)
	{
		_last_edge_l = 0;
		_current_edge_l = 0;
		_timeDiff_l = 0;
		_timeDiff_l_fil = 0;
		for(unsigned i = 0; i < MED_FIL_ENTRY; i ++) _time_l[i] = 0;
		_rads_l = 0.f;
		_rads_l_raw = 0.f;
		//printf("reset left!\n");
	}
	if (ch == RPM_CH_RIGHT)
	{
		_last_edge_r = 0;
		_current_edge_r = 0;
		_timeDiff_r = 0;
		_timeDiff_r_fil = 0;
		for(unsigned i = 0; i < MED_FIL_ENTRY; i ++) _time_r[i] = 0;
		_rads_r = 0.f;
		_rads_r_raw = 0.f;
		//printf("reset right!\n");
	}
}

void
TSFMU::rads_task_main_trampoline(int argc, char *argv[])
{
	g_fmu->rads_task_main();
}

void
TSFMU::rads_task_main()
{
	struct timespec time;
	struct esc_rads_s esc_rads_msg;
	while(!_rads_task_should_exit) {
		(void) clock_gettime (CLOCK_REALTIME, &time);
		time.tv_nsec += TIMEOUT_MS * 1000 * 1000;
		if (time.tv_nsec >= 1000 * 1000 * 1000)
		{
			time.tv_sec++;
			time.tv_nsec -= 1000 * 1000 * 1000;
		}
		int ret = sem_timedwait (&_sem_timer, &time);
		hrt_abstime now = hrt_absolute_time();
		if (ret == 0)
		{
			/*Check if any timeDiff is invalid */
			if ((now - CH_TIMEOUT_MS * 1000 ) > _current_edge_l) {
				//printf("L-now:%llu\t current_edge:%llu\n", now, _current_edge_l);
				reset_rads_meas(RPM_CH_LEFT);
			}
			if ((now - CH_TIMEOUT_MS * 1000) >  _current_edge_r) {
				//printf("R-now:%llu\t current_edge:%llu\n", now, _current_edge_r);
				reset_rads_meas(RPM_CH_RIGHT);
			}


			_rads_l = _timeDiff_l_fil ? TIMER_PSC * PI * 1000000.f / (float)(NUM_POLES * NUM_SYNC_PER_CYCLE * _timeDiff_l_fil) : 0.f;
			_rads_l_raw = _timeDiff_l ? TIMER_PSC * PI * 1000000.f / (float) (NUM_POLES * NUM_SYNC_PER_CYCLE * _timeDiff_l) : 0.f;
			_rads_r = _timeDiff_r_fil ? TIMER_PSC * PI * 1000000.f / (float)(NUM_POLES * NUM_SYNC_PER_CYCLE * _timeDiff_r_fil) : 0.f;
			_rads_r_raw = _timeDiff_r ? TIMER_PSC * PI * 1000000.f / (float) (NUM_POLES * NUM_SYNC_PER_CYCLE * _timeDiff_r) : 0.f;
		}
		/* timeout */
		else
		{
			//printf("semaphore timeout!\n");
			reset_rads_meas(RPM_CH_LEFT);
			reset_rads_meas(RPM_CH_RIGHT);
		}
		esc_rads_msg.timestamp = now;
		esc_rads_msg.rads_filtered[0] = _rads_l;
		esc_rads_msg.rads_raw[0] = _rads_l_raw;
		esc_rads_msg.rads_filtered[1] = _rads_r;
		esc_rads_msg.rads_raw[1] = _rads_r_raw;
		esc_rads_msg.rads_filtered[2] = 0;
		esc_rads_msg.rads_raw[2] = 0;
		esc_rads_msg.rads_filtered[3] = 0;
		esc_rads_msg.rads_raw[3] = 0;

		if(esc_rads_msg.rads_raw[0] < 0 || esc_rads_msg.rads_raw[0] > 10000.f){
			esc_rads_msg.rads_raw[0] = 0;
			warnx("esc rads 0 not normal.\n");
		}
		if(esc_rads_msg.rads_raw[1] < 0 || esc_rads_msg.rads_raw[1] > 10000.f){
			esc_rads_msg.rads_raw[1] = 0;
			warnx("esc rads 1 not normal.\n");
		}
		if (_rads_pub != nullptr) {
			orb_publish(ORB_ID(esc_rads), _rads_pub, &esc_rads_msg);


		} else {
			_rads_pub = orb_advertise(ORB_ID(esc_rads), &esc_rads_msg);
		}
	}

	_rads_task = -1;
}


void
TSFMU::capture_trampoline(void *context, uint32_t chan_index,
			   hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow)
{
	TSFMU *dev = reinterpret_cast<TSFMU *>(context);
	dev->capture_callback(chan_index, edge_time, edge_state, overflow);
}

void
TSFMU::capture_callback(uint32_t chan_index,
			 hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow)
{
	perf_begin(_capture_cb);
	//fprintf(stdout, "FMU: Capture chan:%d time:%lld state:%d overflow:%d\n", chan_index, edge_time, edge_state, overflow);
	bool valid = true;
	//tested: overflow does not occur as long as there is no printf in the callback which takes 3500-4000us each printf
	//if (overflow) warnx("overflow: %x", overflow);
	if (chan_index == RPM_CH_LEFT){
		_current_edge_l = edge_time;
		if ((_current_edge_l  - _last_edge_l) > CH_TIMEOUT_MS * 1000){
			ridi_timediffs[ridi_idx] = _current_edge_l - _last_edge_l;
			ridi_idx = (ridi_idx + 1) % 5;
			valid = false;
			//printf("L-current_edge:\t%llu\n", _current_edge_l);
			//printf("L-last_edge:\t%llu\n", _last_edge_l);
			//printf("left timediff timeout\n");

		}
		else{
			_timeDiff_l = (_current_edge_l - _last_edge_l);

			_time_l[_timeIdx_l] = _timeDiff_l;
			_timeIdx_l = (_timeIdx_l + 1) % MED_FIL_ENTRY;

			uint64_t timediff_valid = check_anomaly(_time_l, MED_FIL_ENTRY, _timeDiff_l);
			_timeDiff_l_fil = RADS_FILTER_CONSTANT * _timeDiff_l_fil +
							(1 - RADS_FILTER_CONSTANT) * timediff_valid;
		}
		_last_edge_l = edge_time;




	}

	if (chan_index == RPM_CH_RIGHT){
		_current_edge_r = edge_time;

		if ((_current_edge_r - _last_edge_r) > CH_TIMEOUT_MS * 1000){
			ridi_timediffs[ridi_idx] = _current_edge_r - _last_edge_r;
			ridi_idx = (ridi_idx + 1) % 5;
			valid = false;
			//printf("R-current_edge:\t%llu\n", _current_edge_r);
			//printf("R-last_edge:\t%llu\n", _last_edge_r);
			//printf("right timediff timeout\n");
		}
		else{
			_timeDiff_r = (_current_edge_r - _last_edge_r);
			_time_r[_timeIdx_r] = _timeDiff_r;
			_timeIdx_r = (_timeIdx_r + 1) % MED_FIL_ENTRY;

			uint64_t timediff_valid = check_anomaly(_time_r, MED_FIL_ENTRY, edge_time);
			_timeDiff_r_fil = RADS_FILTER_CONSTANT * _timeDiff_r_fil +
					(1 - RADS_FILTER_CONSTANT) * timediff_valid;
		}

		_last_edge_r = edge_time;


	}

	/* Signal the rads thread to continue execuation */
	int svalue;
	sem_getvalue (&_sem_timer, &svalue);
	if (valid && svalue < 0) sem_post(&_sem_timer);
	perf_end(_capture_cb);
}

//Bubble sort implementation of finding median, timer input capture tends to
//miss edges or overcount edges
uint64_t
TSFMU::check_anomaly(volatile uint64_t* array, unsigned size, uint64_t newest){
	uint64_t sorted[size];
	for (unsigned i = 0; i < size; i ++) sorted[i] = array[i];
	for (unsigned i = size - 1; i > 0; --i){
		for (unsigned j = 0; j < i; ++j){
			if (sorted[j] > sorted[j + 1]) {
				uint64_t temp = sorted[j];
				sorted[j] = sorted[j + 1];
				sorted[j + 1] = temp;
			}
		}
	}
	uint64_t median = sorted[size/2];
	return (newest > 1.1 * median || newest < 0.9 * median) ? median : newest;
}

void
TSFMU::pwm_output_set(unsigned i, unsigned value)
{
	if (_pwm_initialized) {
		up_pwm_servo_set(i, value);
	}
}

void
TSFMU::update_pwm_out_state(bool on)
{
	if (on && !_pwm_initialized && _pwm_mask != 0) {
		up_pwm_servo_init(_pwm_mask);
		set_pwm_rate(_pwm_alt_rate_channels, _pwm_default_rate, _pwm_alt_rate);
		_pwm_initialized = true;
	}

	up_pwm_servo_arm(on);
}

void
TSFMU::cycle()
{
	if (!_initialized) {
		/* force a reset of the update rate */
		_current_update_rate = 0;

		_armed_sub = orb_subscribe(ORB_ID(actuator_armed));
		_param_sub = orb_subscribe(ORB_ID(parameter_update));
		_esc_rads_sub = orb_subscribe(ORB_ID(esc_rads));

		/* initialize PWM limit lib */
		pwm_limit_init(&_pwm_limit);//set limit->state=PWM_LIMIT_STATE_INIT, limit->time_armed = 0

		_initialized = true;
	}

	if (_groups_subscribed != _groups_required) {
		subscribe();
		/* force setting update rate */
		_current_update_rate = 0;
	}

	/*
	 * Adjust actuator topic update rate to keep up with
	 * the highest servo update rate configured.
	 *
	 * We always mix at max rate; some channels may update slower.
	 */
	unsigned max_rate = (_pwm_default_rate > _pwm_alt_rate) ? _pwm_default_rate : _pwm_alt_rate;

	if (_current_update_rate != max_rate) {
		_current_update_rate = max_rate;
		int update_rate_in_ms = int(1000 / _current_update_rate);

		/* reject faster than 500 Hz updates */
		if (update_rate_in_ms < 2) {
			update_rate_in_ms = 2;
		}

		/* reject slower than 10 Hz updates */
		if (update_rate_in_ms > 100) {
			update_rate_in_ms = 100;
		}

		PX4_DEBUG("adjusted actuator update interval to %ums", update_rate_in_ms);

		for (unsigned i = 0; i < ts_actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
			if (_ts_control_subs[i] > 0) {
				orb_set_interval(_ts_control_subs[i], update_rate_in_ms);
			}
		}

		// set to current max rate, even if we are actually checking slower/faster
		_current_update_rate = max_rate;
	}

	/* check if anything updated */
	int ret = ::poll(_poll_fds, _poll_fds_num, 0);//currently no timeout limit

	/* this would be bad... */
	if (ret < 0) {
		DEVICE_LOG("poll error %d", errno);

	} else if (ret == 0) {
		/* timeout: no control data, switch to failsafe values */
//			warnx("no PWM: failsafe");

	} else {
		perf_begin(_ctl_latency);

		/* get controls for required topics */
		unsigned poll_id = 0;

		for (unsigned i = 0; i < ts_actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
			if (_ts_control_subs[i] > 0) {
				if (_poll_fds[poll_id].revents & POLLIN) {
					orb_copy(_ts_control_topics[i], _ts_control_subs[i], &_ts_controls[i]);

#if defined(DEBUG_BUILD)

					static int main_out_latency = 0;
					static int sum_latency = 0;
					static uint64_t last_cycle_time = 0;

					if (i == 0) {
						uint64_t now = hrt_absolute_time();
						uint64_t latency = now - _controls[i].timestamp;

						if (latency > main_out_latency) { main_out_latency = latency; }

						sum_latency += latency;

						if ((now - last_cycle_time) >= 1000000) {
							last_cycle_time = now;
							PX4_DEBUG("pwm max latency: %d, avg: %5.3f", main_out_latency, (double)(sum_latency / 100.0));
							main_out_latency = latency;
							sum_latency = 0;
						}
					}

#endif
				}

				poll_id++;
			}

			/* During ESC calibration, we overwrite the throttle value. */
			//ESC calibration code removed
		}
	} // poll_fds

	/* run the mixers on every cycle */
	{
		/* can we mix? */
		if (_ts_mixer != nullptr) {

			size_t num_outputs = 6;

			hrt_abstime now = hrt_absolute_time();
			float dt = (now - _time_last_mix) / 1e6f;
			_time_last_mix = now;

			if (dt < 0.0001f) {
				dt = 0.0001f;

			} else if (dt > 0.02f) {
				dt = 0.02f;
			}

			if (_mot_t_max > FLT_EPSILON) {
				// maximum value the ouputs of the multirotor mixer are allowed to change in this cycle
				// factor 2 is needed because actuator ouputs are in the range [-1,1]
				float delta_out_max = 2.0f * 1000.0f * dt / (_max_pwm[0] - _min_pwm[0]) / _mot_t_max;
				_ts_mixer->set_max_delta_out_once(delta_out_max);
			}

			/* do mixing */
			float outputs[_max_actuators] = {0};

			/* check esc rads updates*/
			px4_pollfd_struct_t fds[1];
			bool meas_valid = true;
			struct esc_rads_s	esc_rads_msg;

			fds[0].fd = _esc_rads_sub;
			fds[0].events = POLLIN;

			int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 0.1);

			/* timed out */
			if (pret == 0) {
				//warn("esc rads: poll time out %d, %d", pret, errno);
			}

			/* this is undesirable but not much we can do - might want to flag unhappy status */
			if (pret < 0) {
				warn("esc rads: poll error %d, %d", pret, errno);
				/* sleep a bit before next try */
				usleep(100000);
				meas_valid = false;
			}

			/* read esc_rads and setup pi controller*/
			if (pret > 0 && (fds[0].revents & POLLIN)) {
				orb_copy(ORB_ID(esc_rads), _esc_rads_sub, &esc_rads_msg);

				sem_wait(&_sem_timer_calib);
				_esc_rads_msg = esc_rads_msg;
				sem_post(&_sem_timer_calib);

				float esc_rads_meas[2] = {_esc_rads_msg.rads_filtered[0],
										   _esc_rads_msg.rads_filtered[1]};
				_ts_mixer->set_curr_omega(esc_rads_meas);

				/*book keep rads to get moving avg*/
				if(_is_calib){
					for(int i=0; i<2; i++){
						_esc_rads_calib[i][_esc_rads_curr_indx_calib] = esc_rads_meas[i];
						float sum = 0;
						for(int j=0; j<10; j++){
							sum += _esc_rads_calib[i][j];
						}
						_esc_rads_avg_calib[i] = sum / 10.f;
					}

					_esc_rads_curr_indx_calib++;
					_esc_rads_curr_indx_calib %= 10;
				}
			}

			/*pi controller controls*/
			if (_rotor_controller_init){

				if(!_got_first_cmd && _ts_controls[0].control[0] > 122.f && _ts_controls[0].control[1] > 122.f){
					_got_first_cmd = true;
					_ts_mixer->clear_integral(0);
					_ts_mixer->clear_integral(1);
//					printf("integrals cleared");
				}
				_ts_mixer->set_curr_omega_valid(meas_valid);
				// TODO: get voltage from topics
				_ts_mixer->update_mixer_info(12.5f);
				num_outputs = _ts_mixer->mix(outputs, num_outputs, NULL);

			}
			else{
				_ts_mixer->init_rotor_controller();
				_rotor_controller_init = true;
			}

			/*if the rads measurement is outdated, keep the previous control*/
			if (!meas_valid){
				outputs[0] = _outputs[0];
				outputs[1] = _outputs[1];
			}


			/* disable unused ports by setting their output to NaN */
			outputs[2] = NAN_VALUE;
			outputs[3] = NAN_VALUE;

			/* override outputs with cmd line input if calibrating*/
			if (_is_calib){
//				int sem_value;
//				sem_getvalue(&_sem_timer_calib, &sem_value);
//				if (sem_value == 0){
//					sem_wait(&_sem_timer_calib);
//					sem_post(&_sem_timer_calib);
//				}
				sem_wait(&_sem_timer_calib);
				memcpy(outputs, _outputs_calib, sizeof(_outputs_calib));
				sem_post(&_sem_timer_calib);
			}

			memcpy(_outputs, outputs, sizeof(outputs));

			uint16_t pwm_limited[_max_actuators];
			uint16_t min_pwm_armed[_max_actuators];
			memcpy(min_pwm_armed, _min_pwm, sizeof(_min_pwm));
			min_pwm_armed[0] += 10;
			min_pwm_armed[1] += 10;

			/* the PWM limit call takes care of out of band errors, NaN and constrains */
			pwm_limit_calc(_throttle_armed, arm_nothrottle(), num_outputs, _reverse_pwm_mask,
				       _disarmed_pwm, min_pwm_armed, _max_pwm, outputs, pwm_limited, &_pwm_limit);

			//warn("pre_armed: %d",arm_nothrottle());
			/* overwrite outputs in case of force_failsafe with _failsafe_pwm PWM values */
			if (_armed.force_failsafe) {
				for (size_t i = 0; i < num_outputs; i++) {
					pwm_limited[i] = _failsafe_pwm[i];
				}
			}

			/* overwrite outputs in case of lockdown with disarmed PWM values */
			if (_armed.lockdown || _armed.manual_lockdown) {
				for (size_t i = 0; i < num_outputs; i++) {
					pwm_limited[i] = _disarmed_pwm[i];
				}
			}

			/* output to the servos */
			for (size_t i = 0; i < num_outputs; i++) {
				pwm_output_set(i, pwm_limited[i]);
			}

			publish_pwm_outputs(pwm_limited, num_outputs);
			perf_end(_ctl_latency);
		}
	}
//	} // poll_fds

	_cycle_timestamp = hrt_absolute_time();

#ifdef GPIO_BTN_SAFETY

	if (_cycle_timestamp - _last_safety_check >= (unsigned int)1e5) {
		_last_safety_check = _cycle_timestamp;

		/**
		 * Get and handle the safety status at 10Hz
		 */
		struct safety_s safety = {};

		if (_safety_disabled) {
			_safety_off = true;

		} else {
			/* read safety switch input and control safety switch LED at 10Hz */
			safety_check_button();
		}

		/* Make the safety button flash anyway, no matter if it's used or not. */
		flash_safety_button();

		safety.timestamp = hrt_absolute_time();

		if (_safety_off) {
			safety.safety_off = true;
			safety.safety_switch_available = true;

		} else {
			safety.safety_off = false;
			safety.safety_switch_available = true;
		}

		/* lazily publish the safety status */
		if (_to_safety != nullptr) {
			orb_publish(ORB_ID(safety), _to_safety, &safety);

		} else {
			int instance = _class_instance;
			_to_safety = orb_advertise_multi(ORB_ID(safety), &safety, &instance, ORB_PRIO_DEFAULT);
		}
	}

#endif
	/* check arming state */
	bool updated = false;
	orb_check(_armed_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);

		/* Update the armed status and check that we're not locked down.
		 * We also need to arm throttle for the ESC calibration. */
		_throttle_armed = (_safety_off && _armed.armed && !_armed.lockdown) ||
				  (_safety_off && _armed.in_esc_calibration_mode);


		/* update PWM status if armed or if disarmed PWM values are set */
		bool pwm_on = _armed.armed || _num_disarmed_set > 0 || _armed.in_esc_calibration_mode;

		if (_pwm_on != pwm_on) {
			_pwm_on = pwm_on;

			update_pwm_out_state(pwm_on);
		}
	}


	orb_check(_param_sub, &updated);

	if (updated) {
		parameter_update_s pupdate;
		orb_copy(ORB_ID(parameter_update), _param_sub, &pupdate);

		//update mixer parameters as you like

		param_t param_handle;


		// maximum motor slew rate parameter
		param_handle = param_find("MOT_SLEW_MAX");

		if (param_handle != PARAM_INVALID) {
			param_get(param_handle, &_mot_t_max);
		}

		// thrust to pwm modelling factor
		param_handle = param_find("THR_MDL_FAC");

		if (param_handle != PARAM_INVALID) {
			param_get(param_handle, &_thr_mdl_fac);
		}

		// update rpm controller gain
		param_handle = param_find("TS_RPMC_P");
		if(param_handle != PARAM_INVALID){
			param_get(param_handle, &_mixer_info.k_p);
			_ts_mixer->update_mixer_info(&_mixer_info);
		}

		param_handle = param_find("TS_RPMC_I");
		if(param_handle != PARAM_INVALID){
			param_get(param_handle, &_mixer_info.k_i);
			_ts_mixer->update_mixer_info(&_mixer_info);
		}

		param_handle = param_find("TS_RPMC_PL");
		if(param_handle != PARAM_INVALID){
			param_get(param_handle, &_mixer_info.p_term_lim);
			_ts_mixer->update_mixer_info(&_mixer_info);
		}

	}



	/*
	 * schedule next cycle
	 */
	work_queue(HPWORK, &_work, (worker_t)&TSFMU::cycle_trampoline, this, USEC2TICK(SCHEDULE_INTERVAL));
//		   USEC2TICK(SCHEDULE_INTERVAL - main_out_latency));
}

void TSFMU::work_stop()
{
	work_cancel(HPWORK, &_work);

	for (unsigned i = 0; i < ts_actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		if (_ts_control_subs[i] > 0) {
			orb_unsubscribe(_ts_control_subs[i]);
			_ts_control_subs[i] = -1;
		}
	}

	orb_unsubscribe(_armed_sub);
	orb_unsubscribe(_param_sub);

	/* make sure servos are off */
	up_pwm_servo_deinit();

	DEVICE_LOG("stopping");

	/* note - someone else is responsible for restoring the GPIO config */

	/* tell the dtor that we are exiting */
	_initialized = false;
}

void TSFMU::radsmeter_stop()
{
	if (_rads_task != -1) {
		_rads_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_rads_task);
				break;
			}
		} while (_rads_task != -1);
	}
	PX4_INFO("radsmeter stopped.");
}


int
TSFMU::ts_control_callback(uintptr_t handle,
			 uint8_t control_group,
			 uint8_t control_index,
			 float &input)
{
	const ts_actuator_controls_s *controls = (ts_actuator_controls_s *)handle;

	input = controls[control_group].control[control_index];


	/* motor spinup phase - lock throttle to zero */
	if (_pwm_limit.state == PWM_LIMIT_STATE_RAMP) {
		if ((control_group == ts_actuator_controls_s::GROUP_INDEX_TS_ATTITUDE ||
		     control_group == ts_actuator_controls_s::GROUP_INDEX_TS_ATTITUDE_ALTERNATE) &&
				(control_index == ts_actuator_controls_s::INDEX_RPM_LEFT ||
				control_index == ts_actuator_controls_s::INDEX_RPM_RIGHT)) {
			/* limit the throttle output to zero during motor spinup,
			 * as the motors cannot follow any demand yet
			 */
			input = 0.0f;
		}
	}

	/* throttle not arming - mark throttle input as invalid */
	if (arm_nothrottle() && !_armed.in_esc_calibration_mode) {
		if ((control_group == ts_actuator_controls_s::GROUP_INDEX_TS_ATTITUDE ||
		     control_group == ts_actuator_controls_s::GROUP_INDEX_TS_ATTITUDE_ALTERNATE) &&
		    (control_index == ts_actuator_controls_s::INDEX_RPM_LEFT ||
			control_index == ts_actuator_controls_s::INDEX_RPM_RIGHT)) 		{
			/* set the throttle to an invalid value */
			input = NAN_VALUE;
		}
	}

	return 0;
}

int
TSFMU::ioctl(file *filp, int cmd, unsigned long arg)
{
	int ret = pwm_ioctl(filp, cmd, arg);

	if (ret != -ENOTTY) {
		return ret;
	}

	ret = capture_ioctl(filp, cmd, arg);

	/* if nobody wants it, let CDev have it */
	if (ret == -ENOTTY) {
		ret = CDev::ioctl(filp, cmd, arg);
	}

	return ret;
}

int
TSFMU::pwm_ioctl(file *filp, int cmd, unsigned long arg)
{
	int ret = OK;

	PX4_DEBUG("fmu ioctl cmd: %d, arg: %ld", cmd, arg);

	lock();

	switch (cmd) {
	case PWM_SERVO_ARM:
		update_pwm_out_state(true);
		break;

	case PWM_SERVO_SET_ARM_OK:
	case PWM_SERVO_CLEAR_ARM_OK:
		break;

	case PWM_SERVO_SET_FORCE_SAFETY_OFF:
		/* force safety switch off */
		_safety_off = true;
		break;

	case PWM_SERVO_SET_FORCE_SAFETY_ON:
		/* force safety switch on */
		_safety_off = false;
		break;

	case PWM_SERVO_DISARM:

		/* Ignore disarm if disarmed PWM is set already. */
		if (_num_disarmed_set == 0) {
			update_pwm_out_state(false);
		}

		break;

	case PWM_SERVO_GET_DEFAULT_UPDATE_RATE:
		*(uint32_t *)arg = _pwm_default_rate;
		break;

	case PWM_SERVO_SET_UPDATE_RATE:
		ret = set_pwm_rate(_pwm_alt_rate_channels, _pwm_default_rate, arg);
		break;

	case PWM_SERVO_GET_UPDATE_RATE:
		*(uint32_t *)arg = _pwm_alt_rate;
		break;

	case PWM_SERVO_SET_SELECT_UPDATE_RATE:
		ret = set_pwm_rate(arg, _pwm_default_rate, _pwm_alt_rate);
		break;

	case PWM_SERVO_GET_SELECT_UPDATE_RATE:
		*(uint32_t *)arg = _pwm_alt_rate_channels;
		break;

	case PWM_SERVO_SET_FAILSAFE_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			/* discard if too many values are sent */
			if (pwm->channel_count > _max_actuators) {
				ret = -EINVAL;
				break;
			}

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				if (pwm->values[i] == 0) {
					/* ignore 0 */
				} else if (pwm->values[i] > PWM_HIGHEST_MAX) {
					_failsafe_pwm[i] = PWM_HIGHEST_MAX;

				}

#if PWM_LOWEST_MIN > 0

				else if (pwm->values[i] < PWM_LOWEST_MIN) {
					_failsafe_pwm[i] = PWM_LOWEST_MIN;

				}

#endif

				else {
					_failsafe_pwm[i] = pwm->values[i];
				}
			}

			/*
			 * update the counter
			 * this is needed to decide if disarmed PWM output should be turned on or not
			 */
			_num_failsafe_set = 0;

			for (unsigned i = 0; i < _max_actuators; i++) {
				if (_failsafe_pwm[i] > 0) {
					_num_failsafe_set++;
				}
			}

			break;
		}

	case PWM_SERVO_GET_FAILSAFE_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _max_actuators; i++) {
				pwm->values[i] = _failsafe_pwm[i];
			}

			pwm->channel_count = _max_actuators;
			break;
		}

	case PWM_SERVO_SET_DISARMED_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			/* discard if too many values are sent */
			if (pwm->channel_count > _max_actuators) {
				ret = -EINVAL;
				break;
			}

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				if (pwm->values[i] == 0) {
					/* ignore 0 */
				} else if (pwm->values[i] > PWM_HIGHEST_MAX) {
					_disarmed_pwm[i] = PWM_HIGHEST_MAX;
				}

#if PWM_LOWEST_MIN > 0

				else if (pwm->values[i] < PWM_LOWEST_MIN) {
					_disarmed_pwm[i] = PWM_LOWEST_MIN;
				}

#endif

				else {
					_disarmed_pwm[i] = pwm->values[i];
				}
			}

			/*
			 * update the counter
			 * this is needed to decide if disarmed PWM output should be turned on or not
			 */
			_num_disarmed_set = 0;

			for (unsigned i = 0; i < _max_actuators; i++) {
				if (_disarmed_pwm[i] > 0) {
					_num_disarmed_set++;
				}
			}

			break;
		}

	case PWM_SERVO_GET_DISARMED_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _max_actuators; i++) {
				pwm->values[i] = _disarmed_pwm[i];
			}

			pwm->channel_count = _max_actuators;
			break;
		}

	case PWM_SERVO_SET_MIN_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			/* discard if too many values are sent */
			if (pwm->channel_count > _max_actuators) {
				ret = -EINVAL;
				break;
			}

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				if (pwm->values[i] == 0) {
					/* ignore 0 */
				} else if (pwm->values[i] > PWM_HIGHEST_MIN) {
					_min_pwm[i] = PWM_HIGHEST_MIN;

				}

#if PWM_LOWEST_MIN > 0

				else if (pwm->values[i] < PWM_LOWEST_MIN) {
					_min_pwm[i] = PWM_LOWEST_MIN;
				}

#endif

				else {
					_min_pwm[i] = pwm->values[i];
				}
			}

			break;
		}

	case PWM_SERVO_GET_MIN_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _max_actuators; i++) {
				pwm->values[i] = _min_pwm[i];
			}

			pwm->channel_count = _max_actuators;
			arg = (unsigned long)&pwm;
			break;
		}

	case PWM_SERVO_SET_MAX_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			/* discard if too many values are sent */
			if (pwm->channel_count > _max_actuators) {
				ret = -EINVAL;
				break;
			}

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				if (pwm->values[i] == 0) {
					/* ignore 0 */
				} else if (pwm->values[i] < PWM_LOWEST_MAX) {
					_max_pwm[i] = PWM_LOWEST_MAX;

				} else if (pwm->values[i] > PWM_HIGHEST_MAX) {
					_max_pwm[i] = PWM_HIGHEST_MAX;

				} else {
					_max_pwm[i] = pwm->values[i];
				}
			}

			break;
		}

	case PWM_SERVO_GET_MAX_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _max_actuators; i++) {
				pwm->values[i] = _max_pwm[i];
			}

			pwm->channel_count = _max_actuators;
			arg = (unsigned long)&pwm;
			break;
		}

	case PWM_SERVO_SET_TRIM_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			/* discard if too many values are sent */
			if (pwm->channel_count > _max_actuators) {
				PX4_DEBUG("error: too many trim values: %d", pwm->channel_count);
				ret = -EINVAL;
				break;
			}

			/* copy the trim values to the mixer offsets */
			//_ts_mixer->set_trims((int16_t *)pwm->values, pwm->channel_count);
			PX4_DEBUG("set_trims: %d, %d, %d, %d", pwm->values[0], pwm->values[1], pwm->values[2], pwm->values[3]);

			break;
		}

	case PWM_SERVO_GET_TRIM_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _max_actuators; i++) {
				pwm->values[i] = _trim_pwm[i];
			}

			pwm->channel_count = _max_actuators;
			arg = (unsigned long)&pwm;
			break;
		}




	case PWM_SERVO_SET(5):
	case PWM_SERVO_SET(4):
	case PWM_SERVO_SET(3):
	case PWM_SERVO_SET(2):
	case PWM_SERVO_SET(1):
	case PWM_SERVO_SET(0):
		if (arg <= 2100) {
			ret = up_pwm_servo_set(cmd - PWM_SERVO_SET(0), arg);

		} else {
			ret = -EINVAL;
		}

		break;



	case PWM_SERVO_GET(5):
	case PWM_SERVO_GET(4):
	case PWM_SERVO_GET(3):
	case PWM_SERVO_GET(2):
	case PWM_SERVO_GET(1):
	case PWM_SERVO_GET(0):
		*(servo_position_t *)arg = up_pwm_servo_get(cmd - PWM_SERVO_GET(0));
		break;

	case PWM_SERVO_GET_RATEGROUP(0):
	case PWM_SERVO_GET_RATEGROUP(1):
	case PWM_SERVO_GET_RATEGROUP(2):
	case PWM_SERVO_GET_RATEGROUP(3):
	case PWM_SERVO_GET_RATEGROUP(4):
	case PWM_SERVO_GET_RATEGROUP(5):

		*(uint32_t *)arg = up_pwm_servo_get_rate_group(cmd - PWM_SERVO_GET_RATEGROUP(0));
		break;

	case PWM_SERVO_GET_COUNT:
	case MIXERIOCGETOUTPUTCOUNT:

		*(unsigned *)arg = 6;
		break;


	default:
		ret = -ENOTTY;
		break;
	}

	unlock();

	return ret;
}

void
TSFMU::sensor_reset(int ms)
{
	if (ms < 1) {
		ms = 1;
	}

	board_spi_reset(ms);
}

void
TSFMU::peripheral_reset(int ms)
{
	if (ms < 1) {
		ms = 10;
	}

	board_peripheral_reset(ms);
}
void
TSFMU::calibrate(int argc, char *argv[])
{
	actuator_armed_s aa;

	aa.armed = true;
	aa.prearmed = true;
	aa.ready_to_arm = true;
	aa.lockdown = false;
	aa.manual_lockdown = false;
	aa.force_failsafe = false;
	aa.in_esc_calibration_mode = false;


	orb_advert_t handle = orb_advertise(ORB_ID(actuator_armed), &aa);
	if (handle == nullptr) {
		errx(1, "advertise failed 2");
	}

	orb_unadvertise(handle);

	usleep(4000000);
	_is_calib = true;

	float pwm_outputs[4];
	float esc_rads_avg[2];

	int num_intervals = CALIB_NUM_INTVL;
	float h = 2.f / num_intervals;

	FILE *fd;
	float A_l[num_intervals][3];
	float A_r[num_intervals][3];
	float y_l[num_intervals];
	float y_r[num_intervals];


	for (int i=0; i<num_intervals; i++){
		for(int j=0; j<2; j++){
			pwm_outputs[j] = -1.f + 0.01f + i * h;
			pwm_outputs[j] = (pwm_outputs[j] > 1) ? 1 : pwm_outputs[j];
			pwm_outputs[j+2] = 0;
		}
		sem_wait(&_sem_timer_calib);

		memcpy(_outputs_calib, pwm_outputs, sizeof(pwm_outputs));
		sem_post(&_sem_timer_calib);

		usleep(800000);

		sem_wait(&_sem_timer_calib);
		memcpy(esc_rads_avg, _esc_rads_avg_calib, sizeof(_esc_rads_avg_calib));
		sem_post(&_sem_timer_calib);
		printf("PWM1: %.2f, PWM2: %.2f, RPM_L: %.2f, RMP_R: %.2f, RPM_L_AVG: %.2f, RPM_R_AVG:%.2f\n",
							(double) pwm_outputs[0], (double) pwm_outputs[1],
							(double) (_esc_rads_msg.rads_filtered[0]  * 60.f / (2.f*PI)), (double) (_esc_rads_msg.rads_filtered[1]  * 60.f / (2.f*PI)),
							(double) (esc_rads_avg[0] * 60 / (2.f * PI)),(double) (esc_rads_avg[1] * 60 / (2.f * PI)));

		y_l[i] = pwm_outputs[0];
		y_r[i] = pwm_outputs[1];

		A_l[i][0] = esc_rads_avg[0] * esc_rads_avg[0];
		A_l[i][1] = esc_rads_avg[0];
		A_l[i][2] = 1;

		A_r[i][0] = esc_rads_avg[1] * esc_rads_avg[1];
		A_r[i][1] = esc_rads_avg[1];
		A_r[i][2] = 1;

		char buffer[100] = {0};
		if(i==0)
			fd = fopen("/fs/microsd/rotor_calib.txt", "w");
		else
			fd = fopen("/fs/microsd/rotor_calib.txt", "a");
		int buffer_len;
		int bytes_written;

		if (fd != NULL) {
			sprintf(buffer,"%.5f %.5f %.5f %.5f\n",(double) pwm_outputs[0],
												   (double) pwm_outputs[1],
												   (double) esc_rads_avg[0],
												   (double) esc_rads_avg[1]);
			buffer_len = strlen(buffer) + 1;
			bytes_written = fwrite(buffer, 1, buffer_len, fd);
			if (bytes_written != buffer_len) {
				warn("fwrite() %d bytes returned less than expected %d", buffer_len, bytes_written);
			}

			fflush(fd);
			fclose(fd);

		}
		else{
			warn("Can not open file!");
		}
	}

//  linear regression test
//	for(int j=0; j<NUM_INTVL_CALIB; j++){
//		A_l[j][0] = j*j;
//		A_l[j][1] = j;
//		A_l[j][2] = 1;
//		y_l[j] = j*j + 2 * j + 3;
//		A_r[j][0] = j*j;
//		A_r[j][1] = j;
//		A_r[j][2] = 1;
//		y_r[j] = 1.5 * j*j + 3.43 * j + 5.24;
//	}



	math::Matrix<CALIB_NUM_INTVL, 3> A(A_l);
	math::Matrix<3, CALIB_NUM_INTVL> AT = A.transposed();
	math::Vector<CALIB_NUM_INTVL> y(y_l);
	math::Matrix<3,3> ATA_inv = (AT * A).inversed();
	math::Vector<3> coeff_l = (ATA_inv * AT) * y;

	A.set(A_r);
	AT = A.transposed();
	y.set(y_r);
	ATA_inv = (AT * A).inversed();
	math::Vector<3> coeff_r = (ATA_inv * AT) * y;

	printf("pwm-omega fit results,\n L: %.10f, %.10f, %.10f\n R: %.10f, %.10f, %.10f\n",
			(double) coeff_l(0),(double) coeff_l(1),(double) coeff_l(2),
			(double) coeff_r(0),(double) coeff_r(1),(double) coeff_r(2));

	pwm_outputs[0] = -1;

	pwm_outputs[1] = -1;
	memcpy(_outputs_calib, pwm_outputs, sizeof(pwm_outputs));
	usleep(1e6);
	_is_calib = false;

	/*test rise time*/

//	ts_actuator_controls_s ac;
//
//	ac.control[0] = 0.f;
//
//	ac.control[1] = 0.f;
//
//	ac.control[2] = 0.f;
//
//	ac.control[3] = 0.f;
//
//	handle = orb_advertise(ORB_ID(ts_actuator_controls_0), &ac);
//
//	if (handle == nullptr) {
//		errx(1, "advertise failed");
//	}
//	printf("published first cmd\n");
//	usleep(8000000);
//
//
//	ac.control[0] = 300.f;
//
//	ac.control[1] = 300.f;
//
//	handle = orb_advertise(ORB_ID(ts_actuator_controls_0), &ac);
//
//	if (handle == nullptr) {
//		errx(1, "advertise failed");
//	}
//	printf("published 500 cmd\n");
//	usleep(4000000);
//
//	esc_rads_s esc_rads_msg;
//	int esc_rads_sub = -1;
//	esc_rads_sub = orb_subscribe(ORB_ID(esc_rads));
//	px4_pollfd_struct_t fds[1];
//	fds[0].fd = esc_rads_sub;
//	fds[0].events = POLLIN;
//
//	int pret = 0;
//	while(pret<=0){
//		pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 0.1);
//		/* timed out */
//		if (pret == 0) {
//			//warn("esc rads: poll time out %d, %d", pret, errno);
//		}
//
//		/* this is undesirable but not much we can do - might want to flag unhappy status */
//		if (pret < 0) {
//			warn("esc rads: poll error %d, %d", pret, errno);
//		}
//
//		/* read esc_rads and setup pi controller*/
//		if (pret > 0 && (fds[0].revents & POLLIN)) {
//			orb_copy(ORB_ID(esc_rads), esc_rads_sub, &esc_rads_msg);
//		}
//	}
//
//
//
////	sem_wait(&_sem_timer_calib);
////	memcpy(&esc_rads_msg, &_esc_rads_msg, sizeof(_esc_rads_msg));
////	sem_post(&_sem_timer_calib);
//
//	float start[2]= {esc_rads_msg.rads_filtered[0], esc_rads_msg.rads_filtered[1]};
//	int hit_check_point[2] = {2,0};
//	int hit_check_point1[2] = {2,0};
//	float jump_step = 200.f;
//	float check_point_10[2] = {start[0],
//			                   start[1]};
//	float check_point_90[2] = {start[0] + jump_step * 0.63f,
//							   start[1] + jump_step * 0.63f};
//
////	float check_point_10[2] = {start[0] + jump_step * 0.1f,
////							   start[1] + jump_step * 0.1f};
////	float check_point_90[2] = {start[0] + jump_step * 0.90f,
////							   start[1] + jump_step * 0.90f};
//
//	float omega_cp[2] = {0.f, 0.f};
//	float omega_cp1[2] = {0.f, 0.f};
//
//	math::Vector<2> time_cp_10(0,0);
//	math::Vector<2> time_cp_90(0,0);
//
//	math::Vector<2> time_cp1_10(0,0);
//	math::Vector<2> time_cp1_90(0,0);
//
//	ac.control[0] = start[0] + jump_step;
//	ac.control[1] = start[1] + jump_step;
//
//	handle = orb_advertise(ORB_ID(ts_actuator_controls_0), &ac);
//	if (handle == nullptr) {
//		errx(1, "advertise failed");
//	}
//	printf("published 2nd cmd");
//	orb_unadvertise(handle);
//
//	int count = 0;
//
//	while(hit_check_point[1] != 2 || hit_check_point1[1] != 2){
//		pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 0.1);
//
//		/* timed out */
//		if (pret == 0) {
//			//warn("esc rads: poll time out %d, %d", pret, errno);
//		}
//
//		/* this is undesirable but not much we can do - might want to flag unhappy status */
//		if (pret < 0) {
//			warn("esc rads: poll error %d, %d", pret, errno);
//		}
//
//		/* read esc_rads and setup pi controller*/
//		if (pret > 0 && (fds[0].revents & POLLIN)) {
//			count++;
//			orb_copy(ORB_ID(esc_rads), esc_rads_sub, &esc_rads_msg);
//		}
//
//		for(int i = 1; i<2; i++){
//			if(esc_rads_msg.rads_filtered[i] >= check_point_10[i] && hit_check_point[i] == 0){
//				time_cp_10(i) = (float) hrt_absolute_time();
//				hit_check_point[i]++;
//				omega_cp[0] = esc_rads_msg.rads_filtered[i];
//			}
//
//			if(esc_rads_msg.rads_raw[i] >= check_point_10[i] && hit_check_point1[i] == 0){
//				time_cp1_10(i) = (float) hrt_absolute_time();
//				hit_check_point1[i]++;
//				omega_cp1[0] = esc_rads_msg.rads_raw[i];
//			}
//
//			if(esc_rads_msg.rads_filtered[i] >= check_point_90[i] && hit_check_point[i] == 1){
//				time_cp_90(i) = (float) hrt_absolute_time();
//				hit_check_point[i]++;
//				omega_cp[1] = esc_rads_msg.rads_filtered[i];
//			}
//
//			if(esc_rads_msg.rads_raw[i] >= check_point_90[i] && hit_check_point1[i] == 1){
//				time_cp1_90(i) = (float) hrt_absolute_time();
//				hit_check_point1[i]++;
//				omega_cp1[1] = esc_rads_msg.rads_raw[i];
//			}
//		}
//	}
//	printf("checked %d\n", count);
//
//	math::Vector<2> rise_time = (time_cp_90 - time_cp_10) / 1e6f;
//	math::Vector<2> rise_time1 = (time_cp1_90 - time_cp1_10) / 1e6f;
//	printf("omega start: %.10f, %.10f\n", (double) start[0], (double) start[1]);
//	printf("omega: %.10f, %.10f\n", (double) omega_cp[0], (double) omega_cp[1]);
//	printf("filtered rise time: %.10f, %.10f\n", (double) rise_time(0), (double) rise_time(1));
//	printf("omega: %.10f, %.10f\n", (double) omega_cp1[0], (double) omega_cp1[1]);
//	printf("raw rise time: %.10f, %.10f\n", (double) rise_time1(0), (double) rise_time1(1));
//
//	rise_time(0) = 0.02;
//	math::Vector<2> xi_recip = rise_time * _mixer_info.k_p;
//	math::Vector<2> des_ki = xi_recip * 8100.f;
//	math::Vector<2> des_kp = xi_recip * 1.4f * 90.f;
//
//	printf("Xi: %,3f, %.3f\n", (double) (1 / xi_recip(0)), (double) (1 / xi_recip(1)));
//	printf("Desired Ki %.6f, %.6f\n", (double) des_ki(0), (double) des_ki(1));
//	printf("Desired kp %.6f, %.6f\n", (double) des_kp(0), (double) des_kp(1));
//
//
//
//
//	usleep(4000000);
//
//	aa.armed = false;
//	aa.prearmed = true;
//	aa.ready_to_arm = true;
//	aa.lockdown = false;
//	aa.manual_lockdown = false;
//	aa.force_failsafe = false;
//	aa.in_esc_calibration_mode = false;
//
//	handle = orb_advertise(ORB_ID(actuator_armed), &aa);
//	if (handle == nullptr) {
//		errx(1, "advertise failed 2");
//	}
//
//	orb_unadvertise(handle);

}

void
TSFMU::update_mixer_params(int argc, char *argv[])
{
	int i = 1;
	char* param_name;
	char* param_value;

	while(i<argc){
		param_name = argv[i];
		param_value = argv[i+1];

		if(!strcmp(param_name, "kp")){
			_mixer_info.k_p = strtof(param_value, 0);
			i += 2;
			continue;
		}

		if(!strcmp(param_name, "ki")){
			_mixer_info.k_i = strtof(param_value, 0);
			i += 2;
			continue;
		}

		if(!strcmp(param_name, "ilim")){
			_mixer_info.int_term_lim = strtof(param_value, 0);
			i += 2;
			continue;
		}

		if(!strcmp(param_name, "plim")){
			_mixer_info.p_term_lim = strtof(param_value, 0);
			i += 2;
			continue;
		}

		if(!strcmp(param_name, "intlim")){
			_mixer_info.integral_lim = strtof(param_value, 0);
			i += 2;
			continue;
		}
	}

	_ts_mixer->update_mixer_info(&_mixer_info);
}


void
TSFMU::gpio_reset(void)
{
	/*
	 * Setup default GPIO config - all pins as GPIOs, input if
	 * possible otherwise output if possible.
	 */
	for (unsigned i = 0; i < _ngpio; i++) {
		if (_gpio_tab[i].input != 0) {
			px4_arch_configgpio(_gpio_tab[i].input);

		} else if (_gpio_tab[i].output != 0) {
			px4_arch_configgpio(_gpio_tab[i].output);
		}
	}

#if defined(GPIO_GPIO_DIR)
	/* if we have a GPIO direction control, set it to zero (input) */
	px4_arch_gpiowrite(GPIO_GPIO_DIR, 0);
	px4_arch_configgpio(GPIO_GPIO_DIR);
#endif
}

void
TSFMU::gpio_set_function(uint32_t gpios, int function)
{
#if defined(BOARD_GPIO_SHARED_BUFFERED_BITS) && defined(GPIO_GPIO_DIR)

	/*
	 * GPIOs 0 and 1 must have the same direction as they are buffered
	 * by a shared 2-port driver.  Any attempt to set either sets both.
	 */
	if (gpios & BOARD_GPIO_SHARED_BUFFERED_BITS) {
		gpios |= BOARD_GPIO_SHARED_BUFFERED_BITS;

		/* flip the buffer to output mode if required */
		if (GPIO_SET_OUTPUT == function ||
		    GPIO_SET_OUTPUT_LOW == function ||
		    GPIO_SET_OUTPUT_HIGH == function) {
			px4_arch_gpiowrite(GPIO_GPIO_DIR, 1);
		}
	}

#endif

	/* configure selected GPIOs as required */
	for (unsigned i = 0; i < _ngpio; i++) {
		if (gpios & (1 << i)) {
			switch (function) {
			case GPIO_SET_INPUT:
				px4_arch_configgpio(_gpio_tab[i].input);
				break;

			case GPIO_SET_OUTPUT:
				px4_arch_configgpio(_gpio_tab[i].output);
				break;

			case GPIO_SET_OUTPUT_LOW:
				px4_arch_configgpio((_gpio_tab[i].output & ~(GPIO_OUTPUT_SET)) | GPIO_OUTPUT_CLEAR);
				break;

			case GPIO_SET_OUTPUT_HIGH:
				px4_arch_configgpio((_gpio_tab[i].output & ~(GPIO_OUTPUT_CLEAR)) | GPIO_OUTPUT_SET);
				break;

			case GPIO_SET_ALT_1:
				if (_gpio_tab[i].alt != 0) {
					px4_arch_configgpio(_gpio_tab[i].alt);
				}

				break;
			}
		}
	}

#if defined(BOARD_GPIO_SHARED_BUFFERED_BITS) && defined(GPIO_GPIO_DIR)

	/* flip buffer to input mode if required */
	if ((GPIO_SET_INPUT == function) && (gpios & BOARD_GPIO_SHARED_BUFFERED_BITS)) {
		px4_arch_gpiowrite(GPIO_GPIO_DIR, 0);
	}

#endif
}

void
TSFMU::gpio_write(uint32_t gpios, int function)
{
	int value = (function == GPIO_SET) ? 1 : 0;

	for (unsigned i = 0; i < _ngpio; i++)
		if (gpios & (1 << i)) {
			px4_arch_gpiowrite(_gpio_tab[i].output, value);
		}
}

uint32_t
TSFMU::gpio_read(void)
{
	uint32_t bits = 0;

	for (unsigned i = 0; i < _ngpio; i++)
		if (px4_arch_gpioread(_gpio_tab[i].input)) {
			bits |= (1 << i);
		}

	return bits;
}

int
TSFMU::capture_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int	ret = -EINVAL;

	lock();

	input_capture_config_t *pconfig = 0;

	input_capture_stats_t *stats = (input_capture_stats_t *)arg;

	pconfig = (input_capture_config_t *)arg;

	switch (cmd) {

	case INPUT_CAP_SET:
		if (pconfig) {
			ret =  up_input_capture_set(pconfig->channel, pconfig->edge, pconfig->filter,
						    pconfig->callback, pconfig->context);
		}

		break;

	case INPUT_CAP_SET_CALLBACK:
		if (pconfig) {
			ret =  up_input_capture_set_callback(pconfig->channel, pconfig->callback, pconfig->context);
		}

		break;

	case INPUT_CAP_GET_CALLBACK:
		if (pconfig) {
			ret =  up_input_capture_get_callback(pconfig->channel, &pconfig->callback, &pconfig->context);
		}

		break;

	case INPUT_CAP_GET_STATS:
		if (arg) {
			ret =  up_input_capture_get_stats(stats->chan_in_edges_out, stats, false);
		}

		break;

	case INPUT_CAP_GET_CLR_STATS:
		if (arg) {
			ret =  up_input_capture_get_stats(stats->chan_in_edges_out, stats, true);
		}

		break;

	case INPUT_CAP_SET_EDGE:
		if (pconfig) {
			ret =  up_input_capture_set_trigger(pconfig->channel, pconfig->edge);
		}

		break;

	case INPUT_CAP_GET_EDGE:
		if (pconfig) {
			ret =  up_input_capture_get_trigger(pconfig->channel, &pconfig->edge);
		}

		break;

	case INPUT_CAP_SET_FILTER:
		if (pconfig) {
			ret =  up_input_capture_set_filter(pconfig->channel, pconfig->filter);
		}

		break;

	case INPUT_CAP_GET_FILTER:
		if (pconfig) {
			ret =  up_input_capture_get_filter(pconfig->channel, &pconfig->filter);
		}

		break;

	case INPUT_CAP_GET_COUNT:
		ret = OK;
		*(unsigned *)arg = 2;


		break;

	case INPUT_CAP_SET_COUNT:
		ret = OK;
		break;

	default:
		ret = -ENOTTY;
		break;
	}

	unlock();
	return ret;
}

int
TSFMU::gpio_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int	ret = OK;

	lock();

	switch (cmd) {

	case GPIO_RESET:
		gpio_reset();
		break;

	case GPIO_SENSOR_RAIL_RESET:
		sensor_reset(arg);
		break;

	case GPIO_PERIPHERAL_RAIL_RESET:
		peripheral_reset(arg);
		break;

	case GPIO_SET_OUTPUT:
	case GPIO_SET_OUTPUT_LOW:
	case GPIO_SET_OUTPUT_HIGH:
	case GPIO_SET_INPUT:
	case GPIO_SET_ALT_1:
#ifdef CONFIG_ARCH_BOARD_AEROFC_V1
		ret = -EINVAL;
#else
		gpio_set_function(arg, cmd);
#endif
		break;

	case GPIO_SET_ALT_2:
	case GPIO_SET_ALT_3:
	case GPIO_SET_ALT_4:
		ret = -EINVAL;
		break;

	case GPIO_SET:
	case GPIO_CLEAR:
#ifdef CONFIG_ARCH_BOARD_AEROFC_V1
		ret = -EINVAL;
#else
		gpio_write(arg, cmd);
#endif
		break;

	case GPIO_GET:
#ifdef CONFIG_ARCH_BOARD_AEROFC_V1
		ret = -EINVAL;
#else
		*(uint32_t *)arg = gpio_read();
#endif
		break;

	default:
		ret = -ENOTTY;
	}

	unlock();

	return ret;
}

/*
  this implements PWM output via a write() method, for compatibility
  with px4io
 */
ssize_t
TSFMU::write(file *filp, const char *buffer, size_t len)
{
	unsigned count = len / 2;
	uint16_t values[8];

#if BOARD_HAS_PWM == 0
	return 0;
#endif

	if (count > BOARD_HAS_PWM) {
		// we have at most BOARD_HAS_PWM outputs
		count = BOARD_HAS_PWM;
	}

	// allow for misaligned values
	memcpy(values, buffer, count * 2);

	for (uint8_t i = 0; i < count; i++) {
		if (values[i] != PWM_IGNORE_THIS_CHANNEL) {
			up_pwm_servo_set(i, values[i]);
		}
	}

	return count * 2;
}

int
TSFMU::cancel_hp_work()
{
	work_cancel(HPWORK, &_work);
	return 0;
}

int
TSFMU::start_hp_work()
{
	work_queue(HPWORK, &_work, (worker_t)&TSFMU::cycle_trampoline, this, 0);
	return 0;
}

void
TSFMU::perf_counter_reset()
{
	perf_reset(_capture_cb);

	for (unsigned i = 0; i < 5; i ++){
		ridi_timediffs[i] = 0;
	}
	ridi_idx = 0;
}

void
TSFMU::print_info()
{
	printf("TSFMU INFO:\n");

	printf("PWM_LIMIT: ");
	switch(_pwm_limit.state){
	case PWM_LIMIT_STATE_INIT:
		printf("INIT");
		break;

	case PWM_LIMIT_STATE_OFF:
		printf("OFF");
		break;

	case PWM_LIMIT_STATE_RAMP:
		printf("RAMP");
		break;

	case PWM_LIMIT_STATE_ON:
		printf("ON");
		break;

	default:
		printf("ERROR");
		break;
	}
	printf("\n");

	printf("_throttle_armed: ");
	if (_throttle_armed) printf("TRUE\n");
	else printf("FALSE\n");

	printf("_armed: ");
		if (_armed.armed) printf("TRUE\n");
		else printf("FALSE\n");

	printf("_is_calib: ");
		if (_is_calib) printf("TRUE\n");
		else printf("FALSE\n");



	printf("Mixer Output: ");
	for (int i = 0; i < _max_actuators; i++){
		printf("%.2f\t", (double)_outputs[i]);
	}
	printf("\n");

	printf("TS Actuator Controls: ");
	for (int i = 0; i < 4; i++){
		printf("%.2f\t", (double)_ts_controls[0].control[i]);
	}

	printf("\n");
	printf("rads-L: %.2f\n", (double)(_rads_l));
	printf("rads-R: %.2f\n", (double)(_rads_r));

	perf_print_counter(_capture_cb);

	for (unsigned i = 0; i < 5; i++) printf("%llu\t", ridi_timediffs[i]);
	printf("\n");
}

namespace
{

int fmu_new_i2c_speed(unsigned bus, unsigned clock_hz)
{
	return TSFMU::set_i2c_bus_clock(bus, clock_hz);
}

int
tsfmu_start()
{
	int ret = OK;

	if (g_fmu == nullptr) {

		g_fmu = new TSFMU();

		if (g_fmu == nullptr) {
			ret = -ENOMEM;

		} else {
			ret = g_fmu->init();

			if (ret != OK) {
				delete g_fmu;
				g_fmu = nullptr;
			}
		}
	}

	return ret;
}

int
tsfmu_stop(void)
{
	int ret = OK;

	if (g_fmu != nullptr) {

		delete g_fmu;
		g_fmu = nullptr;
	}

	return ret;
}

void
sensor_reset(int ms)
{
	int	 fd;

	fd = open(TSFMU_DEV_PATH, O_RDWR);

	if (fd < 0) {
		errx(1, "open fail");
	}

	if (ioctl(fd, GPIO_SENSOR_RAIL_RESET, ms) < 0) {
		warnx("sensor rail reset failed");
	}

	close(fd);
}

void
peripheral_reset(int ms)
{
	int	 fd;

	fd = open(TSFMU_DEV_PATH, O_RDWR);

	if (fd < 0) {
		errx(1, "open fail");
	}

	if (ioctl(fd, GPIO_PERIPHERAL_RAIL_RESET, ms) < 0) {
		warnx("peripheral rail reset failed");
	}

	close(fd);
}



void
test(void)
{
	if (g_fmu!= NULL){
		g_fmu->cancel_hp_work();
	}

	int	 fd;
	unsigned servo_count = 0;
	unsigned capture_count = 0;
	unsigned pwm_value = 1000;
	int	 direction = 1;
	int	 ret;
	uint32_t rate_limit = 0;
	struct input_capture_t {
		bool valid;
		input_capture_config_t  chan;
	} capture_conf[INPUT_CAPTURE_MAX_CHANNELS];

	fd = open(TSFMU_DEV_PATH, O_RDWR);

	if (fd < 0) {
		errx(1, "open fail");
	}

	if (ioctl(fd, PWM_SERVO_ARM, 0) < 0) { err(1, "servo arm failed"); }

	if (ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count) != 0) {
		err(1, "Unable to get servo count\n");
	}

	if (ioctl(fd, INPUT_CAP_GET_COUNT, (unsigned long)&capture_count) != 0) {
		fprintf(stdout, "Not in a capture mode\n");
	}

	warnx("Testing %u servos and %u input captures", (unsigned)servo_count, capture_count);
	memset(capture_conf, 0, sizeof(capture_conf));

	if (capture_count != 0) {
		for (unsigned i = 0; i < capture_count; i++) {
			// Map to channel number
			capture_conf[i].chan.channel = i + servo_count;

			/* Save handler */
			if (ioctl(fd, INPUT_CAP_GET_CALLBACK, (unsigned long)&capture_conf[i].chan.channel) != 0) {
				err(1, "Unable to get capture callback for chan %u\n", capture_conf[i].chan.channel);

			} else {
				input_capture_config_t conf = capture_conf[i].chan;
				conf.callback = &TSFMU::capture_trampoline;
				conf.context = g_fmu;

				if (ioctl(fd, INPUT_CAP_SET_CALLBACK, (unsigned long)&conf) == 0) {
					capture_conf[i].valid = true;

				} else {
					err(1, "Unable to set capture callback for chan %u\n", capture_conf[i].chan.channel);
				}
			}

		}
	}

	struct pollfd fds;

	fds.fd = 0; /* stdin */

	fds.events = POLLIN;

	//TODO: modify testing servos based on pwm_mask


	warnx("Press CTRL-C or 'c' to abort.");

	for (;;) {
		/* sweep all servos between 1000..2000 */
		servo_position_t servos[servo_count];

		for (unsigned i = 0; i < servo_count ; i++) {
			if (g_fmu->show_pwm_mask() & (1<<i)){
				servos[i] = pwm_value;
			}
		}

		if (direction == 1) {
			// use ioctl interface for one direction
			for (unsigned i = 0; i < servo_count;	i++) {
				if (g_fmu->show_pwm_mask() & (1<<i)){
					if (ioctl(fd, PWM_SERVO_SET(i), servos[i]) < 0) {
						err(1, "servo %u set failed", i);
					}
				}
			}

		} else {
			// and use write interface for the other direction
			ret = write(fd, servos, sizeof(servos));

			if (ret != (int)sizeof(servos)) {
				err(1, "error writing PWM servo data, wrote %u got %d", sizeof(servos), ret);
			}
		}

		if (direction > 0) {
			if (pwm_value < 2000) {
				pwm_value++;

			} else {
				direction = -1;
			}

		} else {
			if (pwm_value > 1000) {
				pwm_value--;

			} else {
				direction = 1;
			}
		}

		/* readback servo values */
		for (unsigned i = 0; i < servo_count; i++) {
			if (g_fmu->show_pwm_mask() & (1<<i)){
				servo_position_t value;

				if (ioctl(fd, PWM_SERVO_GET(i), (unsigned long)&value)) {
					err(1, "error reading PWM servo %d", i);
				}

				if (value != servos[i]) {
					errx(1, "servo %d readback error, got %u expected %u", i, value, servos[i]);
				}
			}
		}

		if (capture_count != 0 && (++rate_limit % 500 == 0)) {
			for (unsigned i = 0; i < capture_count; i++) {
				if (capture_conf[i].valid) {
					input_capture_stats_t stats;
					stats.chan_in_edges_out = capture_conf[i].chan.channel;

					if (ioctl(fd, INPUT_CAP_GET_STATS, (unsigned long)&stats) != 0) {
						err(1, "Unable to get stats for chan %u\n", capture_conf[i].chan.channel);

					} else {
						fprintf(stdout, "FMU: Status chan:%u edges: %d last time:%lld last state:%d overflows:%d lantency:%u\n",
							capture_conf[i].chan.channel,
							stats.chan_in_edges_out,
							stats.last_time,
							stats.last_edge,
							stats.overflows,
							stats.latnecy);
					}
				}
			}

		}

		/* Check if user wants to quit */
		char c;
		ret = poll(&fds, 1, 0);

		if (ret > 0) {

			read(0, &c, 1);

			if (c == 0x03 || c == 0x63 || c == 'q') {
				warnx("User abort\n");
				break;
			}
		}
	}

	if (capture_count != 0) {
		for (unsigned i = 0; i < capture_count; i++) {
			// Map to channel number
			if (capture_conf[i].valid) {
				/* Save handler */
				if (ioctl(fd, INPUT_CAP_SET_CALLBACK, (unsigned long)&capture_conf[i].chan) != 0) {
					err(1, "Unable to set capture callback for chan %u\n", capture_conf[i].chan.channel);
				}
			}
		}
	}

	close(fd);

	if (g_fmu!= NULL){
		g_fmu->start_hp_work();
	}
	exit(0);
}

void
fake(int argc, char *argv[])
{
	if (argc < 5) {
		errx(1, "tsfmu fake <RADS_LEFT> <RADS_RIGHT> <ELV_LEFT_DEG> <ELEVON_RIGHT_DEG> (values -100 .. 100)");
	}

	ts_actuator_controls_s ac;

	ac.control[0] = strtof(argv[1], 0);

	ac.control[1] = strtof(argv[2], 0);

	ac.control[2] = strtof(argv[3], 0);

	ac.control[3] = strtof(argv[4], 0);

	orb_advert_t handle = orb_advertise(ORB_ID(ts_actuator_controls_0), &ac);

	if (handle == nullptr) {
		errx(1, "advertise failed");
	}

	orb_unadvertise(handle);

	actuator_armed_s aa;

	aa.armed = true;
	aa.prearmed = true;
	aa.ready_to_arm = true;
	aa.lockdown = false;
	aa.manual_lockdown = false;
	aa.force_failsafe = false;
	aa.in_esc_calibration_mode = false;

	handle = orb_advertise(ORB_ID(actuator_armed), &aa);

	if (handle == nullptr) {
		errx(1, "advertise failed 2");
	}

	orb_unadvertise(handle);

	exit(0);
}

}//namespace

extern "C" __EXPORT int tsfmu_main(int argc, char *argv[]);

int
tsfmu_main(int argc, char *argv[])
{
	const char *verb = argv[1];

	/* does not operate on a FMU instance */
	if (!strcmp(verb, "i2c")) {
		if (argc > 3) {
			int bus = strtol(argv[2], 0, 0);
			int clock_hz = strtol(argv[3], 0, 0);
			int ret = fmu_new_i2c_speed(bus, clock_hz);

			if (ret) {
				errx(ret, "setting I2C clock failed");
			}

			exit(0);

		} else {
			warnx("i2c cmd args: <bus id> <clock Hz>");
		}
	}

	if (!strcmp(verb, "start")) {
		int ret = tsfmu_start();
		if (ret != OK) {
			errx(1, "failed to start TSFMU. Err: %d", ret);
		}
		exit(0);
	}

	if (!strcmp(verb, "stop")) {
		tsfmu_stop();
		errx(0, "TSFMU driver stopped");
	}

	if (!strcmp(verb, "id")) {
		uint8_t id[12];
		(void)get_board_serial(id);

		errx(0, "Board serial:\n %02X%02X%02X%02X %02X%02X%02X%02X %02X%02X%02X%02X",
		     (unsigned)id[0], (unsigned)id[1], (unsigned)id[2], (unsigned)id[3], (unsigned)id[4], (unsigned)id[5],
		     (unsigned)id[6], (unsigned)id[7], (unsigned)id[8], (unsigned)id[9], (unsigned)id[10], (unsigned)id[11]);
	}


	if (!strcmp(verb, "stop-rads")) {
		if (g_fmu != NULL) g_fmu->radsmeter_stop();
		exit(0);
	}

	if (!strcmp(verb, "test")) {
		test();
	}

	if (!strcmp(verb, "calibrate")){
		if(g_fmu != NULL) g_fmu->calibrate(argc -1, argv + 1);
		exit(0);
	}

	if (!strcmp(verb, "perf-reset")) {
		if (g_fmu!= NULL)
		{
			g_fmu->perf_counter_reset();

		}

		exit(0);
	}

	if (!strcmp(verb, "info")) {
		if (g_fmu!= NULL)
		{
			printf("TSFMU Object created.\n");
			g_fmu->print_info();

		}
		else
			printf("TSFMU Object not exist.\n");
		return 0;
	}

	if (!strcmp(verb, "fake")) {
		fake(argc - 1, argv + 1);
	}

	if (!strcmp(verb, "update_params")) {
		if (g_fmu!= NULL){
			g_fmu->update_mixer_params(argc - 1, argv + 1);
		}
		exit(0);
	}

	if (!strcmp(verb, "sensor_reset")) {
		if (argc > 2) {
			int reset_time = strtol(argv[2], 0, 0);
			sensor_reset(reset_time);

		} else {
			sensor_reset(0);
			warnx("resettet default time");
		}

		exit(0);
	}

	if (!strcmp(verb, "peripheral_reset")) {
		if (argc > 2) {
			int reset_time = strtol(argv[2], 0, 0);
			peripheral_reset(reset_time);

		} else {
			peripheral_reset(0);
			warnx("resettet default time");
		}

		exit(0);
	}

	if (!strcmp(verb, "cancel_hpwork")) {
		if (g_fmu!= NULL){
			g_fmu->cancel_hp_work();
		}

		exit(0);
	}

	if (!strcmp(verb, "start_hpwork")) {
		if (g_fmu!= NULL){
			g_fmu->start_hp_work();
		}

		exit(0);
	}



	fprintf(stderr, "FMU: unrecognized command %s, try:\n", verb);
	fprintf(stderr, "  mode_gpio, mode_pwm, mode_pwm4, test\n");
	fprintf(stderr, "\n");
	exit(1);
}

