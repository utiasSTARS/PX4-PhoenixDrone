#include "../BlockLocalPositionEstimator.hpp"
#include <systemlib/mavlink_log.h>
#include <matrix/math.hpp>

extern orb_advert_t mavlink_log_pub;

// required number of samples for sensor
// to initialize
static const uint32_t 		REQ_MOCAP_INIT_COUNT = 20;
static const uint32_t 		MOCAP_TIMEOUT =     200000;	// 0.2 s

void BlockLocalPositionEstimator::mocapInit()
{
	// measure
	Vector<float, n_y_mocap> y;
	Vector<float, n_y_mocap> v; v.setZero();

	if (mocapMeasure(y, v) != OK) {
		_mocapStats.reset();
		return;
	}

	// if finished
	if (_mocapStats.getCount() > REQ_MOCAP_INIT_COUNT) {
		mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] mocap position init: "
					     "%5.2f, %5.2f, %5.2f m std %5.2f, %5.2f, %5.2f m",
					     double(_mocapStats.getMean()(0)),
					     double(_mocapStats.getMean()(1)),
					     double(_mocapStats.getMean()(2)),
					     double(_mocapStats.getStdDev()(0)),
					     double(_mocapStats.getStdDev()(1)),
					     double(_mocapStats.getStdDev()(2)));
		_sensorTimeout &= ~SENSOR_MOCAP;
		_sensorFault &= ~SENSOR_MOCAP;

		if (!_altOriginInitialized) {
			_altOriginInitialized = true;
			_altOrigin = 0;
		}
	}
}

int BlockLocalPositionEstimator::mocapMeasure(Vector<float, n_y_mocap> &y, Vector<float, n_y_mocap> &v)
{

	y.setZero();
	y(Y_mocap_x) = _sub_mocap.get().x;
	y(Y_mocap_y) = _sub_mocap.get().y;
	y(Y_mocap_z) = _sub_mocap.get().z;
	_mocapStats.update(y);
	_time_last_mocap = _sub_mocap.get().timestamp;
	if (_mocap_old.timestamp != 0) {
		v = (y - _mocap_old.position)/(_sub_mocap.get().timestamp_received - _mocap_old.timestamp)*1e6;
	}
	publishDebugTupple((int8_t*)"mocap_out", v(2));
	_mocap_old.timestamp = _sub_mocap.get().timestamp_received;
	_mocap_old.position = y;
	return OK;
}

void BlockLocalPositionEstimator::mocapCorrect()
{
	// measure
	Vector<float, n_y_mocap> y;
	Vector<float, n_y_mocap> v; v.setZero();

	if (mocapMeasure(y, v) != OK) { return; }

	// mocap measurement matrix, measures position
	Matrix<float, n_y_mocap*2, n_x> C;
	C.setZero();
	C(Y_mocap_x, X_x) = 1;
	C(Y_mocap_y, X_y) = 1;
	C(Y_mocap_z, X_z) = 1;
	C(Y_mocap_x+3, X_vx) = 1;
	C(Y_mocap_y+3, X_vy) = 1;
	C(Y_mocap_z+3, X_vz) = 1;

	// noise matrix
	Matrix<float, n_y_mocap*2, n_y_mocap*2> R;
	R.setZero();
	float mocap_p_var = _mocap_p_stddev.get()* \
			    _mocap_p_stddev.get();
	R(Y_mocap_x, Y_mocap_x) = mocap_p_var;
	R(Y_mocap_y, Y_mocap_y) = mocap_p_var;
	R(Y_mocap_z, Y_mocap_z) = mocap_p_var;
	float mocap_v_var = mocap_p_var*2*200*200;
	R(Y_mocap_x+3, Y_mocap_x+3) = mocap_v_var;
	R(Y_mocap_y+3, Y_mocap_y+3) = mocap_v_var;
	R(Y_mocap_z+3, Y_mocap_z+3) = mocap_v_var;


	// residual
	Matrix<float, n_y_mocap*2, n_y_mocap*2> S_I = inv<float, n_y_mocap*2>((C * _P * C.transpose()) + R);
	Vector<float, n_y_mocap*2> measurement;
	memcpy(measurement._data[0], y._data[0], sizeof(y._data));
	memcpy(measurement._data[3], v._data[0], sizeof(v._data));
	Matrix<float, n_y_mocap*2, 1> r = measurement - C * _x;

	// fault detection
	float beta = (r.transpose() * (S_I * r))(0, 0);

	if (beta > BETA_TABLE[n_y_mocap*2]) {
		if (!(_sensorFault & SENSOR_MOCAP)) {
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] mocap fault, beta %5.2f", double(beta));
			_sensorFault |= SENSOR_MOCAP;
		}

	} else if (_sensorFault & SENSOR_MOCAP) {
		_sensorFault &= ~SENSOR_MOCAP;
		mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] mocap OK");
	}

	// kalman filter correction always
	Matrix<float, n_x, n_y_mocap*2> K = _P * C.transpose() * S_I;
	Vector<float, n_x> dx = K * r;
	_x += dx;
	_P -= K * C * _P;
}

void BlockLocalPositionEstimator::mocapCheckTimeout()
{
	if (_timeStamp - _time_last_mocap > MOCAP_TIMEOUT) {
		if (!(_sensorTimeout & SENSOR_MOCAP)) {
			_sensorTimeout |= SENSOR_MOCAP;
			_mocapStats.reset();
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] mocap timeout ");
		}
	}
}
