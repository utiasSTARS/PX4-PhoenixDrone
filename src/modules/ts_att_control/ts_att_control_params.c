/**
 * @file ts_att_control_params.c
 * Parameters for multicopter attitude controller.
 *
 * @author Xintong Du <xintong.du@mail.utoronto.ca>
 */

/**
 * TS Roll P Gain
 *
 * Attitude PID (ROll)
 *
 * @min 0.0
 * @max 300.0
 * @decimal 2
 * @increment 0.05
 * @group Tailsitter Attitude Control
 */

PARAM_DEFINE_FLOAT(TS_ROLL_P, 3.f);
/**
 * TS Roll I Gain
 *
 * Attitude PID (ROll)
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.05
 * @group Tailsitter Attitude Control
 */
PARAM_DEFINE_FLOAT(TS_ROLL_I, 0f);
/**
 * TS Roll D Gain
 *
 * Attitude PID (ROll)
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.05
 * @group Tailsitter Attitude Control
 */
PARAM_DEFINE_FLOAT(TS_ROLL_D, 0f);
/**
 * TS Roll Integral Limit
 *
 * Attitude PID (ROll)
 *
 * @min 0.0
 * @max 1000000.0
 * @decimal 2
 * @increment 0.05
 * @group Tailsitter Attitude Control
 */
PARAM_DEFINE_FLOAT(TS_ROLL_INT_LIM, 5000f);
/**
 * TS Roll Time Constant
 *
 * Attitude PID (ROll)
 *
 * @min 0.0
 * @max 100.0
 * @decimal 5
 * @increment 0.01
 * @group Tailsitter Attitude Control
 */
PARAM_DEFINE_FLOAT(TS_ROLL_TC, 0.2f);
/**
 * TS Roll Rate Time Constant
 *
 * Attitude Rate PID (ROll)
 *
 * @min 0.0
 * @max 100.0
 * @decimal 5
 * @increment 0.01
 * @group Tailsitter Attitude Control
 */
PARAM_DEFINE_FLOAT(TS_ROLL_RATE_TC, 0.06f);
/**
 * TS Roll Maximum (Deg)
 *
 * Attitude PID (ROll)
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.5
 * @group Tailsitter Attitude Control
 */
PARAM_DEFINE_FLOAT(TS_ROLL_MAX, 60.f);
/**
 * TS Roll Rate Maximum (Rads)
 *
 * Attitude PID (ROll)
 *
 * @min 0.0
 * @max 200.0
 * @decimal 2
 * @increment 0.5
 * @group Tailsitter Attitude Control
 */
PARAM_DEFINE_FLOAT(TS_ROLL_RATE_MAX, 100.f);

/**
 * TS Pitch P Gain
 *
 * Attitude PID (Pitch)
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.01
 * @group Tailsitter Attitude Control
 */
PARAM_DEFINE_FLOAT(TS_PITCH_P, 8.f);
/**
 * TS Pitch I Gain
 *
 * Attitude PID (Pitch)
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.01
 * @group Tailsitter Attitude Control
 */
PARAM_DEFINE_FLOAT(TS_PITCH_I, 0f);
/**
 * TS Pitch D Gain
 *
 * Attitude PID (Pitch)
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.01
 * @group Tailsitter Attitude Control
 */
PARAM_DEFINE_FLOAT(TS_PITCH_D, 0f);
/**
 * TS Pitch Integral Limit
 *
 * Attitude PID (Pitch)
 *
 * @min 0.0
 * @max 1000000.0
 * @decimal 2
 * @increment 0.5
 * @group Tailsitter Attitude Control
 */
PARAM_DEFINE_FLOAT(TS_PITCH_INT_LIM, 5000f);
/**
 * TS Pitch Time Constant
 *
 * Attitude PID (Pitch)
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.01
 * @group Tailsitter Attitude Control
 */
PARAM_DEFINE_FLOAT(TS_PITCH_TC, 0.2f);
/**
 * TS Pitch Rate Time Constant
 *
 * Attitude Rate PID (Pitch)
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.01
 * @group Tailsitter Attitude Control
 */
PARAM_DEFINE_FLOAT(TS_PITCH_RATE_TC, 0.04f);
/**
 * TS Pitch Maximum (Deg)
 *
 * Attitude PID (Pitch)
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.5
 * @group Tailsitter Attitude Control
 */
PARAM_DEFINE_FLOAT(TS_PITCH_MAX, 60.f);
/**
 * TS Pitch Rate Maximum (Rads)
 *
 * Attitude PID (Pitch)
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.1
 * @group Tailsitter Attitude Control
 */
PARAM_DEFINE_FLOAT(TS_PITCH_RATE_MX, 40.f);
/**
 * TS Yaw P Gain
 *
 * Attitude PID (Yaw)
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.01
 * @group Tailsitter Attitude Control
 */
PARAM_DEFINE_FLOAT(TS_YAW_P, 3f);
/**
 * TS Yaw I Gain
 *
 * Attitude PID (Yaw)
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.01
 * @group Tailsitter Attitude Control
 */
PARAM_DEFINE_FLOAT(TS_YAW_I, 0f);
/**
 * TS Yaw D Gain
 *
 * Attitude PID (Yaw)
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.01
 * @group Tailsitter Attitude Control
 */
PARAM_DEFINE_FLOAT(TS_YAW_D, 0f);
/**
 * TS Yaw Int Limit
 *
 * Attitude PID (Yaw)
 *
 * @min 0.0
 * @max 100000.0
 * @decimal 2
 * @increment 1
 * @group Tailsitter Attitude Control
 */
PARAM_DEFINE_FLOAT(TS_YAW_INT_LIM, 5000f);
/**
 * TS Yaw Time Constant
 *
 * Attitude Rate PID (Yaw)
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.01
 * @group Tailsitter Attitude Control
 */
PARAM_DEFINE_FLOAT(TS_YAW_TC, 0.2f);
/**
 * TS Yaw Rate Time Constant
 *
 * Attitude Rate PID (Yaw)
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.01
 * @group Tailsitter Attitude Control
 */
PARAM_DEFINE_FLOAT(TS_YAW_RATE_TC, 0.5f);
/**
 * TS Yaw Max (Deg)
 *
 * Attitude Rate PID (Yaw)
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.5
 * @group Tailsitter Attitude Control
 */
PARAM_DEFINE_FLOAT(TS_YAW_MAX, 60.f);
/**
 * TS Yaw Rate Max (Rads)
 *
 * Attitude Rate PID (Yaw)
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.1
 * @group Tailsitter Attitude Control
 */
PARAM_DEFINE_FLOAT(TS_YAW_RATE_MAX, 40.f);
/**
 * TS Moment of Inertia xx
 *
 * TS Moment of Inertia
 *
 * @min 0.0
 * @max 1.0
 * @decimal 7
 * @increment 0.0000001
 * @group Tailsitter Attitude Control
 */
PARAM_DEFINE_FLOAT(TS_J_XX, 0.0458929f);
/**
 * TS Moment of Inertia yy
 *
 * TS Moment of Inertia
 *
 * @min 0.0
 * @max 1.0
 * @decimal 7
 * @increment 0.0000001
 * @group Tailsitter Attitude Control
 */
PARAM_DEFINE_FLOAT(TS_J_YY, 0.147563f);
/**
 * TS Moment of Inertia zz
 *
 * TS Moment of Inertia
 *
 * @min 0.0
 * @max 1.0
 * @decimal 7
 * @increment 0.0000001
 * @group Tailsitter Attitude Control
 */
PARAM_DEFINE_FLOAT(TS_J_ZZ, 0.1977f);
/**
 * TS Drag Constant
 *
 * TS Drag Constant
 *
 * @min 0.0
 * @max 1.0
 * @decimal 8
 * @increment 0.00000001
 * @group Tailsitter Attitude Control
 */
PARAM_DEFINE_FLOAT(TS_DRAG_CONST, 0.00000175f);
/**
 * TS Lift Constant
 *
 * TS Lift Constant
 *
 * @min 0.0
 * @max 1.0
 * @decimal 8
 * @increment 0.00000001
 * @group Tailsitter Attitude Control
 */
PARAM_DEFINE_FLOAT(TS_LIFT_CONST, 0.00000348f);
/**
 * TS Thrust Constant
 *
 * TS Thrust Constant
 *
 * @min 0.0
 * @max 1.0
 * @decimal 8
 * @increment 0.00000001
 * @group Tailsitter Attitude Control
 */
PARAM_DEFINE_FLOAT(TS_THRUST_CONST, 0.000007864f);
/**
 * TS Momentum Constant - thurst torque arm
 *
 * TS Momentum Constant
 *
 * @min 0.0
 * @max 100.0
 * @decimal 3
 * @increment 0.01
 * @group Tailsitter Attitude Control
 */
PARAM_DEFINE_FLOAT(TS_MOMT_CONST_R, 0.3f);
/**
 * TS Momentum Constant - pitch
 *
 * TS Momentum Constant
 *
 * @min -1.0
 * @max 1.0
 * @decimal 9
 * @increment 0.000000001
 * @group Tailsitter Attitude Control
 */
PARAM_DEFINE_FLOAT(TS_MOMT_CONST_P, 0.000000344f);
/**
 * TS Momentum Constant - motor constant
 *
 * TS Momentum Constant
 *
 * @min -1.0
 * @max 1.0
 * @decimal 4
 * @increment 0.001
 * @group Tailsitter Attitude Control
 */
PARAM_DEFINE_FLOAT(TS_MOMT_CONST_Y, 0.023f);
/**
 * TS Motor RADs max (scaled by 800)
 *
 * TS Momentum Constant
 *
 * @min 0.0
 * @max 1.0
 * @decimal 4
 * @increment 0.001
 * @group Tailsitter Attitude Control
 */
PARAM_DEFINE_FLOAT(TS_MOT_MAX, 1.0f);
/**
 * TS Servo max (deg)
 *
 * TS Momentum Constant
 *
 * @min 0.0
 * @max 70.0
 * @decimal 1
 * @increment 0.1
 * @group Tailsitter Attitude Control
 */
PARAM_DEFINE_FLOAT(TS_SERVO_MAX, 60.0f);


/**
 * TPA P Breakpoint
 *
 * Throttle PID Attenuation (TPA)
 * Magnitude of throttle setpoint at which to begin attenuating roll/pitch I gain
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.1
 * @group Tailsitter Attitude Control
 */
PARAM_DEFINE_FLOAT(TS_TPA_BREAK_P, 1.0f);

/**
 * TPA I Breakpoint
 *
 * Throttle PID Attenuation (TPA)
 * Magnitude of throttle setpoint at which to begin attenuating roll/pitch I gain
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.1
 * @group Tailsitter Attitude Control
 */
PARAM_DEFINE_FLOAT(TS_TPA_BREAK_I, 1.0f);

/**
 * TPA D Breakpoint
 *
 * Throttle PID Attenuation (TPA)
 * Magnitude of throttle setpoint at which to begin attenuating roll/pitch D gain
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.1
 * @group Tailsitter Attitude Control
 */
PARAM_DEFINE_FLOAT(TS_TPA_BREAK_D, 1.0f);

/**
 * TPA Rate P
 *
 * Throttle PID Attenuation (TPA)
 * Rate at which to attenuate roll/pitch P gain
 * Attenuation factor is 1.0 when throttle magnitude is below the setpoint
 * Above the setpoint, the attenuation factor is (1 - rate * (throttle - breakpoint) / (1.0 - breakpoint))
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Tailsitter Attitude Control
 */
PARAM_DEFINE_FLOAT(TS_TPA_RATE_P, 0.0f);

/**
 * TPA Rate I
 *
 * Throttle PID Attenuation (TPA)
 * Rate at which to attenuate roll/pitch I gain
 * Attenuation factor is 1.0 when throttle magnitude is below the setpoint
 * Above the setpoint, the attenuation factor is (1 - rate * (throttle - breakpoint) / (1.0 - breakpoint))
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Tailsitter Attitude Control
 */
PARAM_DEFINE_FLOAT(TS_TPA_RATE_I, 0.0f);

/**
 * TPA Rate D
 *
 * Throttle PID Attenuation (TPA)
 * Rate at which to attenuate roll/pitch D gain
 * Attenuation factor is 1.0 when throttle magnitude is below the setpoint
 * Above the setpoint, the attenuation factor is (1 - rate * (throttle - breakpoint) / (1.0 - breakpoint))
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Tailsitter Attitude Control
 */
PARAM_DEFINE_FLOAT(TS_TPA_RATE_D, 0.0f);
