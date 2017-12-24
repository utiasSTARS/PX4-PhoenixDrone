/**
 * @file ts_path_planner_params.c
 * Parameters for tailsitter path planning.
 *
 * @author Xintong Du <xintong.du@mail.utoronto.ca>
 */

/**
 * TS Cruise Speed Z
 *
 * TS path planner
 *
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group Tailsitter Path Planner
 */

PARAM_DEFINE_FLOAT(TS_CRUISE_MAX_Z, 1.f);

/**
 * TS Cruise Speed XY
 *
 * TS path planner
 *
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group Tailsitter Path Planner
 */

PARAM_DEFINE_FLOAT(TS_CRUISE_MAX_XY, 2.f);

/**
 * TS Cruise Speed XY
 *
 * TS path planner
 *
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group Tailsitter Path Planner
 */

PARAM_DEFINE_FLOAT(TS_CRUISE_SPEED, 2.f);
