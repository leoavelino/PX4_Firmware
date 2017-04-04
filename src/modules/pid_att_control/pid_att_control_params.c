/**
 * @file pid_att_control_params.c
 * Parameters for pid attitude controller.
 *
 * @author Leonardo Avelino
 */

/**
 * Roll P gain
 *
 * @unit
 * @min 0.00
 * @max 10.00
 * @decimal 3
 * @increment 0.001
 * @group Pid Attitude Control
 */
PARAM_DEFINE_FLOAT(PID_ROLL_P, 1.0f);

/**
 * Pitch P gain
 *
 * @unit
 * @min 0.00
 * @max 10.00
 * @decimal 2
 * @increment 0.001
 * @group Pid Attitude Control
 */
PARAM_DEFINE_FLOAT(PID_PITCH_P, 1.0f);

/**
 * Yaw P gain
 *
 * @unit
 * @min 0.00
 * @max 10.00
 * @decimal 3
 * @increment 0.001
 * @group Pid Attitude Control
 */
PARAM_DEFINE_FLOAT(PID_YAW_P, 0.01f);

/**
 * Roll I gain
 *
 * @unit
 * @min 0.00
 * @max 10.00
 * @decimal 2
 * @increment 0.001
 * @group Pid Attitude Control
 */
PARAM_DEFINE_FLOAT(PID_ROLL_I, 0.001f);

/**
 * Pitch I gain
 *
 * @unit
 * @min 0.00
 * @max 10.00
 * @decimal 3
 * @increment 0.001
 * @group Pid Attitude Control
 */
PARAM_DEFINE_FLOAT(PID_PITCH_I, 0.001f);

/**
 * Yaw I gain
 *
 * @unit
 * @min 0.00
 * @max 10.00
 * @decimal 3
 * @increment 0.001
 * @group Pid Attitude Control
 */
PARAM_DEFINE_FLOAT(PID_YAW_I, 0.001f);

/**
 * Roll D gain
 *
 * @unit
 * @min 0.00
 * @max 10.00
 * @decimal 2
 * @increment 0.001
 * @group Pid Attitude Control
 */
PARAM_DEFINE_FLOAT(PID_ROLL_D, 0.1f);

/**
 * Pitch D gain
 *
 * @unit
 * @min 0.00
 * @max 10.00
 * @decimal 3
 * @increment 0.001
 * @group Pid Attitude Control
 */
PARAM_DEFINE_FLOAT(PID_PITCH_D, 0.1f);

/**
 * Yaw D gain
 *
 * @unit
 * @min 0.00
 * @max 10.00
 * @decimal 3
 * @increment 0.001
 * @group Pid Attitude Control
 */
PARAM_DEFINE_FLOAT(PID_YAW_D, 0.01f);

/**
 * Maximum integral term for roll
 *
 * @unit
 * @min 0.00
 * @max 10.00
 * @decimal 3
 * @increment 0.001
 * @group Pid Attitude Control
 */
PARAM_DEFINE_FLOAT(PID_MAX_ROLL_I, 0.3f);

/**
 * Maximum integral term for pitch
 *
 * @unit
 * @min 0.00
 * @max 10.00
 * @decimal 3
 * @increment 0.001
 * @group Pid Attitude Control
 */
PARAM_DEFINE_FLOAT(PID_MAX_PITCH_I, 0.3f);


/**
 * Maximum integral term for yaw
 *
 * @unit
 * @min 0.00
 * @max 10.00
 * @decimal 3
 * @increment 0.001
 * @group Pid Attitude Control
 */
PARAM_DEFINE_FLOAT(PID_MAX_YAW_I, 0.3f);
