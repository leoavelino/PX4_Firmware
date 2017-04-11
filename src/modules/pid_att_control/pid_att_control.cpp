/**
 * @file pid_att_control.cpp
 *
 * PID Attitude Control
 *
 *
 * Created by Leonardo Avelino
 **/

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <lib/mathlib/mathlib.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <systemlib/perf_counter.h>
#include <systemlib/param/param.h>


/* Definitions ******************/

#define MIN_TAKEOFF_THRUST      0.2f

#define AXIS_INDEX_ROLL         0
#define AXIS_INDEX_PITCH        1
#define AXIS_INDEX_YAW          2

#define PI                      3.1415f
#define DEG_TO_RAD_180          PI
#define DEG_TO_RAD_360          2*PI

/********************************/

extern "C" __EXPORT int pid_att_control_main(int argc, char *argv[]);

struct v_angles {
    float roll;
    float pitch;
    float yaw;
}v_angles;

class Pid_Att_Control{
public:

    /**
     * Constructor
     */
    Pid_Att_Control();

    /**
     * Destructor, also kills the main task
     */
    ~Pid_Att_Control();

    /**
     * Start the PID Att control task
     * @return OK on success
     */
    int start();

private:

    //int max_loop; //## DEBUG

    bool _task_should_exit;                                         // if true, task_main() should exit;
    int _control_task;                                              // task handle

    int _v_attitude_sp_sub;                                         // vehicle attitude setpoint subscription
    int _v_attitude_sub;                                            // vehicle attitude subscription
    int _motor_limits_sub;		                                    // motor limits subscription
    int _v_control_mode_sub;	                                    // vehicle control mode subscription
    int _armed_sub;                                                 // actuator armed subscription
    int _params_sub;                                                // parameter updates subscription

    orb_advert_t	_actuators_0_pub;                               // attitude actuator controls publication

    struct vehicle_attitude_setpoint_s	_v_attitude_sp;             // vehicle attitude setpoint
    struct vehicle_attitude_s _attitude;                            // vehicle attitude
    struct multirotor_motor_limits_s	_motor_limits;		        // motor limits
    struct vehicle_control_mode_s _v_control_mode;                  // vehicle control mode
    struct actuator_controls_s _actuators;			                // actuator controls
    struct actuator_armed_s _armed;                                 // arming status
    struct v_angles _angles;                                        // current roll, pitch, yaw
    struct v_angles _angles_prev_error;                             // roll, pitch, yaw previous error
    struct v_angles _angles_sp;                                     // current angles setpoint
    struct v_angles _angles_int;                                    // integral accumulated error
    struct v_angles _att_control;                                   // torques for control
    struct v_angles _angles_p, _angles_i, _angles_d;                // p, i, d terms of equation

    union {
        struct {
            uint16_t motor_pos	: 1; // 0 - true when any motor has saturated in the positive direction
            uint16_t motor_neg	: 1; // 1 - true when any motor has saturated in the negative direction
            uint16_t roll_pos	: 1; // 2 - true when a positive roll demand change will increase saturation
            uint16_t roll_neg	: 1; // 3 - true when a negative roll demand change will increase saturation
            uint16_t pitch_pos	: 1; // 4 - true when a positive pitch demand change will increase saturation
            uint16_t pitch_neg	: 1; // 5 - true when a negative pitch demand change will increase saturation
            uint16_t yaw_pos	: 1; // 6 - true when a positive yaw demand change will increase saturation
            uint16_t yaw_neg	: 1; // 7 - true when a negative yaw demand change will increase saturation
            uint16_t thrust_pos	: 1; // 8 - true when a positive thrust demand change will increase saturation
            uint16_t thrust_neg	: 1; // 9 - true when a negative thrust demand change will increase saturation
        } flags;
        uint16_t value;
    } _saturation_status;

    struct {
        param_t roll_p;
        param_t pitch_p;
        param_t yaw_p;
        param_t roll_i;
        param_t pitch_i;
        param_t yaw_i;
        param_t roll_d;
        param_t pitch_d;
        param_t yaw_d;
        param_t max_roll_i;
        param_t max_pitch_i;
        param_t max_yaw_i;
    } _params_handle;                                   // handle for important parameters

    struct {
        float roll_p, roll_i, roll_d;                   // PID gains for roll
        float pitch_p, pitch_i, pitch_d;                // PID gains for pitch
        float yaw_p, yaw_i, yaw_d;                      // PID gains for yaw
        float max_roll_i, max_pitch_i, max_yaw_i;       // Maximum integral term for roll, pitch and yaw
    } _params;

    perf_counter_t	_loop_perf;			/** loop performance counter */

    /**
     * Shim for calling task_main from task_create.
     */
    static void	task_main_trampoline(int argc, char *argv[]);

    /**
     *  Check for parameters update
     */
    void parameter_update_poll();

    /**
     *  Update parameters
     */
    void parameters_update();

    /**
     * Convert Attitude quaternion to euler angles
     */
    void att_quaternion_to_euler(const struct vehicle_attitude_s *att, struct v_angles *angles);

    /**
     *  Check the arming status of the system
     */
    void arming_status_poll();

    /**
     * Poll angle setpoint from the position controller algorithm (or manual control)
     */
    void vehicle_attitude_setpoint_poll();

    /**
      * Poll vehicle attitude (roll, pitch, yaw)
      */
    void vehicle_attitude_poll();

    /**
      * Poll vehicle motor limits
      */
    void vehicle_motor_limits_poll();

    /**
      * Check for changes in vehicle control mode
      */
    void vehicle_control_mode_poll();

    /**
      * Attitude controller;
      */
    void control_attitude(float dt);

    /**
     * Main attitude control task.
     */
    void task_main();
};

namespace pid_att_control
{
    Pid_Att_Control *g_control;
}

Pid_Att_Control::Pid_Att_Control() :

    //max_loop(0), //## DEBUG

   _task_should_exit(false),
   _control_task(-1),
   _v_attitude_sp_sub(-1),
   _v_attitude_sub(-1),
   _motor_limits_sub(-1),
   _v_control_mode_sub(-1),
   _params_sub(-1),
   _actuators_0_pub(nullptr),

   _v_attitude_sp{},
   _attitude{},
   _motor_limits{},
   _v_control_mode{},
   _actuators{},
   _angles{},
   _angles_prev_error{},
   _angles_sp{},
   _angles_int{0.0f, 0.0f, 0.0f},
   _att_control{},
   _angles_p{},
   _angles_i{},
   _angles_d{},
   _saturation_status{},
   _params{},
   _loop_perf(perf_alloc(PC_ELAPSED, "pid_att_control"))

{
    _params_handle.roll_p           =   param_find("PID_ROLL_P");
    _params_handle.roll_i           =   param_find("PID_ROLL_I");
    _params_handle.roll_d           =   param_find("PID_ROLL_D");
    _params_handle.pitch_p          =   param_find("PID_PITCH_P");
    _params_handle.pitch_i          =   param_find("PID_PITCH_I");
    _params_handle.pitch_d          =   param_find("PID_PITCH_D");
    _params_handle.yaw_p            =   param_find("PID_YAW_P");
    _params_handle.yaw_i            =   param_find("PID_YAW_I");
    _params_handle.yaw_d            =   param_find("PID_YAW_D");
    _params_handle.max_roll_i       =   param_find("PID_MAX_ROLL_I");
    _params_handle.max_pitch_i      =   param_find("PID_MAX_PITCH_I");
    _params_handle.max_yaw_i        =   param_find("PID_MAX_YAW_I");
}

Pid_Att_Control::~Pid_Att_Control()
{
    if (_control_task != -1) {
        /* task wakes up every 100ms or so at the longest */
        _task_should_exit = true;

        /* wait for a second for the task to quit at our request */
        unsigned i = 0;

        do {
            /* wait 20ms */
            usleep(20000);

            /* if we have given up, kill it */
            if (++i > 50) {
                px4_task_delete(_control_task);
                break;
            }
        } while (_control_task != -1);
    }

    pid_att_control::g_control = nullptr;
}

void Pid_Att_Control::parameters_update()
{
    float tmp;

    param_get(_params_handle.roll_p, &tmp);
    _params.roll_p = tmp;
    param_get(_params_handle.roll_i, &tmp);
    _params.roll_i = tmp;
    param_get(_params_handle.roll_d, &tmp);
    _params.roll_d = tmp;
    param_get(_params_handle.pitch_p, &tmp);
    _params.pitch_p = tmp;
    param_get(_params_handle.pitch_i, &tmp);
    _params.pitch_i = tmp;
    param_get(_params_handle.pitch_d, &tmp);
    _params.pitch_d = tmp;
    param_get(_params_handle.yaw_p, &tmp);
    _params.yaw_p = tmp;
    param_get(_params_handle.yaw_i, &tmp);
    _params.yaw_i = tmp;
    param_get(_params_handle.yaw_d, &tmp);
    _params.yaw_d = tmp;
    param_get(_params_handle.max_roll_i, &tmp);
    _params.max_roll_i = tmp;
    param_get(_params_handle.max_pitch_i, &tmp);
    _params.max_pitch_i = tmp;
    param_get(_params_handle.max_yaw_i, &tmp);
    _params.max_yaw_i = tmp;
}

void Pid_Att_Control::parameter_update_poll()
{
    bool updated;

    /* Check if parameters have changed */
    orb_check(_params_sub, &updated);

    if (updated) {
        struct parameter_update_s param_update;
        orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
        parameters_update();
    }
}

void Pid_Att_Control::arming_status_poll()
{
    /* check if there is a new setpoint */
    bool updated;
    orb_check(_armed_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
    }
}

void Pid_Att_Control::vehicle_attitude_setpoint_poll()
{
    /* check if there is a new setpoint */
    bool updated;
    orb_check(_v_attitude_sp_sub, &updated);

    if(updated) {
        orb_copy(ORB_ID(vehicle_attitude_setpoint), _v_attitude_sp_sub, &_v_attitude_sp);
    }
    _angles_sp.roll = _v_attitude_sp.roll_body;
    _angles_sp.pitch = _v_attitude_sp.pitch_body;
    _angles_sp.yaw = _v_attitude_sp.yaw_body;
}

void Pid_Att_Control::vehicle_attitude_poll()
{
    /* check if there is a new attitude */
    bool updated;
    orb_check(_v_attitude_sub, &updated);

    if(updated) {
        orb_copy(ORB_ID(vehicle_attitude), _v_attitude_sub, &_attitude);
    }
    //quaternion_to_euler(&_attitude, &_angles);
    att_quaternion_to_euler(&_attitude, &_angles);
}

void Pid_Att_Control::vehicle_motor_limits_poll()
{
    /* check if there is a new message */
    bool updated;
    orb_check(_motor_limits_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(multirotor_motor_limits), _motor_limits_sub, &_motor_limits);
        _saturation_status.value = _motor_limits.saturation_status;
    }
}

void Pid_Att_Control::vehicle_control_mode_poll()
{
    bool updated;

    /* Check if vehicle control mode has changed */
    orb_check(_v_control_mode_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_control_mode), _v_control_mode_sub, &_v_control_mode);
    }
}

void Pid_Att_Control::att_quaternion_to_euler(const struct vehicle_attitude_s *att, struct v_angles *angles)
{
    matrix::Eulerf att_euler = matrix::Quatf(att->q);
    angles->roll = att_euler(0);
    angles->pitch = att_euler(1);
    angles->yaw = att_euler(2);
}

void Pid_Att_Control::control_attitude(float dt)
{
    /* poll atittude setpoint data */
    vehicle_attitude_setpoint_poll();
    /* check if there is an update in the attitude topic, and convert data from quaternion to euler */
    vehicle_attitude_poll();

    /* reset integral if disarmed */
    if (!_armed.armed) {
        _angles_int.roll = 0;
        _angles_int.pitch = 0;
        _angles_int.yaw = 0;
    }

    /* calculate error */
    struct v_angles angles_error;
    angles_error.roll = _angles_sp.roll - _angles.roll;
    angles_error.pitch = _angles_sp.pitch - _angles.pitch;
    angles_error.yaw = _angles_sp.yaw - _angles.yaw;

    /* check for the shortest yaw path */
/**
    if (fabsf(_angles_sp.yaw - _angles.yaw) < DEG_TO_RAD_180) {
        angles_error.yaw = _angles_sp.yaw - _angles.yaw;
    }
    else {
        if ((_angles_sp.yaw - _angles.yaw) >  0) {
            angles_error.yaw = _angles_sp.yaw - _angles.yaw - DEG_TO_RAD_360;
        } else
            angles_error.yaw = _angles_sp.yaw - _angles.yaw + DEG_TO_RAD_360;
    }
*/

    /** Proportional control */
    _angles_p.roll = _params.roll_p * angles_error.roll;
    _angles_p.pitch = _params.pitch_p * angles_error.pitch;
    _angles_p.yaw = _params.yaw_p * angles_error.yaw;

    /** Derivative control */
    _angles_d.roll = ((angles_error.roll - _angles_prev_error.roll) / dt) * _params.roll_d;
    _angles_d.pitch = ((angles_error.pitch - _angles_prev_error.pitch) / dt) * _params.pitch_d;
    _angles_d.yaw = ((angles_error.yaw - _angles_prev_error.yaw) / dt) * _params.yaw_d;

    /** Integral control */
    bool positive_saturation = false, negative_saturation = false;

    /*minimum thrust to update integral term*/
    if (_v_attitude_sp.thrust > MIN_TAKEOFF_THRUST)
    {
        for (int i = 0; i < 3; i++)
        {
            if (((i == AXIS_INDEX_ROLL) && (_saturation_status.flags.roll_pos)) ||
                    ((i == AXIS_INDEX_PITCH) && (_saturation_status.flags.pitch_pos))
                    || ((i == AXIS_INDEX_YAW) && (_saturation_status.flags.yaw_pos)))
            {
                positive_saturation = true;
            }
            else if (((i == AXIS_INDEX_ROLL) && (_saturation_status.flags.roll_neg)) ||
                     ((i == AXIS_INDEX_PITCH) && (_saturation_status.flags.pitch_neg))
                    || ((i == AXIS_INDEX_YAW) && (_saturation_status.flags.yaw_neg)))
            {
                negative_saturation = true;
            }

            if (positive_saturation)
            {
                if (i == AXIS_INDEX_ROLL){
                    angles_error.roll = math::min(angles_error.roll, 0.0f);
                }
                else if (i == AXIS_INDEX_PITCH){
                    angles_error.pitch = math::min(angles_error.pitch, 0.0f);
                }
                else if (i == AXIS_INDEX_YAW){
                    angles_error.yaw = math::min(angles_error.yaw, 0.0f);
                }
            }

            if (negative_saturation)
            {
                if (i == AXIS_INDEX_ROLL){
                    angles_error.roll = math::max(angles_error.roll, 0.0f);
                }
                else if (i == AXIS_INDEX_PITCH){
                    angles_error.pitch = math::max(angles_error.pitch, 0.0f);
                }
                else if (i == AXIS_INDEX_YAW){
                    angles_error.yaw = math::max(angles_error.yaw, 0.0f);
                }
            }
        }

        _angles_i.roll = _angles_int.roll + (_params.roll_i * dt * angles_error.roll);
        _angles_i.pitch = _angles_int.pitch + (_params.pitch_i * dt * angles_error.pitch);
        _angles_i.yaw = _angles_int.yaw + (_params.yaw_i * dt * angles_error.yaw);

        /* update integral term only if it is in the allowed range*/
        if(PX4_ISFINITE(_angles_i.roll) && _angles_i.roll > -_params.max_roll_i && _angles_i.roll < _params.max_roll_i)
        {
            _angles_int.roll = _angles_i.roll;
        }
        if(PX4_ISFINITE(_angles_i.pitch) && _angles_i.pitch > -_params.max_pitch_i && _angles_i.pitch < _params.max_pitch_i)
        {
            _angles_int.pitch = _angles_i.pitch;
        }
        if(PX4_ISFINITE(_angles_i.yaw) && _angles_i.yaw > -_params.max_yaw_i && _angles_i.yaw < _params.max_yaw_i)
        {
            _angles_int.yaw = _angles_i.yaw;
        }
    }
    /* force integral term to be in the allowed range */
    _angles_int.roll = math::constrain(_angles_i.roll, -_params.max_roll_i, _params.max_roll_i);
    _angles_int.pitch = math::constrain(_angles_i.pitch, -_params.max_pitch_i, _params.max_pitch_i);
    _angles_int.yaw = math::constrain(_angles_i.yaw, -_params.max_yaw_i, _params.max_yaw_i);

    /* update previous error for derivative term*/
    _angles_prev_error.roll = angles_error.roll;
    _angles_prev_error.pitch = angles_error.pitch;
    _angles_prev_error.yaw = angles_error.yaw;

    /** PID */
    _att_control.roll = _angles_p.roll + _angles_int.roll + _angles_d.roll;
    _att_control.pitch = _angles_p.pitch + _angles_int.pitch + _angles_d.pitch;
    _att_control.yaw = _angles_p.yaw + _angles_int.yaw + _angles_d.yaw;


/*  DEBUG
    max_loop++;
    if (max_loop == 100)
    {
        PX4_INFO("Erro roll: %f", (double)angles_error.roll);
        PX4_INFO("Erro pitch: %f", (double)angles_error.pitch);
        PX4_INFO("Erro Yaw: %f", (double)angles_error.yaw);
        PX4_INFO("Proportional roll: %f", (double)_angles_p.roll);
        PX4_INFO("Proportional pitch: %f", (double)_angles_p.pitch);
        PX4_INFO("Proportional yaw: %f", (double)_angles_p.yaw);
        PX4_INFO("Integral roll: %f", (double)_angles_int.roll);
        PX4_INFO("Integral pitch: %f", (double)_angles_int.pitch);
        PX4_INFO("Integral yaw: %f", (double)_angles_int.yaw);
        PX4_INFO("Derivative roll: %f", (double)_angles_d.roll);
        PX4_INFO("Derivative pitch: %f", (double)_angles_d.pitch);
        PX4_INFO("Derivative yaw: %f", (double)_angles_d.yaw);
        PX4_INFO("Atuador roll: %f", (double)_att_control.roll);
        PX4_INFO("Atuador pitch: %f", (double)_att_control.pitch);
        PX4_INFO("Atuador yaw: %f", (double)_att_control.yaw);
        PX4_INFO("Atuador thrust: %f", (double)_actuators.control[3]);
        max_loop = 0;
    }
*/
}

int Pid_Att_Control::start()
{
    ASSERT(_control_task == -1);
    /* start the task */
    _control_task = px4_task_spawn_cmd("pid_att_control",
                                       SCHED_DEFAULT,
                                       SCHED_PRIORITY_MAX -5,
                                       2048,
                                       (px4_main_t)&task_main_trampoline,
                                       nullptr);
    if (_control_task < 0){
        warn("Task start failed");
        return -errno;
    }
    return OK;
}

void Pid_Att_Control::task_main_trampoline(int argc, char *argv[])
{
   pid_att_control::g_control->task_main();
}

void Pid_Att_Control::task_main()
{
    /* Do subscriptions */
    _v_attitude_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
    _v_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    _motor_limits_sub = orb_subscribe(ORB_ID(multirotor_motor_limits));
    _v_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
    _armed_sub = orb_subscribe(ORB_ID(actuator_armed));
    _params_sub = orb_subscribe(ORB_ID(parameter_update));

    px4_pollfd_struct_t poll_fds = {};
    poll_fds.events = POLLIN;

    while(!_task_should_exit)
    {

        poll_fds.fd = _v_attitude_sub;

        // wait for up to 50ms for data
        int pret = px4_poll(&poll_fds, 1, 100);

        // timed out - periodid check for _task_should_exit
        if (pret == 0) {
            continue;
        }

        /* this is undesirable but not much we can do - might want to flag unhappy status */
        if (pret < 0) {
            warn("pid att ctrl: poll error %d, %d", pret, errno);
            /* sleep a bit before next try */
            usleep(100000);
            continue;
        }

        perf_begin(_loop_perf); //loop performance begin

        if (poll_fds.revents & POLLIN) {
            static uint64_t last_run = 0;
            float dt = (hrt_absolute_time() - last_run) / 1000000.0f;
            last_run = hrt_absolute_time();

            /* guard against too small (< 2ms) and too large (> 20ms) dt's */
            if (dt < 0.002f) {
                dt = 0.002f;
            }
            else if (dt > 0.02f) {
                dt = 0.02f;
            }

            orb_copy(ORB_ID(vehicle_attitude), _v_attitude_sub, &_attitude);

            /* Poll important data */
            parameter_update_poll();
            arming_status_poll();
            vehicle_motor_limits_poll();
            vehicle_control_mode_poll();

            if ((_v_control_mode.flag_control_attitude_enabled) && (_v_control_mode.flag_control_rates_enabled))
            {
                control_attitude(dt);
                float thrust_sp = _v_attitude_sp.thrust;
                _actuators.control[0] = (PX4_ISFINITE(_att_control.roll)) ? _att_control.roll : 0.0f;
                _actuators.control[1] = (PX4_ISFINITE(_att_control.pitch)) ? _att_control.pitch : 0.0f;
                _actuators.control[2] = (PX4_ISFINITE(_att_control.yaw)) ? _att_control.yaw : 0.0f;
                _actuators.control[3] = (PX4_ISFINITE(thrust_sp)) ? thrust_sp : 0.0f;

                _actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators);
            }
        }
        perf_end(_loop_perf); //loop performance end
    } _control_task = -1;
}

int pid_att_control_main(int argc, char* argv[])
{
    /* send a warn if no input argument available */
    if (argc < 2){
        warnx("usage: pid_att_control {start|stop|status}");
        return 1;
    }
    /* start pid_att_control manually */
    if (!strcmp(argv[1], "start")) {
        if (pid_att_control::g_control != nullptr) {
            warnx("Already running!");
            return 1;
        }
        pid_att_control::g_control = new Pid_Att_Control;
        if (pid_att_control::g_control == nullptr) {
            warnx("Allocation failed.");
            return 1;
        }
        if (OK != pid_att_control::g_control->start()) {
            delete pid_att_control::g_control;
            pid_att_control::g_control = nullptr;
            warnx("Start failed.");
            return 1;
        }
        return 0;
    }
    /* stop pid_att_control manually */
    if (!strcmp(argv[1], "stop")) {
        if (pid_att_control::g_control == nullptr) {
            warnx("Not running.");
            return 1;
        }
        delete pid_att_control::g_control;
        pid_att_control::g_control = nullptr;
        return 0;
    }
    /* return running status of the application */
    if (!strcmp(argv[1], "status")) {
        if (pid_att_control::g_control) {
            warnx("Running.");
            return 0;
        } else {
            warnx("Not running.");
            return 1;
        }
    }
    /* if argument is not in one of the if statement */
    warnx("Unrecognized command.");
    return 0;
}
