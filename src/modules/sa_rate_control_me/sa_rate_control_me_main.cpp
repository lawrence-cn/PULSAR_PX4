//
// Created by Lawrence Chen on 2021/4/17.
//

#include "sa_rate_control_me_main.hpp"

using namespace matrix;
using namespace time_literals;
using math::radians;

SingleAxisRateControl::SingleAxisRateControl() :
        ModuleParams(nullptr),
        WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
        _actuators_0_pub(ORB_ID(actuator_controls_0)),
        _loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
  _vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
  parameters_updated();

  current_time = hrt_absolute_time();
  last_time = hrt_absolute_time();
}

SingleAxisRateControl::~SingleAxisRateControl()
{
    perf_free(_loop_perf);
}

bool SingleAxisRateControl::init()
{
    if (!_vehicle_angular_velocity_sub.registerCallback())
    {
        PX4_ERR("vehicle_angular_velocity callback registration failed!");
        return false;
    }
    PX4_INFO("vehicle_angular_velocity callback registration successful");
    return true;
}

void SingleAxisRateControl::parameters_updated()
{
    // rate control parameters
    // The controller gain K is used to convert the parallel (P + I/s + sD) form
    // to the ideal (K * [1 + 1/sTi + sTd]) form
    const Vector3f rate_k = Vector3f(_param_mc_rollrate_k.get(), _param_mc_pitchrate_k.get(), _param_mc_yawrate_k.get());

    _rate_control.setGains(
            rate_k.emult(Vector3f(_param_mc_rollrate_p.get(), _param_mc_pitchrate_p.get(), _param_mc_yawrate_p.get())),
            rate_k.emult(Vector3f(_param_mc_rollrate_i.get(), _param_mc_pitchrate_i.get(), _param_mc_yawrate_i.get())),
            rate_k.emult(Vector3f(_param_mc_rollrate_d.get(), _param_mc_pitchrate_d.get(), _param_mc_yawrate_d.get())));

    _rate_control.setIntegratorLimit(
            Vector3f(_param_mc_rr_int_lim.get(), _param_mc_pr_int_lim.get(), _param_mc_yr_int_lim.get()));

    _rate_control.setFeedForwardGain(
            Vector3f(_param_mc_rollrate_ff.get(), _param_mc_pitchrate_ff.get(), _param_mc_yawrate_ff.get()));


    // manual rate control acro mode rate limits
    _acro_rate_max = Vector3f(radians(_param_mc_acro_r_max.get()), radians(_param_mc_acro_p_max.get()),
                              radians(_param_mc_acro_y_max.get()));

    _actuators_0_circuit_breaker_enabled = circuit_breaker_enabled_by_val(_param_cbrk_rate_ctrl.get(), CBRK_RATE_CTRL_KEY);
}

float SingleAxisRateControl::get_landing_gear_state()
{
    // Only switch the landing gear up if we are not landed and if
    // the user switched from gear down to gear up.
    // If the user had the switch in the gear up position and took off ignore it
    // until he toggles the switch to avoid retracting the gear immediately on takeoff.
    if (_landed) {
        _gear_state_initialized = false;
    }

    float landing_gear = landing_gear_s::GEAR_DOWN; // default to down

    if (_manual_control_setpoint.gear_switch == manual_control_setpoint_s::SWITCH_POS_ON && _gear_state_initialized) {
        landing_gear = landing_gear_s::GEAR_UP;

    } else if (_manual_control_setpoint.gear_switch == manual_control_setpoint_s::SWITCH_POS_OFF) {
        // Switching the gear off does put it into a safe defined state
        _gear_state_initialized = true;
    }

    return landing_gear;
}

//void SingleAxisRateControl::Run()
//{
//    static uint64_t frame_counter = 0;
//    current_time = hrt_absolute_time();
//    frame_counter++;
//    //although can print to debug uart port, but it will seriously affect the running speed!
//    //PX4_INFO("SingleAxisRateControl, counter = %d, time = %d", frame_counter, current_time);
//
//    //printing debug_value_s in px4_cn for frequency test, result = 794Hz
//    /* run controller on gyro changes */
//    vehicle_angular_velocity_s angular_velocity;
//    if (_vehicle_angular_velocity_sub.update(&angular_velocity))
//    {
//        debug_value_s debug_value_data;
//        debug_value_data.timestamp = hrt_absolute_time();
//        debug_value_data.value = (float) frame_counter++;
//        _debug_value_pub.publish(debug_value_data);
//    }
//}

void SingleAxisRateControl::Run()
{
    if (should_exit()) {
        _vehicle_angular_velocity_sub.unregisterCallback();
        exit_and_cleanup();
        return;
    }

    //begin cn
    static uint64_t frame_counter = 0;
    //although can print to debug uart port, but it will seriously affect the running speed!
    //PX4_INFO("SingleAxisRateControl, counter = %d, time = %d", frame_counter, current_time);
    //printing debug_value_s in px4_cn for frequency test, result = 794Hz
    //end cn

    perf_begin(_loop_perf);

    // Check if parameters have changed
    if (_parameter_update_sub.updated()) {
        // clear update
        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);

        updateParams();
        parameters_updated();
    }

    /* run controller on gyro changes */
    vehicle_angular_velocity_s angular_velocity;

    if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

        // grab corresponding vehicle_angular_acceleration immediately after vehicle_angular_velocity copy
        vehicle_angular_acceleration_s v_angular_acceleration{};
        _vehicle_angular_acceleration_sub.copy(&v_angular_acceleration);

        const hrt_abstime now = angular_velocity.timestamp_sample;

        // Guard against too small (< 0.125ms) and too large (> 20ms) dt's.
        const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
        _last_run = now;

        Vector3f angular_accel{v_angular_acceleration.xyz};
        Vector3f rates{angular_velocity.xyz};

        if (_vehicle_land_detected_sub.updated())
        {
            vehicle_land_detected_s vehicle_land_detected;

            if (_vehicle_land_detected_sub.copy(&vehicle_land_detected))
            {
                _landed = vehicle_land_detected.landed;
                _maybe_landed = vehicle_land_detected.maybe_landed;
            }
        }

        _vehicle_status_sub.update(&_vehicle_status);

        const bool manual_control_updated = _manual_control_setpoint_sub.update(&_manual_control_setpoint);

        // generate the rate setpoint from sticks?
        bool manual_rate_sp = false;

        /* check for updates in other topics */
        _v_control_mode_sub.update(&_v_control_mode);

        if (_v_control_mode.flag_control_manual_enabled &&
            !_v_control_mode.flag_control_altitude_enabled &&
            !_v_control_mode.flag_control_velocity_enabled &&
            !_v_control_mode.flag_control_position_enabled)
        {

            // landing gear controlled from stick inputs if we are in Manual/Stabilized mode
            //  limit landing gear update rate to 10 Hz
            if ((now - _landing_gear.timestamp) > 100_ms) {
                _landing_gear.landing_gear = get_landing_gear_state();
                _landing_gear.timestamp = hrt_absolute_time();
                _landing_gear_pub.publish(_landing_gear);
            }

            if (!_v_control_mode.flag_control_attitude_enabled) {
                manual_rate_sp = true;
            }

            // Check if we are in rattitude mode and the pilot is within the center threshold on pitch and roll
            //  if true then use published rate setpoint, otherwise generate from manual_control_setpoint (like acro)
            if (_v_control_mode.flag_control_rattitude_enabled) {
                manual_rate_sp =
                        (fabsf(_manual_control_setpoint.y) > _param_mc_ratt_th.get()) ||
                        (fabsf(_manual_control_setpoint.x) > _param_mc_ratt_th.get());
            }

        }
        else
        {
            _landing_gear_sub.update(&_landing_gear);
        }

        if (manual_rate_sp)
        {
            if (manual_control_updated)
            {

                // manual rates control - ACRO mode
                //superexpo is a 1/(1-x) function to further shape the rc input curve intuitively.
                const Vector3f man_rate_sp{
                        math::superexpo(_manual_control_setpoint.y, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
                        math::superexpo(-_manual_control_setpoint.x, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
                        math::superexpo(_manual_control_setpoint.r, _param_mc_acro_expo_y.get(), _param_mc_acro_supexpoy.get())};

                _rates_sp = man_rate_sp.emult(_acro_rate_max);
                _thrust_sp = _manual_control_setpoint.z;

                // publish rate setpoint
                vehicle_rates_setpoint_s v_rates_sp{};
                v_rates_sp.roll = _rates_sp(0);
                v_rates_sp.pitch = _rates_sp(1);
                v_rates_sp.yaw = _rates_sp(2);
                v_rates_sp.thrust_body[0] = 0.0f;
                v_rates_sp.thrust_body[1] = 0.0f;
                v_rates_sp.thrust_body[2] = -_thrust_sp;
                v_rates_sp.timestamp = hrt_absolute_time();

                _v_rates_sp_pub.publish(v_rates_sp);
            }
        }
        else
        {
            // use rates setpoint topic
            vehicle_rates_setpoint_s v_rates_sp;

            if (_v_rates_sp_sub.update(&v_rates_sp))
            {
                _rates_sp(0) = v_rates_sp.roll;
                _rates_sp(1) = v_rates_sp.pitch;
                _rates_sp(2) = v_rates_sp.yaw;
                _thrust_sp = -v_rates_sp.thrust_body[2];
            }
        }

        // run the rate controller
        if (_v_control_mode.flag_control_rates_enabled && !_actuators_0_circuit_breaker_enabled)
        {

            // reset integral if disarmed
            if (!_v_control_mode.flag_armed ||
                _vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING)
            {
                _rate_control.resetIntegral();
            }

            // update saturation status from mixer feedback
            if (_motor_limits_sub.updated())
            {
                multirotor_motor_limits_s motor_limits;

                if (_motor_limits_sub.copy(&motor_limits))
                {
                    MultirotorMixer::saturation_status saturation_status;
                    saturation_status.value = motor_limits.saturation_status;

                    _rate_control.setSaturationStatus(saturation_status);
                }
            }

            // run rate controller in body frame
            const Vector3f att_control = _rate_control.update(rates, _rates_sp, angular_accel, dt, _maybe_landed || _landed);

            // transform the rate setpoint, feedback, and angular_accel to world frame to log
            if (vehicle_attitude_sub.update(&vehicle_attitude_data))
            {
                R_updated_counter = 0;
                //vehicle_attitude_data.q is an array so must be converted to quaternion first
                matrix::Quatf att_q{vehicle_attitude_data.q};
                matrix::Dcmf R_temp{att_q};
                R_ib = R_temp;
            }
            matrix::Vector3f rates_set_i = R_ib * _rates_sp;
            matrix::Vector3f rates_fb_i = R_ib * rates;
            matrix::Vector3f rates_acc_fb_i = R_ib * angular_accel;

            // publish rate controller status
            rate_ctrl_status_s rate_ctrl_status{};
            _rate_control.getRateControlStatus(rate_ctrl_status);
            rate_ctrl_status.timestamp = hrt_absolute_time();
            _controller_status_pub.publish(rate_ctrl_status);

            // publish actuator controls
            actuator_controls_s actuators{};
            actuators.control[actuator_controls_s::INDEX_ROLL] = PX4_ISFINITE(att_control(0)) ? att_control(0) : 0.0f;
            actuators.control[actuator_controls_s::INDEX_PITCH] = PX4_ISFINITE(att_control(1)) ? att_control(1) : 0.0f;
            actuators.control[actuator_controls_s::INDEX_YAW] = PX4_ISFINITE(att_control(2)) ? att_control(2) : 0.0f;
            actuators.control[actuator_controls_s::INDEX_YAW] = 0.6;
            actuators.control[actuator_controls_s::INDEX_THROTTLE] = PX4_ISFINITE(_thrust_sp) ? _thrust_sp : 0.0f;
            actuators.control[actuator_controls_s::INDEX_LANDING_GEAR] = (float)_landing_gear.landing_gear;
            actuators.timestamp_sample = angular_velocity.timestamp_sample;

            // scale effort by battery status if enabled
            if (_param_mc_bat_scale_en.get())
            {
                if (_battery_status_sub.updated())
                {
                    battery_status_s battery_status;

                    if (_battery_status_sub.copy(&battery_status)) {
                        _battery_status_scale = battery_status.scale;
                    }
                }

                if (_battery_status_scale > 0.0f)
                {
                    for (int i = 0; i < 4; i++)
                    {
                        actuators.control[i] *= _battery_status_scale;
                    }
                }
            }
            actuators.timestamp = hrt_absolute_time();
            _actuators_0_pub.publish(actuators);

            //begin cn, add pub
            //publish data in world frame for logging
            vehicle_angular_velocity_s rate_temp{};
            rate_temp.xyz[0] = rates_set_i(0);
            rate_temp.xyz[1] = rates_set_i(1);
            rate_temp.xyz[2] = rates_set_i(2);
            rate_temp.timestamp = hrt_absolute_time();
            rate_set_pub.publish(rate_temp);

            rate_temp.xyz[0] = rates_fb_i(0);
            rate_temp.xyz[1] = rates_fb_i(1);
            rate_temp.xyz[2] = rates_fb_i(2);
            rate_temp.timestamp = hrt_absolute_time();
            rate_fb_pub.publish(rate_temp);
            //end cn, add pub
        }
        else if (_v_control_mode.flag_control_termination_enabled)
        {
            if (!_vehicle_status.is_vtol)
            {
                // publish actuator controls
                actuator_controls_s actuators{};
                actuators.timestamp = hrt_absolute_time();
                _actuators_0_pub.publish(actuators);
            }
        }
    }

    perf_end(_loop_perf);
}


int SingleAxisRateControl::task_spawn(int argc, char *argv[])
{
    SingleAxisRateControl *instance = new SingleAxisRateControl();
    PX4_INFO("SingleAxisRateControl task_spawn new instance");

    if (instance)
    {
        _object.store(instance);
        _task_id = task_id_is_work_queue;
        if (instance->init())
        {
            return PX4_OK;
        }
    }
    else
    {
        PX4_ERR("alloc failed");
    }

    delete instance;
    _object.store(nullptr);
    _task_id = -1;

    return PX4_ERROR;
}

int SingleAxisRateControl::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int SingleAxisRateControl::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
            R"DESCR_STR(
### Description
This implements the single-axis rate controller. It takes rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller has a PID loop for angular rate error.

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("sa_rate_control", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int SingleAxisRateControl::print_status()
{
    PX4_INFO("Running");
    // TODO: print additional runtime information about the state of the module
    return 0;
}


extern "C" __EXPORT int sa_rate_control_me_main(int argc, char *argv[])
{
    PX4_INFO("sa_rate_control_me_main running");
    return SingleAxisRateControl::main(argc, argv);
}



//module example function, creat a new thread
/*
int sa_rate_control_me_task(int argc, char *argv[])
{
    PX4_INFO("sa_rate_control_me is running");
    for(int i = 0; i<10; i++)
    {
        printf("i = %d\n", i);
        sleep(1);
    }
    return 0;
}


int sa_rate_control_me_main(int argc, char *argv[])
{
    px4_task_spawn_cmd("sa_rate_control_me",
                       SCHED_DEFAULT,
                       SCHED_PRIORITY_DEFAULT,
                       2048,
                       (px4_main_t) sa_rate_control_me_task,
                       nullptr);
    return 0;
}
*/

