/**
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_log.h>
#include <string.h> //for include memset
#include <poll.h>

#include <matrix/matrix/math.hpp>

#include <uORB/uORB.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/topics/orb_motor_telemetry_me.h>
#include <uORB/topics/orb_motor_status_me.h>
#include <uORB/topics/orb_motor_status_one_me.h>

#include <uORB/topics/adc_report.h>
#include <uORB/topics/esc_report.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/pwm_input.h>
#include <uORB/topics/debug_value.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/key_command.h>

#include "px4_cn.h"
#include "drivers/dshot/swashplateless/swashplateless.hpp"



//#define PRINT_RC
#define PRINT_PWM_IN
//#define PRINT_MOTOR_STATUS
//#define PRINT_DEBUG_VALUE
//#define PRINT_ACTUATOR_CONTROL
//#define PRINT_VEHICLE_CONTROL
//#define PRINT_SENSOR_MAG
//#define PRINT_SENSOR_ACCEL
//#define PRINT_KEY_COMMAND


extern "C" __EXPORT int px4_cn_main(int argc, char *argv[])
{
    PX4_INFO("px4_cn_main enter");

//    px4_task_spawn_cmd("px4_cn_thread",
//                       SCHED_DEFAULT,
//                       SCHED_PRIORITY_DEFAULT,
//                       1024,
//                       (px4_main_t) uORB_data_print_new,
//                       nullptr);
//    return 0;

    return PX4cn::main(argc, argv);
}


int PX4cn::print_status()
{
    PX4_INFO("Running");
    return 0;
}

int PX4cn::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}


int PX4cn::task_spawn(int argc, char *argv[])
{
    //stack too small (such as 1024) will cause crash when print orb_motor_status_me
    _task_id = px4_task_spawn_cmd("PX4cn",
                                  SCHED_DEFAULT,
                                  SCHED_PRIORITY_DEFAULT,
                                  2048,
                                  (px4_main_t)&run_trampoline,
                                  (char *const *)argv);

    if (_task_id < 0) {
        _task_id = -1;
        return -errno;
    }

    return 0;
}

PX4cn *PX4cn::instantiate(int argc, char *argv[])
{
    int example_param = 0;
    bool example_flag = false;
    bool error_flag = false;

    int myoptind = 1;
    int ch;
    const char *myoptarg = nullptr;

    // parse CLI arguments
    while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
        switch (ch) {
            case 'p':
                example_param = (int)strtol(myoptarg, nullptr, 10);
                break;

            case 'f':
                example_flag = true;
                break;

            case '?':
                error_flag = true;
                break;

            default:
                PX4_WARN("unrecognized flag");
                error_flag = true;
                break;
        }
    }

    if (error_flag) {
        return nullptr;
    }

    PX4cn *instance = new PX4cn(example_param, example_flag);

    if (instance == nullptr) {
        PX4_ERR("alloc failed");
    }

    return instance;
}

PX4cn::PX4cn(int example_param, bool example_flag)
        : ModuleParams(nullptr)
{}



void PX4cn::run()
{
    static SUBSCRIBE_VAR_t subscribe_var{0};

    // subscribe adc_data from adc hardware
    adc_report_s adc_data{0};
    uORB::Subscription adc_report_sub{ORB_ID(adc_report)};

    // subscribe pwm_data from pwm_capture
    pwm_input_s pwm_input_data{0};
    uORB::Subscription pwm_input_sub{ORB_ID(pwm_input)};

    // subscribe rc_data from remote controller
    rc_channels_s rc_data{0};
    uORB::Subscription rc_channels_sub{ORB_ID(rc_channels)};

    // subscribe motor_status from dshot thread
    orb_motor_status_me_s motor_status_data{0};
    uORB::Subscription orb_motor_status_me_sub{ORB_ID(orb_motor_status_me)};
    orb_motor_status_one_me_s motor_status_one_data{0};
    uORB::Subscription orb_motor_status_one_me_sub{ORB_ID(orb_motor_status_one_me)};

    // subscribe debug_value
    debug_value_s debug_value_data{0};
    uORB::Subscription debug_value_sub{ORB_ID(debug_value)};

    // subscribe actuator_controls
    actuator_controls_s actuator_controls_data{0};
    uORB::Subscription actuator_controls_sub{ORB_ID(actuator_controls_0)};

    // subscribe vehicle_attitude_s
    vehicle_attitude_s vehicle_attitude_data{0};
    uORB::Subscription vehicle_attitude_sub{ORB_ID(vehicle_attitude)};

    // subscribe sensor_mag
    sensor_mag_s sensor_mag_data{0};
    uORB::Subscription sensor_mag_sub{ORB_ID(sensor_mag)};

    // subscribe sensor_accel
    sensor_accel_s sensor_accel_data{0};
    uORB::Subscription sensor_accel_sub{ORB_ID(sensor_accel)};

    // subscribe key_command for mavros-mavlink test
    key_command_s key_command_data{0};
    uORB::Subscription key_command_sub{ORB_ID(key_command)};

    PX4_INFO("px4_cn is running");

    while(!should_exit())
    {
#ifdef PRINT_PWM_IN
        if(pwm_input_sub.update(&pwm_input_data))
        {
            subscribe_var.pwm_pulse_width = pwm_input_data.pulse_width;
            subscribe_var.pwm_period = pwm_input_data.period;
            float angle_compensated = motor_angle_cal(subscribe_var.pwm_pulse_width,
                                          SENDOR_PWM_MAX, SENDOR_PWM_MIN, SENSOR_ROTOR_ANGLE_BIAS);
            float angle_raw = motor_angle_cal_raw(subscribe_var.pwm_pulse_width,
                                                 SENDOR_PWM_MAX, SENDOR_PWM_MIN);
            PX4_INFO("width = %d, period = %d, angle_compensated = %.3f deg, angle_raw = %.3f deg",
                     subscribe_var.pwm_pulse_width, subscribe_var.pwm_period, angle_compensated, angle_raw);
        }
#endif

#ifdef PRINT_RC
        if(rc_channels_sub.update(&rc_data))
        {
            subscribe_var.rc_RX = (double)rc_data.channels[0];
            subscribe_var.rc_RY = (double)rc_data.channels[1];
            subscribe_var.rc_LY = (double)rc_data.channels[2];
            subscribe_var.rc_LX = (double)rc_data.channels[3];
//             PX4_INFO("rc = %f %f %f %f", subscribe_var.rc_LX, subscribe_var.rc_LY, subscribe_var.rc_RX, subscribe_var.rc_RY);
            PX4_INFO("0=%.3f, 1=%.3f, 2=%.3f, 3=%.3f, 4=%.3f, 5=%.3f, 6=%.3f,"
                     "7=%.3f, 8=%.3f, 9=%.3f, 10=%.3f, 11=%.3f, 12=%.3f, 13=%.3f",
                     rc_data.channels[0], rc_data.channels[1],
                     rc_data.channels[2], rc_data.channels[3],
                     rc_data.channels[4], rc_data.channels[5],
                     rc_data.channels[6], rc_data.channels[7],
                     rc_data.channels[8], rc_data.channels[9],
                     rc_data.channels[10], rc_data.channels[11],
                     rc_data.channels[12], rc_data.channels[13]
            );
        }
#endif

#ifdef PRINT_MOTOR_STATUS
        if(orb_motor_status_me_sub.update(&motor_status_data))
        {
            static uint32_t counter = 0;
            counter++;
            subscribe_var.motor_angle_deg = motor_status_data.angle_deg[4];
            subscribe_var.motor_speed_rpm = motor_status_data.speed_rpm[4];
            subscribe_var.dt_s = motor_status_data.dt[4];
            if(counter > 10)
            {
                PX4_INFO("Multi: deg = %.3f, rpm = %.3f, dt = %.3f, cnt = %d, out = %d, sin = %.3f, amp = %.3f, pha = %.3f",
                         subscribe_var.motor_angle_deg, subscribe_var.motor_speed_rpm, subscribe_var.dt_s,
                         motor_status_data.frame_counter, motor_status_data.throttle_out[4],
                         motor_status_data.throttle_sin[4], motor_status_data.sin_amp[4],
                         motor_status_data.sin_pha[4]);
                counter = 0;
            }
        }

        if(orb_motor_status_one_me_sub.update(&motor_status_one_data))
        {
            static uint32_t counter = 0;
            counter++;
            subscribe_var.motor_angle_deg = motor_status_one_data.angle_deg;
            subscribe_var.motor_speed_rpm = motor_status_one_data.speed_rpm;
            subscribe_var.dt_s = motor_status_one_data.dt;
            if(counter > 10)
            {
                PX4_INFO("One: deg = %.3f, rpm = %.3f, dt = %.3f, cnt = %d, out = %d, sin = %.3f, amp = %.3f, pha = %.3f",
                         subscribe_var.motor_angle_deg, subscribe_var.motor_speed_rpm, subscribe_var.dt_s,
                         motor_status_one_data.frame_counter,
                         motor_status_one_data.throttle_out,
                         motor_status_one_data.throttle_sin,
                         motor_status_one_data.sin_amp,
                         motor_status_one_data.sin_pha);
                counter = 0;
            }
        }
#endif

#ifdef PRINT_DEBUG_VALUE
        if(debug_value_sub.update(&debug_value_data))
        {
            static float last_value;
            PX4_INFO("debug: ind = %d, value = %.3f", debug_value_data.ind, debug_value_data.value - last_value);
            last_value = debug_value_data.value;
        }
#endif

#ifdef PRINT_ACTUATOR_CONTROL
        if(actuator_controls_sub.update(&actuator_controls_data))
        {
            PX4_INFO("actuator_controls = %.3f, %.3f, %.3f, %.3f",
                     actuator_controls_data.control[0],
                     actuator_controls_data.control[1],
                     actuator_controls_data.control[2],
                     actuator_controls_data.control[3]);
        }
#endif

#ifdef PRINT_VEHICLE_CONTROL
        if(vehicle_attitude_sub.update(&vehicle_attitude_data))
        {
            matrix::Quatf att_q(vehicle_attitude_data.q);
            // for stabilized attitude generation only extract the heading change from the delta quaternion
            const float rad2deg = 57.3;
            float roll_angle = matrix::Eulerf(att_q).phi() * rad2deg;
            float pitch_angle = matrix::Eulerf(att_q).theta() * rad2deg;
            float yaw_angle = matrix::Eulerf(att_q).psi() * rad2deg;
//            PX4_INFO("vehicle_attitude_data = %.3f, %.3f, %.3f, %.3f",
//                     vehicle_attitude_data.q[0],
//                     vehicle_attitude_data.q[1],
//                     vehicle_attitude_data.q[2],
//                     vehicle_attitude_data.q[3]);
            PX4_INFO("vehicle_attitude_data = %.3f, %.3f, %.3f",
                     roll_angle, pitch_angle, yaw_angle);
        }
#endif

#ifdef PRINT_SENSOR_MAG
        if(sensor_mag_sub.update(&sensor_mag_data))
        {
            PX4_INFO("sensor_mag_data = %.3f, %.3f, %.3f",
                     sensor_mag_data.x, sensor_mag_data.y, sensor_mag_data.z);
        }
#endif

#ifdef PRINT_SENSOR_ACCEL
        if(sensor_accel_sub.update(&sensor_accel_data))
        {
            PX4_INFO("sensor_accel_data = %.3f, %.3f, %.3f",
                     sensor_accel_data.x, sensor_accel_data.y, sensor_accel_data.z);
        }
#endif

#ifdef PRINT_KEY_COMMAND
        if(key_command_sub.update(&key_command_data))
        {
            PX4_INFO("key_command_sub = %c", key_command_data.cmd);
        }
#endif

//        PX4_INFO("PX4_INFO");
//        sleep(1);//parameter is int
        usleep(100000);//100ms
    }
}


void PX4cn::parameters_update(bool force)
{}


int PX4cn::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
            R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("module", "template");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
    PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}


