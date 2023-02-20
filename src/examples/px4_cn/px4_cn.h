//
// Created by Lawrence Chen on 2020/12/3.
//

#ifndef PX4_PX4_APP_CN_H
#define PX4_PX4_APP_CN_H

#include <px4_time.h>

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>


typedef struct{
    int32_t motor_rpm;

    double adc_V33;
    double adc_V66;

    int32_t esc_rpm;

    double rc_LX;
    double rc_LY;
    double rc_RX;
    double rc_RY;

    uint32_t pwm_pulse_width;
    uint32_t pwm_period;

    float32 motor_angle_deg;
    float32 motor_speed_rpm;
    float32 dt_s;
    uint32_t pulse_width;
    uint32_t pulse_period;
}SUBSCRIBE_VAR_t;

class RunTimeCalculation
{
public:
    double dt_ms;
    double dt_s;
    double freq;

    RunTimeCalculation(struct timespec arg_ts = {0})
    {
        sys_time_ms_now = arg_ts;
        sys_time_ms_last = sys_time_ms_now;
    }
    ~RunTimeCalculation(){};

    double run_time_cal(struct timespec arg_ts)
    {
        sys_time_ms_now = arg_ts;
        double dt_s_part = (double)(sys_time_ms_now.tv_sec - sys_time_ms_last.tv_sec);
        double dt_ns_part = (double)(sys_time_ms_now.tv_nsec - sys_time_ms_last.tv_nsec);
        dt_ms = dt_s_part * 1000.0 + dt_ns_part / 1000000.0;
        dt_s = dt_ms / 1000.0;
        freq = 1.0 / dt_s;
        sys_time_ms_last = sys_time_ms_now;
        return dt_ms;
    }

    void update(struct timespec arg_ts)
    {
        run_time_cal(arg_ts);
    }

private:
    struct timespec sys_time_ms_now;
    struct timespec sys_time_ms_last;
};



class PX4cn : public ModuleBase<PX4cn>, public ModuleParams
{
public:
    PX4cn(int example_param, bool example_flag);

    virtual ~PX4cn() = default;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);

    /** @see ModuleBase */
    static PX4cn *instantiate(int argc, char *argv[]);

    /** @see ModuleBase */
    static int custom_command(int argc, char *argv[]);

    /** @see ModuleBase */
    static int print_usage(const char *reason = nullptr);

    /** @see ModuleBase::run() */
    void run() override;

    /** @see ModuleBase::print_status() */
    int print_status() override;

private:

    /**
     * Check for parameter changes and update them if needed.
     * @param parameter_update_sub uorb subscription to parameter_update
     * @param force for a parameter update
     */
    void parameters_update(bool force = false);

};

#endif //PX4_PX4_APP_CN_H
