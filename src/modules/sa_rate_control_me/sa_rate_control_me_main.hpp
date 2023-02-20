//
// Created by Lawrence Chen on 2021/4/17.
//

#ifndef PX4_SA_RATE_CONTROL_ME_MAIN_HPP
#define PX4_SA_RATE_CONTROL_ME_MAIN_HPP

#pragma once

//PID calculation library
#include <./mc_rate_control/RateControl/RateControl.hpp>

//PX4 fundamental library
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

//uORB topic
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/orb_motor_status_me.h>
#include <uORB/topics/debug_value.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/landing_gear.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_angular_acceleration.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_attitude.h>

//math calculation library
#include <lib/matrix/matrix/math.hpp>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

//assistant library
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>


class SingleAxisRateControl : public ModuleBase<SingleAxisRateControl>, public ModuleParams, public px4::WorkItem
{
public:
    SingleAxisRateControl();

    ~SingleAxisRateControl() override;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);

    /** @see ModuleBase */
    static int custom_command(int argc, char *argv[]);

    /** @see ModuleBase */
    static int print_usage(const char *reason = nullptr);

    /** @see ModuleBase::print_status() */
    int print_status() override;

    bool init();

private:
    /** @see ModuleBase::run() */
    void Run() override;

    /**
     * Check for parameter changes and update them if needed.
     * @param parameter_update_sub uorb subscription to parameter_update
     * @param force for a parameter update
     */
    void parameters_updated();

    float get_landing_gear_state();

    RateControl _rate_control; ///< class for rate control calculations

    // Subscriptions
    uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};

    uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
    uORB::Subscription _vehicle_angular_acceleration_sub{ORB_ID(vehicle_angular_acceleration)};
    uORB::Subscription _v_control_mode_sub{ORB_ID(vehicle_control_mode)};
    uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
    uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
    uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
    uORB::Subscription _landing_gear_sub{ORB_ID(landing_gear)};
    uORB::Subscription _v_rates_sp_sub{ORB_ID(vehicle_rates_setpoint)};
    uORB::Subscription _motor_limits_sub{ORB_ID(multirotor_motor_limits)};
    uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
    uORB::Subscription vehicle_attitude_sub{ORB_ID(vehicle_attitude)};//cn

    // Publications
    //begin, add by cn
    uORB::Publication<debug_value_s>            _debug_value_pub{ORB_ID(debug_value)};//for debug
    uORB::Publication<vehicle_angular_velocity_s> rate_fb_pub{ORB_ID(vehicle_rate_fb_c)};
    uORB::Publication<vehicle_angular_velocity_s> rate_set_pub{ORB_ID(vehicle_rate_set_c)};
    //end, add by cn
    uORB::Publication<actuator_controls_s>      _actuators_0_pub;
    uORB::Publication<landing_gear_s>           _landing_gear_pub{ORB_ID(landing_gear)};
    uORB::Publication<vehicle_rates_setpoint_s>	_v_rates_sp_pub{ORB_ID(vehicle_rates_setpoint)};
    uORB::PublicationMulti<rate_ctrl_status_s>	_controller_status_pub{ORB_ID(rate_ctrl_status), ORB_PRIO_DEFAULT};	/**< controller status publication */

    //uORB data struct
    landing_gear_s 			  _landing_gear{};
    manual_control_setpoint_s _manual_control_setpoint{};
    vehicle_control_mode_s	  _v_control_mode{};
    vehicle_status_s          _vehicle_status{};

    //begin, add by cn
    vehicle_attitude_s vehicle_attitude_data{};
    float I_array[9]{1,0,0,0,1,0,0,0,1};
    matrix::Dcmf R_ib{I_array};
    uint64_t current_time;
    uint64_t last_time;
    uint32_t R_updated_counter{0};
    //end, add by cn

    //flag variables
    bool _actuators_0_circuit_breaker_enabled{false};	/**< circuit breaker to suppress output */
    bool _landed{true};
    bool _maybe_landed{true};
    bool _gear_state_initialized{false};		/**< true if the gear state has been initialized */

    //assitant variables
    perf_counter_t	_loop_perf;			/**< loop duration performance counter */
    hrt_abstime _last_run{0};
    matrix::Vector3f _rates_sp;			/**< angular rates setpoint */
    matrix::Vector3f _acro_rate_max;	/**< max attitude rates in acro mode */
    float _thrust_sp{0.0f};		/**< thrust setpoint */
    float _battery_status_scale{0.0f};

    DEFINE_PARAMETERS(
    (ParamFloat<px4::params::MC_ROLLRATE_P>) _param_mc_rollrate_p,
    (ParamFloat<px4::params::MC_ROLLRATE_I>) _param_mc_rollrate_i,
    (ParamFloat<px4::params::MC_RR_INT_LIM>) _param_mc_rr_int_lim,
    (ParamFloat<px4::params::MC_ROLLRATE_D>) _param_mc_rollrate_d,
    (ParamFloat<px4::params::MC_ROLLRATE_FF>) _param_mc_rollrate_ff,
    (ParamFloat<px4::params::MC_ROLLRATE_K>) _param_mc_rollrate_k,

    (ParamFloat<px4::params::MC_PITCHRATE_P>) _param_mc_pitchrate_p,
    (ParamFloat<px4::params::MC_PITCHRATE_I>) _param_mc_pitchrate_i,
    (ParamFloat<px4::params::MC_PR_INT_LIM>) _param_mc_pr_int_lim,
    (ParamFloat<px4::params::MC_PITCHRATE_D>) _param_mc_pitchrate_d,
    (ParamFloat<px4::params::MC_PITCHRATE_FF>) _param_mc_pitchrate_ff,
    (ParamFloat<px4::params::MC_PITCHRATE_K>) _param_mc_pitchrate_k,

    (ParamFloat<px4::params::MC_YAWRATE_P>) _param_mc_yawrate_p,
    (ParamFloat<px4::params::MC_YAWRATE_I>) _param_mc_yawrate_i,
    (ParamFloat<px4::params::MC_YR_INT_LIM>) _param_mc_yr_int_lim,
    (ParamFloat<px4::params::MC_YAWRATE_D>) _param_mc_yawrate_d,
    (ParamFloat<px4::params::MC_YAWRATE_FF>) _param_mc_yawrate_ff,
    (ParamFloat<px4::params::MC_YAWRATE_K>) _param_mc_yawrate_k,

    (ParamFloat<px4::params::MPC_MAN_Y_MAX>) _param_mpc_man_y_max,			/**< scaling factor from stick to yaw rate */

    (ParamFloat<px4::params::MC_ACRO_R_MAX>) _param_mc_acro_r_max,
    (ParamFloat<px4::params::MC_ACRO_P_MAX>) _param_mc_acro_p_max,
    (ParamFloat<px4::params::MC_ACRO_Y_MAX>) _param_mc_acro_y_max,
    (ParamFloat<px4::params::MC_ACRO_EXPO>) _param_mc_acro_expo,				/**< expo stick curve shape (roll & pitch) */
    (ParamFloat<px4::params::MC_ACRO_EXPO_Y>) _param_mc_acro_expo_y,				/**< expo stick curve shape (yaw) */
    (ParamFloat<px4::params::MC_ACRO_SUPEXPO>) _param_mc_acro_supexpo,			/**< superexpo stick curve shape (roll & pitch) */
    (ParamFloat<px4::params::MC_ACRO_SUPEXPOY>) _param_mc_acro_supexpoy,			/**< superexpo stick curve shape (yaw) */

    (ParamFloat<px4::params::MC_RATT_TH>) _param_mc_ratt_th,

    (ParamBool<px4::params::MC_BAT_SCALE_EN>) _param_mc_bat_scale_en,

    (ParamInt<px4::params::CBRK_RATE_CTRL>) _param_cbrk_rate_ctrl
    )
};

#endif //PX4_SA_RATE_CONTROL_ME_MAIN_HPP
