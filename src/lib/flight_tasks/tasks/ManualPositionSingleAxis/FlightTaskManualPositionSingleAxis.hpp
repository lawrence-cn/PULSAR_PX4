//
// Created by Lawrence Chen on 2021/6/21.
//

#ifndef PX4_FLIGHTTASKMANUALPOSITIONSINGLEAXIS_HPP
#define PX4_FLIGHTTASKMANUALPOSITIONSINGLEAXIS_HPP

#include "FlightTask.hpp"
#include "Sticks.hpp"


class FlightTaskManualPositionSingleAxis : public FlightTask
{
public:
    FlightTaskManualPositionSingleAxis();

    virtual ~FlightTaskManualPositionSingleAxis() = default;
    bool update() override;
    bool activate(const vehicle_local_position_setpoint_s &last_setpoint) override;
    bool updateInitialize() override;
    void _updateXYlock();
    void _updateAltitudeLock();
    void _scaleSticks();
    void velocity_from_stick_me();

private:
    float _velocity_scale{0.0f}; //scales the stick input to velocity
    uint8_t _reset_counter_xy{0}; /**< counter for estimator resets in xy-direction */
    uint8_t _reset_counter_z{0}; /**< counter for estimator resets in z-direction */

protected:
    Sticks _sticks;
    bool _sticks_data_required = true; ///< let inherited task-class define if it depends on stick data

    DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTask,
    (ParamFloat<px4::params::MPC_HOLD_MAX_XY>) _param_mpc_hold_max_xy,
    (ParamFloat<px4::params::MPC_LAND_ALT1>) _param_mpc_land_alt1, /**< altitude at which to start downwards slowdown */
    (ParamFloat<px4::params::MPC_LAND_ALT2>) _param_mpc_land_alt2, /**< altitude below wich to land with land speed */
    (ParamFloat<px4::params::MPC_LAND_VEL_XY>) _param_mpc_land_vel_xy,

    (ParamFloat<px4::params::MPC_HOLD_MAX_Z>) _param_mpc_hold_max_z,
    (ParamInt<px4::params::MPC_ALT_MODE>) _param_mpc_alt_mode
    )
};

#endif //PX4_FLIGHTTASKMANUALPOSITIONSINGLEAXIS_HPP
