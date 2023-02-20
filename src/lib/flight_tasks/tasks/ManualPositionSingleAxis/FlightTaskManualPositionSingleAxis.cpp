//
// Created by Lawrence Chen on 2021/6/21.
//

#include "FlightTaskManualPositionSingleAxis.hpp"

using namespace matrix;


FlightTaskManualPositionSingleAxis::FlightTaskManualPositionSingleAxis() :
    _sticks(this)
{}

bool FlightTaskManualPositionSingleAxis::updateInitialize()//called before update everytime
{
    //_time_stamp_current is inherited and initialized by the parent function updateInitialize
    bool ret = FlightTask::updateInitialize();

    _sticks.checkAndSetStickInputs(_time_stamp_current);
    _sticks.setGearAccordingToSwitch(_gear);

    if (_sticks_data_required)
    {
        ret = ret && _sticks.isAvailable();
    }

    // in addition to manual require valid position and velocity in D-direction and valid yaw
    return ret && PX4_ISFINITE(_position(2)) && PX4_ISFINITE(_velocity(2));
}

void FlightTaskManualPositionSingleAxis::_updateXYlock()
{
    /* If position lock is not active, position setpoint is set to NAN.*/
    const float vel_xy_norm = matrix::Vector2f(_velocity).length();
    const bool apply_brake = matrix::Vector2f(_velocity_setpoint).length() < FLT_EPSILON;
    const bool stopped = (_param_mpc_hold_max_xy.get() < FLT_EPSILON || vel_xy_norm < _param_mpc_hold_max_xy.get());

    if (apply_brake && stopped && !PX4_ISFINITE(_position_setpoint(0)))
    {
        _position_setpoint(0) = _position(0);
        _position_setpoint(1) = _position(1);
    }
    else if (PX4_ISFINITE(_position_setpoint(0)) && apply_brake)
    {
        // Position is locked but check if a reset event has happened.
        // We will shift the setpoints.
        if (_sub_vehicle_local_position.get().xy_reset_counter != _reset_counter_xy)
        {
            _position_setpoint(0) = _position(0);
            _position_setpoint(1) = _position(1);
            _reset_counter_xy = _sub_vehicle_local_position.get().xy_reset_counter;
        }
    }
    else
    {
        /* don't lock*/
        _position_setpoint(0) = NAN;
        _position_setpoint(1) = NAN;
    }
}

void FlightTaskManualPositionSingleAxis::_updateAltitudeLock()
{
    // Depending on stick inputs and velocity, position is locked.
    // If not locked, altitude setpoint is set to NAN.

    // Check if user wants to break
    const bool apply_brake = fabsf(_sticks.getPositionExpo()(2)) <= FLT_EPSILON;

    // Check if vehicle has stopped
    const bool stopped = (_param_mpc_hold_max_z.get() < FLT_EPSILON || fabsf(_velocity(2)) < _param_mpc_hold_max_z.get());

    // Manage transition between use of distance to ground and distance to local origin
    // when terrain hold behaviour has been selected.

    if (apply_brake && stopped && !PX4_ISFINITE(_position_setpoint(2)))
    {
        _position_setpoint(2) = _position(2);
    }
    else if (PX4_ISFINITE(_position_setpoint(2)) && apply_brake)
    {
        // Position is locked but check if a reset event has happened.
        // We will shift the setpoints.
        if (_sub_vehicle_local_position.get().z_reset_counter != _reset_counter_z)
        {
            _position_setpoint(2) = _position(2);
            _reset_counter_z = _sub_vehicle_local_position.get().z_reset_counter;
        }
    }
    else
    {
        // user demands velocity change
        _position_setpoint(2) = NAN;
    }
}

void FlightTaskManualPositionSingleAxis::_scaleSticks()
{
    //xy direction process
    /* Constrain length of stick inputs to 1 for xy*/
    Vector2f stick_xy = _sticks.getPositionExpo().slice<2, 1>(0, 0);
    const float mag = math::constrain(stick_xy.length(), 0.0f, 1.0f);
    if (mag > FLT_EPSILON)
    {
        stick_xy = stick_xy.normalized() * mag;
    }
    // scale velocity to its maximum limits
    _velocity_scale = _constraints.speed_xy;
    _velocity_setpoint.xy() = stick_xy * _velocity_scale;

    //z direction process
    // Use sticks input with deadzone and exponential curve for vertical velocity
    const float vel_max_z = (_sticks.getPosition()(2) > 0.0f) ? _constraints.speed_down : _constraints.speed_up;
    _velocity_setpoint(2) = vel_max_z * _sticks.getPositionExpo()(2);
}

void FlightTaskManualPositionSingleAxis::velocity_from_stick_me()
{
    Vector<float, 4> stick_set = _sticks.getPositionExpo();
//    _position_setpoint(0) += stick_set(0) * 0.01;
//    _position_setpoint(1) += stick_set(1) * 0.01;
//    _position_setpoint(2) += stick_set(2) * 0.01;
    _velocity_setpoint(0) = stick_set(0);
    _velocity_setpoint(1) = stick_set(1);
    _velocity_setpoint(2) = stick_set(2);
    //PX4_INFO("value = %.2f, %.2f, %.2f, %.2f", stick_set(0), stick_set(1), stick_set(2), stick_set(3));
}

bool FlightTaskManualPositionSingleAxis::update()
{
    _acceleration_setpoint.setNaN(); // don't use the horizontal setpoints from FlightTaskAltitude
    _scaleSticks(); //use data from sticks to determine xy and z velocities
    _updateXYlock(); // check for position lock
    _updateAltitudeLock(); // check for altitude lock
    return true;
}

bool FlightTaskManualPositionSingleAxis::activate(const vehicle_local_position_setpoint_s &last_setpoint)
{
    bool ret = FlightTask::activate(last_setpoint);//set default constraints and reset all setpoints to NAN
    _position_setpoint = _position;
    _velocity_setpoint.setZero();
    return ret;
}

