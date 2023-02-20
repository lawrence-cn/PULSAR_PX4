/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file AttitudeControl.cpp
 */

#include <AttitudeControl.hpp>

#include <mathlib/math/Functions.hpp>

using namespace matrix;

void AttitudeControl::setProportionalGain(const matrix::Vector3f &proportional_gain, const float yaw_weight)
{
	_proportional_gain = proportional_gain;
	_yaw_w = math::constrain(yaw_weight, 0.f, 1.f);

	// compensate for the effect of the yaw weight rescaling the output
	if (_yaw_w > 1e-4f) {
		_proportional_gain(2) /= _yaw_w;
	}
}

Dcm<float> XYZ_euler_to_DCM(float a, float b, float c)
{
    Dcm<float> dcm;
    float c1 = float(cos(a));
    float s1 = float(sin(a));
    float c2 = float(cos(b));
    float s2 = float(sin(b));
    float c3 = float(cos(c));
    float s3 = float(sin(c));

    dcm(0, 0) = c2 * c3;
    dcm(0, 1) = -c2 * s3;
    dcm(0, 2) = s2;

    dcm(1, 0) = c1 * s3 + c3 * s1 * s2;
    dcm(1, 1) = c1 * c3 - s1 * s2 * s3;
    dcm(1, 2) = -c2 * s1;

    dcm(2, 0) = s1 * s3 - c1 * c3 * s2;
    dcm(2, 1) = c3 * s1 + c1 * s2 * s3;
    dcm(2, 2) = c1 * c2;
    return dcm;
}

static Vector3f q_to_xyz_euler(Quatf q)
{
    float qw = q(0);
    float qx = q(1);
    float qy = q(2);
    float qz = q(3);
    float x_phi = atan2(2*qy*qw-2*qx*qz , 1 - 2*(qy*qy + qz*qz));
    float y_theta = asin(2*(qx*qy + qz*qw));
    float z_psi = atan2(2*(qx*qw-qy*qz), 1 - 2*(qx*qx + qz*qz));
    Vector3f euler(x_phi, y_theta, z_psi);
    return euler;
}

static Vector3f q_to_zyx_euler(Quatf q)
{
    float qw = q(0);
    float qx = q(1);
    float qy = q(2);
    float qz = q(3);
    float x_phi = atan2(2*(qw*qx + qy*qz), 1-2*(qx*qx + qy*qy));
    float y_theta = asin(2*(qw*qy - qx*qz));
    float z_psi = atan2(2*(qw*qz + qx*qy), 1-2*(qy*qy + qz*qz));
    Vector3f euler(x_phi, y_theta, z_psi);
    return euler;
}

// PULSAR's attitude control

static float matrix_value[9] = {0,-1,0,1,0,0,0,0,0};
static Matrix<float, 3, 3> z_axis_skew{matrix_value};
static Vector3f z_axis{0,0,1};

matrix::Vector3f AttitudeControl::update(const Quatf &q, const Vector3f &thrust_cmd, struct vehicle_control_mode_s mode)
{
    Dcmf Rd{_attitude_setpoint_q};
    Dcmf R{q};
    Vector3f rate_setpoint{0,0,0};

    Dcmf R_T = R.transpose();
    if (!mode.flag_control_position_enabled)// for attitude control manually
    {
        att_setpoint = Rd * z_axis;
        att_feedback = R * z_axis;
        att_error = R * z_axis_skew * R_T * att_setpoint; //vector cross product in world frame
    }
    else //thrust_acc is thrust cmd, for position control
    {
        att_setpoint = -thrust_cmd.normalized(); //thrust is negative in body frame
        att_feedback = R * z_axis;
        att_error = R * z_axis_skew * R_T * att_setpoint; //vector cross product in world frame
    }

    rate_setpoint = att_error.emult(_proportional_gain); //att gain, rate_setpoint is in world frame

    rate_setpoint = R_T * rate_setpoint;//rate_setpoint is converted from world frame to body frame

    for (int i = 0; i < 3; i++)
    {
        rate_setpoint(i) = math::constrain(rate_setpoint(i), -_rate_limit(i), _rate_limit(i));
    }
    return rate_setpoint;
}


void AttitudeControl::publish_att_status(void)
{
    //pub att states for logging
    orb_att_states_me_s _att_states{};
    att_setpoint.copyTo(_att_states.setpoint);
    att_feedback.copyTo(_att_states.feedback);
    att_error.copyTo(_att_states.error);
    _att_states.timestamp = hrt_absolute_time();
    _att_states_pub.publish(_att_states);
}
