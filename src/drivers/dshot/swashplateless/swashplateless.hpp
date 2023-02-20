//
// Created by Lawrence Chen on 2021/4/17.
//

#ifndef PX4_SWASHPLATELESS_HPP
#define PX4_SWASHPLATELESS_HPP

//sensor parameters, need to change when switch experimental platform
//angular velocity of propeller is positive in body frame (z-axis is downward)
//positive blade must be aligned with the negative direction of body y-axis
//define this situation as 0 degree, by changing the SENSOR_ROTOR_ANGLE_BIAS set 0 degree
//so that the moment in 0 degree is aligned with the positive direction of body frame

// On single_axis vehicle
#define SENSOR_ROTOR_ANGLE_BIAS  (227.7)  //deg, encoder installation offset angle
#define SENDOR_PWM_MAX           (9658)
#define SENDOR_PWM_MIN           (292)
// End


//Lag angle compensation is very important, which is obtained in a test stand with a 6-axis force sensor
//Change motor or propeller should also change this value

//#define MOTOR_DELAY_ANLGE_BIAS (2.496) //rad, throttle = 1050
//#define MOTOR_DELAY_ANLGE_BIAS (3.09) //2.496 + 0.594 = 3.09 rad, throttle = 1436, thrust = 11.8N
#define MOTOR_DELAY_ANLGE_BIAS   (2.88)  //165 deg, 1200-1300 dshot, MF1302, standard pulsar, 1234g
//#define MOTOR_DELAY_ANLGE_BIAS   (2.496)  //143 deg, 1200-1300 dshot, MF1302, no lidar pulsar, 750g
//#define MOTOR_DELAY_ANLGE_BIAS   (1.885)  //108 deg, 1200-1300 dshot, MF1503, no lidar pulsar, 929g
//#define MOTOR_DELAY_ANLGE_BIAS   (1.047)  //60 deg, 1200-1300 dshot, MF1604, no lidar pulsar, 760g
//#define MOTOR_DELAY_ANLGE_BIAS (0.0) //for debug



#define SWASH_SIN_DEADZONE (10)

typedef struct
{
    uint64_t timestamp;
    float pulse_angle_deg;
    float pulse_angle_deg_last;
    float pulse_angle_deg_delta;
    float pulse_speed_rpm;
    float throttle_sin;
    float throttle_dc;
    float dt;
    uint16_t throttle_out;
    float ref_x;
    float ref_y;
    float sin_amp;
    float sin_pha;
    float bat_coef;
}Motor_State_Def;

typedef struct
{
    uint32_t pulse_width;
    uint32_t pulse_width_last;
    uint32_t period;
    uint64_t timestamp_curr;
    uint64_t timestamp_last;
    float dt;
    uint32_t width_max;
    uint32_t width_min;
    uint32_t width_range;
}PWM_CAP_Def;

typedef struct
{
    float LX;
    float LY;
    float RX;
    float RY;

    float LX_last;
    float LY_last;
    float RX_last;
    float RY_last;

    int8_t switch_B;
    int8_t switch_C;
    int8_t switch_D;
    int8_t switch_E;
    int8_t switch_G;

    int8_t switch_B_last;
    int8_t switch_C_last;
    int8_t switch_D_last;
    int8_t switch_E_last;
    int8_t switch_G_last;

    float L_amp;
    float L_pha_rad;
    float R_amp;
    float R_pha_rad;
}RC_Def;

typedef struct
{
    uint64_t yaw_rate_timestamp;
    uint64_t yaw_rate_timestamp_last;
    float yaw_rate_dt;

    float vehicle_yaw_rad;
    float vehicle_yaw_rate_radps;

    float body_yaw_rad;
    float body_yaw_rate_radps;

    float actuator_ctrls_roll;
    float actuator_ctrls_pitch;
    float actuator_ctrls_yaw;
    float actuator_ctrls_thrust;
    float actuator_ctrls_amp;
    float actuator_ctrls_pha;

    float bat_vol_now;
    float bat_vol_compen_coeff;
}SINGLE_AXIS_PROP_Def;

float motor_angle_cal(uint32_t pwm_now, uint32_t pwm_max, uint32_t pwm_min, float angle_bias);
uint16_t speed_control_for_dshot(bool on_flag);


#endif //PX4_SWASHPLATELESS_HPP
