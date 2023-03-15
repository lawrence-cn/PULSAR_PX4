//
// Created by Lawrence Chen on 2021/4/17.
//

#include <float.h>
#include <math.h>

#include <systemlib/mavlink_log.h>
#include <examples/px4_cn/px4_cn.h>
#include <matrix/matrix/math.hpp>

#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/adc_report.h>
#include <uORB/topics/pwm_input.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/debug_value.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/orb_motor_status_me.h>
#include <uORB/topics/orb_motor_status_one_me.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_control_mode.h>

#include <swashplateless.hpp>

//using namespace matrix;

#define THROTTLE_MAX  1800 //maximum is 2047
#define THROTTLE_MIN  50
#define LOG_ONCE_LEN  5
#define RC_SWITCH_THROTTLE 1000 //400,600,800,1000
#define CONTROL_MAX_THROTTLE (1800) //original is 1800
#define ACTUATOR_CONTROLS_TO_DSHOT (2000)


#define CONST_MOMENT_TO_SIN_AMP  (147)  //coefficient from desired moment to sin amplitude
#define CONST_SIN_AMP_LIMIT      (200)  //amp 200 = 0.6 Nm
#define CONST_BAT_VOLT_FULL      (4.0f) //per cell voltage, consider voltage drop in line
#define CONST_BAT_CELL_NUM       (6.0f)


#define MOTOR_SPEED_LARGE_DT           (0.002)
#define MOTOR_SPEED_SMALL_DT           (0.0001)
#define MOTOR_SPEED_PWM_DEAD_ZONE      2000


#define USE_LOG_ORB_MOTOR_STATUS_ME

//#define ROTATION_ANTI_CLOCKWISE
//#define USING_SQUARE_WAVE

#define USE_RC_RIGHT_STICK //rc right stick can control the sin amp and pha

static orb_advert_t mavlink_log_pub = nullptr;
bool orb_subscribe_flag = false;

// subscribe rc_data from remote controller
rc_channels_s rc_data{0};
uORB::Subscription rc_channels_sub{ORB_ID(rc_channels)};

// subscribe orb pwm_input
pwm_input_s pwm_input_data{0};
uORB::Subscription pwm_input_sub{ORB_ID(pwm_input)};

// subscribe actuator_controls
actuator_controls_s actuator_controls_data{0};
uORB::Subscription actuator_controls_sub{ORB_ID(actuator_controls_0)};

// subscribe vehicle_attitude
vehicle_attitude_s vehicle_attitude_data{0};
uORB::Subscription vehicle_attitude_sub{ORB_ID(vehicle_attitude)};

// subscribe vehicle_angular_velocity
vehicle_angular_velocity_s vehicle_angular_velocity_data{0};
uORB::Subscription vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};

// subscribe battery_status
battery_status_s battery_status_data{0};
uORB::Subscription battery_status_sub{ORB_ID(battery_status)};

// subscribe vehicle_control_mode
vehicle_control_mode_s vehicle_control_mode_data{0};
uORB::Subscription vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};


// publish orb_motor_status_me
orb_motor_status_me_s status_for_log = {0};
uORB::Publication<orb_motor_status_me_s> motor_status_pub{ORB_ID(orb_motor_status_me)};
orb_motor_status_one_me_s status_one_for_log = {0};
uORB::Publication<orb_motor_status_one_me_s> motor_status_one_pub{ORB_ID(orb_motor_status_one_me)};

// publish debug_value
//debug_value_s debug_value_data;
//uORB::Publication<debug_value_s> debug_value_pub{ORB_ID(debug_value)};//for debug

static struct timespec ts;
static RunTimeCalculation time_cal(ts);


static double sys_time_ms = 0.0;
static double sys_time_ms_last = 0.0;
static double dt, freq;
static double const_2_pi = 6.28318;
static double const_pi = 3.14159;
static double const_half_pi = 3.14159 / 2;
static double deg_to_rad = 57.296;

static double speed_ref = 0.0;
static double error_inte = 0.0;
static double dshot_dt = 0.001;
static double adc_feedback = 0.0, pos_feedback = 0.0, pos_feedback_last = 0.0;

static PWM_CAP_Def pwm_cap =
    {
        .width_max = SENDOR_PWM_MAX,
        .width_min = SENDOR_PWM_MIN,
    };

static SINGLE_AXIS_PROP_Def single_axis{0};
static RC_Def rc{0};


static Motor_State_Def motor_state{0};
static Motor_State_Def motor_state_log[LOG_ONCE_LEN]{0};
static int8_t log_index = 0;
static uint32_t frame_counter = 0;

static uint64_t inject_begin_timestamp = 0;
static float force_ref_x = 0, force_ref_y = 0, force_ref_amp = 0, force_ref_pha = 0;

//butter,fc=10Hz,fs=910Hz
static float b[] = {0.0011, 0.0022, 0.0011};
static float a[] = {1, -1.9024, 0.9070};


static float v1m1 = 0, v2m1 = 0, v1m, v2m;
static double iirfilter(float x1)
{
    float y1 = 0;
    y1 = (b[0] * x1 + v1m1) / a[0];
    v1m = (b[1] * x1 + v2m1) - a[1] * y1;
    v2m = b[2] * x1 - a[2] * y1;
    v1m1 = v1m;
    v2m1 = v2m;
    return y1;
}


static float q_to_z_euler(matrix::Quatf q)
{
    float qw = q(0);
    float qx = q(1);
    float qy = q(2);
    float qz = q(3);
//    float x_phi = atan2(2*qy*qw-2*qx*qz , 1 - 2*(qy*qy + qz*qz));
//    float y_theta = asin(2*(qx*qy + qz*qw));
    float z_psi = atan2(2*(qx*qw-qy*qz), 1 - 2*(qx*qx + qz*qz));
//    matrix::Vector3f euler(x_phi, y_theta, z_psi);
    return z_psi;
}


float motor_angle_cal(uint32_t pwm_now, uint32_t pwm_max, uint32_t pwm_min, float angle_bias)
{
    pwm_cap.width_range = abs(pwm_cap.width_max - pwm_cap.width_min);
    float angle = float(pwm_now - pwm_min) / float(pwm_cap.width_range) * 360.0f;
    if(angle > 360)
        angle = 360;
    else if(angle < 0)
        angle = 0;
    //reverse the angle direction
    //let the angle increases when clockwise rotation
    angle = 360 - angle;
    //compensate the angle bias
    angle -= angle_bias;
    //remap to 0~360 deg
    if(angle < 0) angle += 360;
    return angle;
}

float motor_angle_cal_raw(uint32_t pwm_now, uint32_t pwm_max, uint32_t pwm_min)
{
    pwm_cap.width_range = abs(pwm_cap.width_max - pwm_cap.width_min);
    float angle = float(pwm_now - pwm_min) / float(pwm_cap.width_range) * 360.0f;
    if(angle > 360)
        angle = 360;
    else if(angle < 0)
        angle = 0;
    //reverse the angle direction
    //let the angle increases when clockwise rotation
    angle = 360 - angle;
    return angle;
}

float motor_speed_cal(float angle_now, float angle_last, float dt, float dt_max, float dt_min)
{
    static float updated_speed = 0;
    uint8_t speed_update_flag = 0;
    float delta_angle = angle_now - angle_last;
    //Handle the angle step from 360 to 0
#ifdef ROTATION_ANTI_CLOCKWISE
    //angular speed is negative
    //Don't update speed when angle step
    if(delta_angle > 0)
    {
        delta_angle -= 360;
        speed_update_flag = 1; //when angle step, don't update speed
    }
#else
    //angular speed is positive
    //Don't update speed when angle step
    if(delta_angle < 0)
    {
        delta_angle += 360;
        speed_update_flag = 1;
    }
#endif

    //Don't update speed if dt is too large
    if (dt > dt_max || dt < dt_min)
    {
        speed_update_flag = 2;
    }

    //Don't update speed when delta_angle is extravagantly large
    if(delta_angle > 100) //100/(1/910)/360*60 = 15166rpm
    {
        speed_update_flag = 3;
    }

    //update speed
    if(speed_update_flag == 0)
    {
        float new_speed = delta_angle / dt / 6; // /360*60=/6
        //fpass = 10Hz, Fs = 833.33Hz
//        updated_speed = new_speed;
//        updated_speed = 0.9274 * updated_speed + 0.0726 * new_speed;
        updated_speed = iirfilter(new_speed);
    }
    return updated_speed;
}


void inline update_orb_topic()
{
    // init
    if (orb_subscribe_flag == false)
    {
        px4_clock_gettime(CLOCK_REALTIME, &ts);
        time_cal.update(ts);
        pwm_cap.width_range = abs(pwm_cap.width_max - pwm_cap.width_min);
        orb_subscribe_flag = true;
    }

    //for debug
//    static uint64_t frame_counter = 0;
//    debug_value_data.timestamp = hrt_absolute_time();
//    debug_value_data.ind = 199;
//    debug_value_data.value = (float) frame_counter++;
//    debug_value_pub.publish(debug_value_data);
//    usleep(1000);//tested: dshot module running at 1700Hz

    if(actuator_controls_sub.update(&actuator_controls_data))//rate controller->dshot->esc
    {
        single_axis.actuator_ctrls_roll = actuator_controls_data.control[0];
        single_axis.actuator_ctrls_pitch = actuator_controls_data.control[1];
        single_axis.actuator_ctrls_yaw = actuator_controls_data.control[2];
        single_axis.actuator_ctrls_thrust = actuator_controls_data.control[3];
        single_axis.actuator_ctrls_amp =
                (float)sqrt(single_axis.actuator_ctrls_roll * single_axis.actuator_ctrls_roll +
                            single_axis.actuator_ctrls_pitch * single_axis.actuator_ctrls_pitch);
        single_axis.actuator_ctrls_pha = (float)atan2(single_axis.actuator_ctrls_pitch, single_axis.actuator_ctrls_roll);
    }


    if(rc_channels_sub.update(&rc_data))
    {
        rc.RX = (double)rc_data.channels[0];
        rc.RY = (double)rc_data.channels[1];
        rc.LY = (double)rc_data.channels[2];//throttle, value: 0 ~ 1, in my rc
        rc.LX = (double)rc_data.channels[3];

        rc.switch_B = (int8_t)rc_data.channels[5];
        rc.switch_C = (int8_t)rc_data.channels[6];
        rc.switch_D = (int8_t)rc_data.channels[7];
        rc.switch_E = (int8_t)rc_data.channels[8];
        rc.switch_G = (int8_t)rc_data.channels[9];
        // mavlink_log_info(&mavlink_log_pub, "throttle_out = %f", LY);
        if(rc.LY < 0) rc.LY = 0.0;
        speed_ref = rc.LY * 2047;
#ifdef USE_RC_RIGHT_STICK
        rc.R_amp = (float)sqrt(rc.RX * rc.RX + rc.RY * rc.RY);
        rc.R_pha_rad = (float)atan2(rc.RY, rc.RX);
#else
        rc.R_amp = 0;
        rc.R_pha_rad = 0;
#endif
        if (rc.switch_E == 0)
        {
            if(rc.switch_G == 0)
            {rc.R_amp = 0.0; rc.R_pha_rad = 0;}
            else if(rc.switch_G == 1)
            {rc.R_amp = 0.05; rc.R_pha_rad = 0;}
            else if(rc.switch_G == -1)
            {rc.R_amp = 0.1; rc.R_pha_rad = 0;}
        }
        if (rc.switch_E == 1)
        {
            if(rc.switch_G == 0)
            {rc.R_amp = 0.15; rc.R_pha_rad = 0;}
            else if(rc.switch_G == 1)
            {rc.R_amp = 0.2; rc.R_pha_rad = 0;}
            else if(rc.switch_G == -1)
            {rc.R_amp = 0.25; rc.R_pha_rad = 0;}
        }
        else if (rc.switch_E == -1)
        {
            if(rc.switch_G == 0)
            {rc.R_amp = 0.3; rc.R_pha_rad = 0;}
            else if(rc.switch_G == 1)
            {rc.R_amp = 0.35; rc.R_pha_rad = 0;}
            else if(rc.switch_G == -1)
            {rc.R_amp = 0.4; rc.R_pha_rad = 0;}
        }

        //if comment, switch B is used for kill
//        if(rc.switch_B == 1)
//            speed_ref = RC_SWITCH_THROTTLE;
    }

    if(battery_status_sub.update(&battery_status_data))
    {
        single_axis.bat_vol_now = battery_status_data.voltage_filtered_v;
        float bat_vol_per_cell = single_axis.bat_vol_now / CONST_BAT_CELL_NUM;
        if (bat_vol_per_cell >= 3.1 && bat_vol_per_cell <= 4.2)
            single_axis.bat_vol_compen_coeff = CONST_BAT_VOLT_FULL / bat_vol_per_cell;

        if (single_axis.bat_vol_compen_coeff < 1 || single_axis.bat_vol_compen_coeff > 1.3)
            single_axis.bat_vol_compen_coeff = 1.0;
    }

    //update control mode
    vehicle_control_mode_sub.update(&vehicle_control_mode_data);
}

void motor_status_multi_logging(bool on_flag)
{
    frame_counter++;
    if(!on_flag)
    {
        frame_counter = 0;
    }

    motor_state.timestamp = hrt_absolute_time();
    motor_state_log[log_index] = motor_state;

    // for array log
    int8_t temp_index = log_index;
    for(uint8_t i = 0; i < LOG_ONCE_LEN; i++)
    {
        status_for_log.angle_deg[i] = motor_state_log[temp_index].pulse_angle_deg;
        status_for_log.speed_rpm[i] = motor_state_log[temp_index].pulse_speed_rpm;
        status_for_log.speed_rpm[i] = motor_state_log[temp_index].pulse_speed_rpm;
        status_for_log.throttle_sin[i] = motor_state_log[temp_index].throttle_sin;
        status_for_log.dt[i] = motor_state_log[temp_index].dt;
        status_for_log.throttle_out[i] = motor_state_log[temp_index].throttle_out;
        status_for_log.timestamp_array[i] = motor_state_log[temp_index].timestamp;
        status_for_log.ref_x[i] = motor_state_log[temp_index].ref_x;
        status_for_log.ref_y[i] = motor_state_log[temp_index].ref_y;
//        status_for_log.ref_y[i] = single_axis.bat_vol_compen_coeff;
        status_for_log.sin_amp[i] = motor_state_log[temp_index].sin_amp;
        status_for_log.sin_pha[i] = motor_state_log[temp_index].sin_pha;
        temp_index--;
        if(temp_index < 0) temp_index = LOG_ONCE_LEN - 1;
    }
    status_for_log.timestamp = hrt_absolute_time();
    status_for_log.frame_counter = frame_counter;
    motor_status_pub.publish(status_for_log);

    log_index++;
    if(log_index == LOG_ONCE_LEN) log_index = 0;
}

void motor_status_one_logging(bool on_flag)
{
    frame_counter++;
    if(!on_flag)
    {
        frame_counter = 0;
    }

    status_one_for_log.angle_deg = motor_state.pulse_angle_deg;
    status_one_for_log.speed_rpm = motor_state.pulse_speed_rpm;
    status_one_for_log.throttle_sin = motor_state.throttle_sin;
    status_one_for_log.throttle_dc = motor_state.throttle_dc;
    status_one_for_log.throttle_out = motor_state.throttle_out;
    status_one_for_log.dt = motor_state.dt;
    status_one_for_log.sin_amp = motor_state.sin_amp;
    status_one_for_log.sin_pha = motor_state.sin_pha;
    status_one_for_log.bat_coef = motor_state.bat_coef;

    status_one_for_log.timestamp = hrt_absolute_time();
    status_one_for_log.frame_counter = frame_counter;
    motor_status_one_pub.publish(status_one_for_log);
}

void get_motor_angle_and_speed(void)
{
    pwm_cap.pulse_width_last = pwm_cap.pulse_width;
    pwm_cap.pulse_width = pwm_input_data.pulse_width;
    pwm_cap.period = pwm_input_data.period;
    pwm_cap.timestamp_curr = pwm_input_data.timestamp;
    pwm_cap.dt = ((float)(pwm_cap.timestamp_curr - pwm_cap.timestamp_last)) / 1000000.0; //us to s
    pwm_cap.timestamp_last = pwm_cap.timestamp_curr;
    //find the range automatically
//        if(width_max < pulse_width) width_max = pulse_width;
//        if(width_min > pulse_width) width_min = pulse_width;

    motor_state.pulse_angle_deg = motor_angle_cal(pwm_cap.pulse_width,
                                                  pwm_cap.width_max, pwm_cap.width_min, SENSOR_ROTOR_ANGLE_BIAS);

    motor_state.pulse_angle_deg_delta = motor_state.pulse_angle_deg - motor_state.pulse_angle_deg_last;
    motor_state.dt = pwm_cap.dt;

    //calculate the motor speed
    //avoiding the edge of pwm range, as the sensor has nonlinearity
    if (abs(pwm_cap.pulse_width - SENDOR_PWM_MAX) > MOTOR_SPEED_PWM_DEAD_ZONE &&
        abs(pwm_cap.pulse_width - SENDOR_PWM_MIN) > MOTOR_SPEED_PWM_DEAD_ZONE &&
        abs(pwm_cap.pulse_width_last - SENDOR_PWM_MIN) > MOTOR_SPEED_PWM_DEAD_ZONE &&
        abs(pwm_cap.pulse_width_last - SENDOR_PWM_MIN) > MOTOR_SPEED_PWM_DEAD_ZONE)
    {
        motor_state.pulse_speed_rpm = motor_speed_cal(motor_state.pulse_angle_deg,
                                                      motor_state.pulse_angle_deg_last,
                                                      motor_state.dt,
                                                      MOTOR_SPEED_LARGE_DT,
                                                      MOTOR_SPEED_SMALL_DT);
    }
    motor_state.pulse_angle_deg_last = motor_state.pulse_angle_deg;
}


void mix_throttle(double sin_amp, double sin_pha, double throttle_common)
{
    //begin, calculate the sin_amp and sin_pha
    double output_margin = THROTTLE_MAX - throttle_common;//use maximum throttle
    if(sin_amp > throttle_common) sin_amp = throttle_common;
    if(sin_amp > output_margin)     sin_amp = output_margin;

    //sin amplitude limitation
    sin_amp = math::constrain(sin_amp, (double)0, (double)CONST_SIN_AMP_LIMIT);

    if (abs(sin_amp) < SWASH_SIN_DEADZONE)
    {
        sin_amp = 0;
        sin_pha = 0;
    }
    //end, calculate the sin_amp and sin_pha

    //begin, mix the sin_amp, sin_pha and obtain throttle_sin
    motor_state.sin_amp = sin_amp;//sin amplitude
    motor_state.sin_pha = sin_pha;//sin phase
    motor_state.throttle_dc = (float)throttle_common;
    float motor_angle_rad = motor_state.pulse_angle_deg / deg_to_rad;//motor angular position in rad
    motor_state.throttle_sin = motor_state.sin_amp * cos(motor_angle_rad - motor_state.sin_pha + MOTOR_DELAY_ANLGE_BIAS);

#ifdef USING_SQUARE_WAVE
    if (motor_state.throttle_sin >= 0)
        motor_state.throttle_sin = sin_amp;
    else
        motor_state.throttle_sin = -sin_amp;
#endif
    //end, mix the sin_amp, sin_pha and obtain throttle_sin

    //begin, mix the throttle_common and throttle_sin
    motor_state.throttle_out = (uint16_t)(throttle_common + motor_state.throttle_sin);
    motor_state.throttle_out = math::constrain(motor_state.throttle_out, (uint16_t)THROTTLE_MIN, (uint16_t)THROTTLE_MAX);
    //end, mix the throttle_common and throttle_sin
}


uint16_t mixer_pulsar(bool on_flag)
{
    if(pwm_input_sub.update(&pwm_input_data))
    {
        get_motor_angle_and_speed();//the angle and speed will be obtained here

        //begin, get controller_output, the throttle
        /***** using auto throttle is dangerous *****/
        double throttle_common;
        if (!vehicle_control_mode_data.flag_control_position_enabled)//not in position mode
        {
            //throttle come from rc
            throttle_common = speed_ref;
        }
        else
        {
            //throttle come from position controller
            throttle_common = single_axis.actuator_ctrls_thrust * ACTUATOR_CONTROLS_TO_DSHOT;
        }
        throttle_common = math::constrain(throttle_common, (double)0, (double)CONTROL_MAX_THROTTLE);
        //end, get controller_output, the throttle

        float sin_amp = single_axis.actuator_ctrls_amp * CONST_MOMENT_TO_SIN_AMP * single_axis.bat_vol_compen_coeff;
        float sin_pha = single_axis.actuator_ctrls_pha;
        mix_throttle(sin_amp, sin_pha, throttle_common);

        #ifdef USE_LOG_ORB_MOTOR_STATUS_ME
        motor_state.bat_coef = single_axis.bat_vol_compen_coeff;//log for debug
        motor_status_one_logging(on_flag);
        #endif
    }
    return motor_state.throttle_out;
}



uint16_t throttle_rc(bool on_flag)
{
    if(pwm_input_sub.update(&pwm_input_data))
    {
        get_motor_angle_and_speed();

        double throttle_common = math::constrain(speed_ref, (double)0, (double)CONTROL_MAX_THROTTLE);
        double sin_amp = 0, sin_pha = 0;

        //use switch_D as dynamic sin_amp, chirp signal swap
        if(rc.switch_D == 1 && rc.switch_D != rc.switch_D_last)
        {
            inject_begin_timestamp = hrt_absolute_time();
        }
        rc.switch_D_last = rc.switch_D;

        if (rc.switch_D < 0)//rc directly control, switch to up
        {
            float pha_compensation = 0;
//            if (motor_state.pulse_speed_rpm > 2000)
//            {
//                pha_compensation = motor_state.pulse_speed_rpm * 0.0228 - 41.965;
//                pha_compensation = pha_compensation / 180 * const_pi;
//            }

            rc.R_pha_rad = 0;//for angle compan test;
            sin_pha = rc.R_pha_rad + pha_compensation;
            sin_amp = rc.R_amp * 1000;
        }
        else//swap
        {
            float t_interval = ((float)(hrt_absolute_time() - inject_begin_timestamp)) / 1000000.0;
            float freq = 0.2 * t_interval * t_interval; //square chirp
            float freq_end = 50; //Hz
            float swap_amp = 400;
            if(freq > freq_end)
            {
                freq = freq_end;
                force_ref_x = swap_amp * sin(2 * const_pi * freq * t_interval);
            }
            else
            {
                force_ref_x = swap_amp * sin(2 * const_pi / 3 * freq * t_interval);
            }
            force_ref_y = 0;
            force_ref_amp = (float)sqrt(force_ref_x * force_ref_x + force_ref_y * force_ref_y);
            force_ref_pha = (float)atan2(force_ref_y, force_ref_x);
            sin_amp = force_ref_amp;
            sin_pha = force_ref_pha;
            motor_state.ref_x = force_ref_x;
            motor_state.ref_y = force_ref_y;
        }

        mix_throttle(sin_amp, sin_pha, throttle_common);

        #ifdef USE_LOG_ORB_MOTOR_STATUS_ME
        motor_status_multi_logging(on_flag);
        #endif
    }
    return motor_state.throttle_out;
}



uint16_t speed_control_for_dshot(bool on_flag)
{
    update_orb_topic();
    return mixer_pulsar(on_flag); //for pulsar fly control
    //return throttle_rc(on_flag); //for force/torque experiment or motor test
}

