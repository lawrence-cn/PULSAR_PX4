# PULSAR (powered-flying ultra-underactuated LiDAR sensing aerial robot)

## A self-rotating, single-actuated UAV with extended sensor field of view for autonomous navigation

## PX4 firmware of PULSAR

### Hardware configuration
+ Power the magnetic encoder and config it using a I2C bus, and make sure the 910-Hz PWM signal is outputed (e.g., verified by a oscilloscope).
+ Connect the power port of magnetic encoder to a 5V port of the Pixhawk 4 Mini.
+ Connect the PWM signal of magnetic encoder to the CAP1 port of the Pixhawk 4 Mini.
+ Connect the ESC (off-the-shelf ESC is ok as long as it supports dshot protocol) to the MAIN OUT 1 port of the Pixhawk 4 Mini.

### Software configuration
+ Clone the repository
+ Config your compile environment https://docs.px4.io/main/en/dev_setup/dev_env.html
+ Build the code and upload to Pixhawk 4 mini through USB connection
```
cd 3-PX4_Firmware/PULSAR_PX4
make px4_fmu-v5_default upload
```
+ Load the parameter file "PULSAR_PX4_parameters.params" to the Pixhawk 4 mini by QGroundControl or other tools (very important)
+ Keeping the USB connection, in the MAVLINK Console of QGroundControl, type
```
px4_cn start
```
+ If the encoder works well, two rotor angles (angle_compensated and angle_raw) will be printed in the MAVLINK terminal. Rotate the positive blade of the swashplateless mechanism to align with the negative direction of the body y axis of PULSAR (facing to LiDAR's front side, right hand side is the nevative y axis), and record the value of the printed "angle_raw". 
+ Modified the MACRO (SENSOR_ROTOR_ANGLE_BIAS) with the recorded value (see 3.2.2).
+ Compile the code and upload again.
+ Keeping the same blade's position and starting the angle print in MAVLINK again, make sure the "angle_compensated" is close to 0 deg or 360 deg.
+ To stop the print, type
```
px4_cn stop
```

### Main modifications of the PX4 firmware

#### PWM capture driver for magnetic encoder 
The files are located in "src/drivers/pwm_input". Please make sure that the encoder 910-Hz PWM signal is connected to the CAP1 of the Pixhawk 4 Mini. Changing the frequency should also modify the timer frequency in "PULSAR_PX4/src/drivers/pwm_input/pwm_input.cpp".

#### Mixer of PULSAR
The mixer files are located in the "PULSAR_PX4/src/drivers/dshot/swashplateless".

SENSOR_ROTOR_ANGLE_BIAS (unit is degree) is used for compensating the encoder installation offset angle and it must be set properly.

MOTOR_DELAY_ANLGE_BIAS (unit is radian) is used for compensating the lag angle of the swashplateless mechanism. This value is different when using different motor or propeller and can be determined on a test stand with 6-axis force sensor by analyzing the moment data. If you use the propulsion system same as PULSAR (i.e., T-MOTOR MN5006 KV450 and MF1302 propeller), you don't need to change this value.

The data of swashplateless mechanism are published for logging.


#### The repalcement of throttle output
The repalcement is conducted in "PULSAR_PX4/src/drivers/dshot/dshot.cpp".
Please make sure that the ESC signal wire is connected to the MAIN OUT1 of the Pixhawk 4 Mini. The dshot has been enabled if you had loaded the parameter file into the Pixhawk 4 mini.


#### The attitude controller of PULSAR
The attitude controller is implemented in "PULSAR_PX4/src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp".
The attitude setpoint and feedback are published for data logging.


#### The angular velocity controller of PULSAR
The angular velocity controller is implemented in "PULSAR_PX4/src/modules/sa_rate_control_me/sa_rate_control_me_main.cpp".
The angular velocity setpoint and feedback both in world frame are published for data logging.


#### The position controller of PULSAR
There are some modifications in "PULSAR_PX4/src/modules/mc_pos_control" and "PULSAR_PX4/src/lib/flight_tasks".
This ensure the position command from remote controller (RC) is available in world frame rather than in body frame.

#### Logging topics
Some ulog topics are added in "PULSAR_PX4/src/modules/logger/logged_topics.cpp" and "PULSAR_PX4/msg".

