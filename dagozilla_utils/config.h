#ifndef CONFIG_H
#define CONFIG_H

/*****************************
        CONSTANTS
 *****************************/
// Robot Components
const double PI = 3.141593;             //PI constanta
const double RADTODEG = 57.2957795131;  //Radian to Deg const
const double SCALE_POT_L = -10.0;       //Left potentiometer scaling const
const double SCALE_POT_R = 10.0;        //Right potentiometer scaling const
const int LOCOMOTION_PWM_PERIOD_US = 125;
const int DRIBBLER_PWM_PERIOD_US = 2000;

const double WHEEL_RADIUS = 0.05;
const double WHEEL_PPR = 364;
const double CONTROL_COMPUTE_PERIOD = 0.005;

#endif