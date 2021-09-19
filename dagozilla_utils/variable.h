#ifndef VARIABLE_H
#define VARIABLE_H

#include "mbed.h"
#include <ros/time.h>

/*****************************
        Global Variable
 *****************************/
// PID Controller Config
// PID Mode is PI
int PIDModeFR = 1;
int PIDModeFL = 1;
int PIDModeBR = 1;
int PIDModeBL = 1;
float cp = 0.005;
float kpFR = 1;
float kpFL = 1;
float kpBR = 1;
float kpBL = 1;
float tiFR = 1;
float tiFL = 1;
float tiBR = 1;
float tiBL = 1;
float tdFR = 0;
float tdFL = 0;
float tdBR = 0;
float tdBL = 0;
float ffFR = 0;
float ffFL = 0;
float ffBR = 0;
float ffBL = 0;
float fcFR = 0;
float fcFL = 0;
float fcBR = 0;
float fcBL = 0;

// Motor Pulse Accumulation
int32_t motorPulseBL = 0;
int32_t motorPulseFL = 0;
int32_t motorPulseFR = 0;
int32_t motorPulseBR = 0;

//Kicker global variable
double kick_power_target = 0;
bool kicker_shoot_mode = 0;
float range = 0.0011;
float position = 0.03;
float ball_distance = 0;

//For initiate encoder value
float cur_locomotion_L = 0;
float cur_locomotion_R = 0;
float cur_locomotion_B = 0;
float cur_dribbler_L = 0;
float cur_dribbler_R = 0;

//Potentio value
float cur_pot_L = 0;
float cur_pot_R = 0;

//pwm value
double locomotion_FL_target_rate = 0;
double locomotion_FR_target_rate = 0;
double locomotion_BL_target_rate = 0;
double locomotion_BR_target_rate = 0;
double dribbler_L_target_rate = 0;
double dribbler_R_target_rate = 0;

// Target Velocity & Feedback Velocity
double locomotion_FL_target_vel = 0;
double locomotion_FR_target_vel = 0;
double locomotion_BL_target_vel = 0;
double locomotion_BR_target_vel = 0;

double locomotion_FL_vel = 0;
double locomotion_FR_vel = 0;
double locomotion_BL_vel = 0;
double locomotion_BR_vel = 0;

//extended variable
unsigned long last_timer;
int32_t rotInFL = 0;
int32_t rotInFR = 0;
int32_t rotInBL = 0;
int32_t rotInBR = 0;

Timer t;

//Odometry
float theta_com;
float theta_result = 0;
float theta_prev;

#endif