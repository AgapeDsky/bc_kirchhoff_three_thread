#ifndef ROBOTPIN_H
#define ROBOTPIN_H

#include "BNO055_DAGOZ.h"
#include "MotorDAGOZ.h"
#include "EncoderMotor.h"
#include "EncoderDAGOZ.h"
#include "Servo.h"
/*****************************
        Pin Declaration
 *****************************/
//Encoder pin

EncoderMotor intEncFL(PG_8, PE_0, 364.0, EncoderMotor::X4_ENCODING);
EncoderMotor intEncFR(PD_10, PG_14, 364.0, EncoderMotor::X4_ENCODING);
EncoderMotor intEncBL(PF_2, PF_1, 364.0, EncoderMotor::X4_ENCODING);
EncoderMotor intEncBR(PB_7, PC_13, 364.0, EncoderMotor::X4_ENCODING);

EncoderDAGOZ locomotionEncL(TIM2);
EncoderDAGOZ locomotionEncR(TIM1);
EncoderDAGOZ locomotionEncB(TIM3);

//Motor pin buat Board Sistem next ver.
MotorDagoz locomotionMotorFL(LOCOMOTION_PWM_PERIOD_US, PF_11, PF_7); //Locomotion Front Left Motor
MotorDagoz locomotionMotorFR(LOCOMOTION_PWM_PERIOD_US, PF_12, PF_8); //Locomotion Front Right Motor
MotorDagoz locomotionMotorBL(LOCOMOTION_PWM_PERIOD_US, PG_1, PF_9);  //Locomotion Back Left Motor
MotorDagoz locomotionMotorBR(LOCOMOTION_PWM_PERIOD_US, PC_2, PD_15); //Locomotion Back Right Motor

MotorDagoz dribblerMotorR(DRIBBLER_PWM_PERIOD_US, PF_15, PE_6); //Dribbler Right Motor
MotorDagoz dribblerMotorL(DRIBBLER_PWM_PERIOD_US, PE_15, PE_5); //Dribbler Left Motor

//Serial pin
Serial pc(USBTX, USBRX, 115200); //Serial debug

//Compass BNO055 pin
BNO055_DAGOZ compass(PB_9, PB_8, PC_5); //Compass I2C Communication SDA SCL

//Potensio Pin
AnalogIn dribblerPotR(PF_6); //Potensio for Left Dribbler, di board saat ini masih PC_2
AnalogIn dribblerPotL(PC_3); //Potensio for Right Dribbler
AnalogIn infraRed(PF_5);     //Potensio for Kicker, di board saat ini masih PD_4

//LED Pin
DigitalOut ledRed(PG_4);   //Red LED
DigitalOut ledGreen(PG_5); //Green LED
DigitalOut ledBlue(PG_6);  //Blue LED

//Kicker Pin
PwmOut kicker(PB_15); //Kicker pwm effort, di board saat ini masih PC_8

//Servo Pin
Servo kickerServo(PC_7);

#endif