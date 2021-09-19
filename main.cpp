#include "mbed.h"
#include "math.h"
#include <vector>
#include <ros.h>
#include <dagozilla_utils/config.h>
#include <dagozilla_utils/robotpin.h>
#include <dagozilla_utils/variable.h>
#include <dgz_msgs/StampedHardwareCommand.h>
#include <dgz_msgs/StampedHardwareState.h>
#include <std_srvs/SetBool.h>
#include "PIDController.h"

//#include "rtos.h"

// For PID Param Logging Purpose (sprintf)
// #include <string>

/*****************************
        ROS node handle 
 *****************************/
ros::NodeHandle nh;
/******************************
  Publisher-Subscriber
******************************/
dgz_msgs::StampedHardwareState stateMsg;

ros::Publisher statePub("/nucleo/state/hardware", &stateMsg);
void commandCallback(const dgz_msgs::StampedHardwareCommand &commandMsg);
ros::Subscriber<dgz_msgs::StampedHardwareCommand> commandSub("/control/command/hardware", &commandCallback);
//Semaphore xSemaphorePulseData(4);

/******************************
         Service
******************************/
//ros::ServiceClient<std_srvs::SetBool::Request, std_srvs::SetBool::Response> client("control/state/notify_kick");

//thread for RTOS
Thread thread1(osPriorityNormal1);
Thread thread2(osPriorityAboveNormal);
Thread thread3(osPriorityNormal);

// initialize controllers
PIDController ControllerFR = PIDController();
PIDController ControllerFL = PIDController();
PIDController ControllerBR = PIDController();
PIDController ControllerBL = PIDController();

//primitive function
void mainProcess();
void getCompass();
void controlCalculation();
void publishMessage();
void moveDribbler();
void initPIDController();
void assignPIDParam();
void setBaseActive(const std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

// 
ros::ServiceServer<std_srvs::SetBool::Request, std_srvs::SetBool::Response> set_base_active_srv("/nucleo/command/set_base_active", &setBaseActive);
/*****************************
  Main Function
 *****************************/

int main()
{
    //init ros advertise and subscribe
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    wait(10); // wait until nucleo node is set
    nh.advertise(statePub);
    nh.subscribe(commandSub);
    
    // Service Client for Kicker
//    nh.serviceClient(client);
    initPIDController();

    t.start();

    //kicker mode
    kickerServo.calibrate(range, 0.0);
    kickerServo = position;
    //kicker setup
    kicker.period(0.02f);
    kicker = 1; //deactivate kicker, active LOW

    thread1.start(mainProcess);
    thread2.start(getCompass);
    thread3.start(controlCalculation);

    while (true)
    {
        //do nothing
    }
}
void setBaseActive(const std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
    ControllerFL.setActive(req.data);
    ControllerFR.setActive(req.data);
    ControllerBL.setActive(req.data);
    ControllerBR.setActive(req.data);
    res.success = true;
    res.message = "success";
}
void initPIDController()
{
    assignPIDParam();
    ControllerFR.init(PIDModeFR, kpFR, tiFR, tdFR, ffFR, fcFR, cp, true);
    ControllerFL.init(PIDModeFL, kpFL, tiFL, tdFL, ffFL, fcFL, cp, true);
    ControllerBR.init(PIDModeBR, kpBR, tiBR, tdBR, ffBR, fcBR, cp, true);
    ControllerBL.init(PIDModeBL, kpBL, tiBL, tdBL, ffBL, fcBL, cp, true);
}
void assignPIDParam()
{
    
    if (!nh.getParam("/control/base/motors_pid/front_left/proportional", &kpFL, 1, 3000))
        nh.loginfo("fail hehe 1");

    //     else {
    //           char buf[255];
    //           sprintf(buf, "Hehe 1: %.4f", kpFL);
    //           nh.loginfo(buf);
    //       }

    if (!nh.getParam("/control/base/motors_pid/front_left/feedforward", &ffFL, 1, 3000))
        nh.loginfo("fail hehe 2");

    if (!nh.getParam("/control/base/motors_pid/front_left/integral_time", &tiFL, 1, 3000))
        nh.loginfo("fail hehe 3");

    if (!nh.getParam("/control/base/motors_pid/front_right/proportional", &kpFR, 1, 3000))
        nh.loginfo("fail hehe 5");

    if (!nh.getParam("/control/base/motors_pid/front_right/feedforward", &ffFR, 1, 3000))
        nh.loginfo("fail hehe 6");

    if (!nh.getParam("/control/base/motors_pid/front_right/integral_time", &tiFR, 1, 3000))
        nh.loginfo("fail hehe 7");

    if (!nh.getParam("/control/base/motors_pid/back_left/proportional", &kpBL, 1, 3000))
        nh.loginfo("fail hehe 9");

    if (!nh.getParam("/control/base/motors_pid/back_left/feedforward", &ffBL, 1, 3000))
        nh.loginfo("fail hehe 10");

    if (!nh.getParam("/control/base/motors_pid/back_left/integral_time", &tiBL, 1, 3000))
        nh.loginfo("fail hehe 11");

    if (!nh.getParam("/control/base/motors_pid/back_right/proportional", &kpBR, 1, 3000))
        nh.loginfo("fail hehe 13");

    if (!nh.getParam("/control/base/motors_pid/back_right/feedforward", &ffBR, 1, 3000))
        nh.loginfo("fail hehe 14");

    if (!nh.getParam("/control/base/motors_pid/back_right/integral_time", &tiBR, 1, 3000))
        nh.loginfo("fail hehe 15");
}

void mainProcess()
{
    float cur_pot_L0 = (float)dribblerPotL.read() * SCALE_POT_L;
    float cur_pot_R0 = (float)dribblerPotR.read() * SCALE_POT_R;
    Thread::wait(1000);

    while (1)
    {

        kick_power_target = 0;

        cur_pot_L = (float)dribblerPotL.read() * SCALE_POT_L - cur_pot_L0;
        cur_pot_R = (float)dribblerPotR.read() * SCALE_POT_R - cur_pot_R0;

        nh.spinOnce();

        //ball distance from IR
        ball_distance = infraRed.read();

        //read encoder
        cur_locomotion_R = -locomotionEncR.GetCounter(1);
        cur_locomotion_L = -locomotionEncL.GetCounter(1);
        cur_locomotion_B = -locomotionEncB.GetCounter(1);

        //cur_dribbler_L = -dribblerEncL.GetCounter(1);
        //cur_dribbler_R = -dribblerEncR.GetCounter(1);

        moveDribbler();
        
        // Development Cooldown Kicker
//              if (clock_ms()-time_last_kick > kicker_ready_time)
//        {
//            kicker.write(1-kick_power_target);
//            time_last_kick = clock_ms();
//            std_srvs::SetBool::Request req;
//            std_srvs::SetBool::Response res;
//            req.data = true;
//            client.call(req, res);
//        }
//        else {
//            kicker.write(1);
//        }
        kicker.write(1 - kick_power_target);

        publishMessage();

        if (t - last_timer >= 1000)
        {

            locomotion_FL_target_vel = 0;
            locomotion_FR_target_vel = 0;
            locomotion_BL_target_vel = 0;
            locomotion_BR_target_vel = 0;

            dribbler_L_target_rate = 0;
            dribbler_R_target_rate = 0;

            kick_power_target = 1;
            kicker_shoot_mode = 0;
        }

        Thread::wait(20);
    }
}

// PID Calculation to generate PWM
void controlCalculation()
{

    while (1)
    {
        // Get Internal Encoder Pulses
        rotInFL = intEncFL.getPulses(1);
        rotInFR = intEncFR.getPulses(1);
        rotInBL = intEncBL.getPulses(1);
        rotInBR = intEncBR.getPulses(1);

//        xSemaphorePulseData.wait();
        motorPulseFL += rotInFL;
        motorPulseFR += rotInFR;
        motorPulseBL += rotInBL;
        motorPulseBR += rotInBR;
//        xSemaphorePulseData.release();

        // Calculate Feedback Velocity from pulses
        locomotion_FL_vel = rotInFL * 2 * PI * WHEEL_RADIUS / (WHEEL_PPR * CONTROL_COMPUTE_PERIOD);
        locomotion_FR_vel = rotInFR * 2 * PI * WHEEL_RADIUS / (WHEEL_PPR * CONTROL_COMPUTE_PERIOD);
        locomotion_BL_vel = rotInBL * 2 * PI * WHEEL_RADIUS / (WHEEL_PPR * CONTROL_COMPUTE_PERIOD);
        locomotion_BR_vel = rotInBR * 2 * PI * WHEEL_RADIUS / (WHEEL_PPR * CONTROL_COMPUTE_PERIOD);

        // Compute action drom PIDController to determine PWM
        locomotion_FR_target_rate = ControllerFR.compute_action(locomotion_FR_target_vel, locomotion_FR_vel, ffFR);
        locomotion_FL_target_rate = ControllerFL.compute_action(locomotion_FL_target_vel, locomotion_FL_vel, ffFL);
        locomotion_BR_target_rate = ControllerBR.compute_action(locomotion_BR_target_vel, locomotion_BR_vel, ffBR);
        locomotion_BL_target_rate = ControllerBL.compute_action(locomotion_BL_target_vel, locomotion_BL_vel, ffBL);

        // Execute PWM value to certain pin
        locomotionMotorFL.setpwm(locomotion_FL_target_rate);
        locomotionMotorFR.setpwm(locomotion_FR_target_rate);
        locomotionMotorBL.setpwm(locomotion_BL_target_rate);
        locomotionMotorBR.setpwm(locomotion_BR_target_rate);

        Thread::wait(5);
    }
}

void moveDribbler()
{
    dribblerMotorL.setpwm(-dribbler_L_target_rate);
    dribblerMotorR.setpwm(dribbler_R_target_rate);
}

void moveLever()
{

    switch (kicker_shoot_mode)
    {
    case 0:
        position = 0.055;
        break;
    case 1:
        position = 0.1;
        break;
    }

    kickerServo.calibrate(range, 0.0);
    kickerServo.write(position);
}

void publishMessage()
{

    // Publish position data//
    stateMsg.data.base_motor_1_pulse_delta = motorPulseFL;
    stateMsg.data.base_motor_2_pulse_delta = motorPulseFR;
    stateMsg.data.base_motor_3_pulse_delta = motorPulseBL;
    stateMsg.data.base_motor_4_pulse_delta = motorPulseBR;

    stateMsg.data.base_encoder_1_pulse_delta = cur_locomotion_L;
    stateMsg.data.base_encoder_2_pulse_delta = cur_locomotion_R;
    stateMsg.data.base_encoder_3_pulse_delta = cur_locomotion_B;

    stateMsg.data.dribbler_motor_l_pulse_delta = cur_dribbler_L;
    stateMsg.data.dribbler_motor_r_pulse_delta = cur_dribbler_R;

    stateMsg.data.dribbler_potentio_l_reading = 0.0;
    stateMsg.data.dribbler_potentio_r_reading = 0.0;

    stateMsg.data.ir_reading = ball_distance;
    stateMsg.data.compass_reading = theta_result;

    stateMsg.header.stamp = nh.now();

    statePub.publish(&stateMsg);

    // Reset pulse delta value for distinguished control thread frequency
    //xSemaphorePulseData.wait();
    motorPulseFL = 0;
    motorPulseFR = 0;
    motorPulseBR = 0;
    motorPulseBL = 0;
    //xSemaphorePulseData.release();
}

void commandCallback(const dgz_msgs::StampedHardwareCommand &commandMsg)
{

    // Temporary, base_motor_x_pwm should be modified by new commandMsg (branch master)
    locomotion_FL_target_vel = commandMsg.data.base_motor_1_pwm;
    locomotion_FR_target_vel = commandMsg.data.base_motor_2_pwm;
    locomotion_BL_target_vel = commandMsg.data.base_motor_3_pwm;
    locomotion_BR_target_vel = commandMsg.data.base_motor_4_pwm;

    dribbler_L_target_rate = commandMsg.data.dribbler_motor_l_pwm;
    dribbler_R_target_rate = commandMsg.data.dribbler_motor_r_pwm;

    kick_power_target = commandMsg.data.kicker_pwm;
    kicker_shoot_mode = commandMsg.data.is_shoot;

    last_timer = t;
}

void getCompass()
{
    //printf("test\n");
    //check compass
    bool startupPassed = false;
    compass._reset = 0;
    while (!startupPassed)
    {
        compass.reset();
        startupPassed = compass.init();
        Thread::wait(100);
    }
    compass._reset = 1;
    //check compass healthy
    //startupPassed = false;

    //while(startupPassed == false){
    //        if (compass.bno055Healthy()){
    //            pc.printf("\nBNO055 is Healthy..");
    //            startupPassed = true;
    //        } else {
    //            pc.printf("\nBNO055 Has an Error..");
    //            //compassLed = 0;
    ////            wait(500);
    ////            compassLed = 1;
    //            compass.reset();
    //            compass.init();
    //        }
    //        Thread::wait(500);
    //    }
    //initialize compass
    // compass.init();
    Thread::wait(100);
    Euler e = compass.getEulerAngles();
    float theta0 = e.heading;
    Thread::wait(100);
    while (1)
    {

        //nh.spinOnce();

        if (compass.check())
        {
            theta_result = compass.getThetaDegree(theta0);
            //            pc.printf("Theta : %.2f\n", theta_result);
            compass._reset = 1;
        }
        else
        {
            compass.reset();
            //reinitializing
            compass.init();
            compass._reset = 0;
        }
        Thread::wait(50);
    }
}

/*
void mainProcess(){ //Kicker feedback
    bool kick_executed = 0;
    float cur_pot_L0 = (float)dribblerPotL.read() * SCALE_POT_L;
    float cur_pot_R0 = (float)dribblerPotR.read() * SCALE_POT_R;
    Thread::wait(1000);
    
    while(1){
        // Turn off kicker        
        kick_power_target = 0;
        kicker.write(1 - kick_power_target);
        
        cur_pot_L = (float)dribblerPotL.read() * SCALE_POT_L - cur_pot_L0;
        cur_pot_R = (float)dribblerPotR.read() * SCALE_POT_R - cur_pot_R0;
                
        nh.spinOnce();
        
        // Reset kick_executed once kick_power_target == 0
        if (kick_executed && kick_power_target == 0) {
            kick_executed = 0;   
        }
        
        //ball distance from IR
        ball_distance = infraRed.read();
        
        //read encoder
//        cur_locomotion_R = -locomotionEncR.GetCounter(1);
//        cur_locomotion_L = -locomotionEncL.GetCounter(1);
        
        rotInFL = intEncFL.getPulses(1);
        rotInFR = intEncFR.getPulses(1);
        rotInBL = intEncBL.getPulses(1);
        rotInBR = intEncBR.getPulses(1);
        
//        cur_dribbler_L = -dribblerEncL.GetCounter(1);
//        cur_dribbler_R = -dribblerEncR.GetCounter(1);
        
        moveLocomotion();
        moveDribbler();
        
        // If kick is not executed and kick_power_target > 0, kick
        if (kick_power_target > 0 && !kick_executed) {
            kicker.write(1-kick_power_target);
            
            kick_executed = 1;
        }
        
        publishMessage();
        
        if (t - last_timer >=1000){
            
            locomotion_FL_target_rate = 0;
            locomotion_FR_target_rate = 0;
            locomotion_BL_target_rate = 0;
            locomotion_BR_target_rate = 0;
            
            dribbler_L_target_rate = 0;
            dribbler_R_target_rate = 0;
            
            kick_power_target = 1;
            kicker_shoot_mode = 0;
        }
        
        Thread::wait(20);
    }
}
*/