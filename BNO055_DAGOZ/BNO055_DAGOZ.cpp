#include "BNO055_DAGOZ.h"

//Create BNO055 INSTANCE
BNO055_DAGOZ::BNO055_DAGOZ(PinName SDA, PinName SCL, PinName RESET) : _i2c(SDA,SCL), _reset(RESET){
    //Set I2C fast and bring reset line high
    _i2c.frequency(400000);
    }

/**
 * Checks that there are no errors on the accelerometer
 */
 
void BNO055_DAGOZ::reset(){
    sysReset();
    _reset = 0;
    Thread::wait(100);
    _reset = 1;
    Thread::wait(100);
    _reset = 0;
} 
void BNO055_DAGOZ::sysReset(){
    
//Perform a power-on-reset
    readReg(BNO055_SYS_TRIGGER_ADDR);
    regVal = regVal | 0x20;
    //writeReg(BNO055_SYS_TRIGGER_ADDR,0x20);
////Thread::wait for the system to come back up again (datasheet says 650ms)
//    Thread::wait(700);
//    writeReg(BNO055_SYS_TRIGGER_ADDR,0x00);

}
bool BNO055_DAGOZ::bno055Healthy()
{
    int sys_error = readReg(BNO055_SYS_ERR_ADDR);
    Thread::wait(1);
    int sys_stat = readReg(BNO055_SYS_STAT_ADDR);
    Thread::wait(1);
    
    if(sys_error == 0 && sys_stat == 5)
        return true;
    else
        return false;
}
    
 
/**
 * Configure and initialize the BNO055
 */
bool BNO055_DAGOZ::check()
{
    bool startupPass = true;
    // Do some basic power-up test
    regVal = readReg(BNO055_ID_ADDR);
    //printf("\nID : %d\n",regVal);
    if(regVal != BNO055_ID_VAL) return false;
    
    Thread::wait(50);
    regVal = readReg(BNO055_TEMP_ADDR); 
    //printf("TEMP : %d\n",regVal);
    if(regVal == 0) startupPass = false;
    
    //printf("startup : %d\n",startupPass);
    return startupPass;
}    
bool BNO055_DAGOZ::init()
{
    bool startupPass = true;
    startupPass = check();
    //sysReset();
    printf("startup : %d\n",startupPass);
    // Change mode to CONFIG
    writeReg(BNO055_OPR_MODE_ADDR, 0x00);
    //wait(0.2);
    Thread::wait(100);
    
    regVal = readReg(BNO055_OPR_MODE_ADDR);
    Thread::wait(100);
    
    // Remap axes
    writeReg(BNO055_AXIS_MAP_CONFIG_ADDR, 0x06);    // b00_00_01_10
    Thread::wait(100);    
 
    // Set to external crystal
    //writeReg(BNO055_SYS_TRIGGER_ADDR, 0x80);
    //Thread::wait(100);    
    
    // change mode to Compass
    writeReg(BNO055_OPR_MODE_ADDR, 0x09);
    Thread::wait(100);
    // Change mode to NDOF
//    writeReg(BNO055_OPR_MODE_ADDR, 0x0C);
//    Thread::wait(0.2);
 
    regVal = readReg(BNO055_OPR_MODE_ADDR);
    Thread::wait(100);
    return startupPass;
}

 
/**
 * Returns the calibration status of each component
 */
CalibStatus BNO055_DAGOZ::readCalibrationStatus()
{
    int regVal = readReg(BNO055_CALIB_STAT_ADDR);
        
    _status.mag = regVal & 0x03;
    _status.acc = (regVal >> 2) & 0x03;
    _status.gyr = (regVal >> 4) & 0x03;
    _status.sys = (regVal >> 6) & 0x03;
    
    return _status;
}
  
/**
 * Reads the Euler angles, zeroed out
 */
Euler BNO055_DAGOZ::getEulerAngles()
{
    char buf[16];
    Euler e;
    
    // Read in the Euler angles
    buf[0] = BNO055_EULER_H_LSB_ADDR;
    _i2c.write(BNO055_ADDR, buf, 1, false);
    _i2c.read(BNO055_ADDR, buf, 6, false);
    
    short int euler_head = buf[0] | (buf[1] << 8);
    short int euler_roll = buf[2] | (buf[3] << 8);
    short int euler_pitch = buf[4] | (buf[5] << 8);
    
    e.heading = ((float)euler_head) / 16.0;
    e.roll = ((float)euler_roll) / 16.0;
    e.pitch = ((float)euler_pitch) / 16.0;
    
    return e;
}

float BNO055_DAGOZ::getThetaDegree(float theta0){
      Euler e = getEulerAngles();
      float headingTemp = e.heading - theta0;
      float heading = headingTemp;
      
      if (headingTemp >= 0 ) {
            if(headingTemp > 180 && headingTemp <360){
                return(headingTemp - 360.0);
            }
            else {
                return headingTemp;
            }  
       }
       else {
            //invert headingTemp
            headingTemp = -headingTemp;
            if(headingTemp > 180 && headingTemp <360){
                return(360.0-headingTemp);
            }
            else {
                return (-headingTemp);
            }  
       }
      
}