/**
 * adopted from
 * BNO055 example by: Simon Scott
 * 
 * Bismillahirahmanirrahim
 * created by : Dagozilla :: rizkyardimaulana@gmail.com 
 */


#ifndef BNO055_DAGOZ_H
#define BNO055_DAGOZ_H

/* NECESSARY REGISTER */
#define BNO055_ADDR                                             0x52 // 0x29 << 1
#define BNO055_ID_VAL                                           0xA0 
#define BNO055_ID_ADDR                                          0x00
#define BNO055_EULER_H_LSB_ADDR                                 0x1A
#define BNO055_TEMP_ADDR                                        0x34
#define BNO055_OPR_MODE_ADDR                                    0x3D
#define BNO055_CALIB_STAT_ADDR                                  0x35
#define BNO055_SYS_STAT_ADDR                                    0x39
#define BNO055_SYS_ERR_ADDR                                     0x3A
#define BNO055_AXIS_MAP_CONFIG_ADDR                             0x41
#define BNO055_SYS_TRIGGER_ADDR                                 0x3F

/* STRUCT DATA */

//Calibration Status
typedef struct CalibStatus_t
{
    int mag;
    int acc;
    int gyr;
    int sys;
} CalibStatus;

//Orientation 
typedef struct Euler_t
{
    float heading;
    float pitch;
    float roll;
} Euler;

/*INCLUDE*/

#include "mbed.h"

class BNO055_DAGOZ{
public :     
        DigitalOut _reset;
        //Constructor
        BNO055_DAGOZ(PinName SDA, PinName SCL, PinName RESET);
        //initialization mode and set mapping
        bool init(); 
        //Returns the calibration status of each component
        CalibStatus readCalibrationStatus();
        //Checks that there are no errors on the accelerometer
        bool bno055Healthy();
        //onfigure and initialize the BNO055
        bool check();
        //Reads the Euler angles, zeroed out
        Euler getEulerAngles();
        //get the theta or yaw in Degree
        float getThetaDegree(float theta0);
        //perform reset
        void reset();
        void sysReset();
        //current Euler
        Euler _euler;
        //current calibration status
        CalibStatus _status;
        
/**
 * Function to write to a single 8-bit register
 */
void writeReg(int regAddr, char value)
{
    char wbuf[2];
    wbuf[0] = regAddr;
    wbuf[1] = value;
    _i2c.write(BNO055_ADDR, wbuf, 2, false);  
}
 
/**
 * Function to read from a single 8-bit register
 */
char readReg(int regAddr)
{
    char rwbuf = regAddr;
    _i2c.write(BNO055_ADDR, &rwbuf, 1, false);
    _i2c.read(BNO055_ADDR, &rwbuf, 1, false);
//    pc.printf("\nID : %x",rwbuf);
    return rwbuf;
}
    private:
        char regVal;
        I2C _i2c;
};

#endif /* BNO055_DAGOZ_H */