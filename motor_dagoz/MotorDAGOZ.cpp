#include "MotorDAGOZ.h"



MotorDagoz :: MotorDagoz (int period, PinName direction, PinName pwmpin) : _period(period), _direction(direction), _pwmpin(pwmpin)
{
    _pwmpin.period_us(_period);  
}

void MotorDagoz::setpwm(float inputpwm )
{
    if ( inputpwm >= 0 )
    {
        _direction = 1;
        _pwmpin.write(inputpwm);
    }
    else
    {
        _direction = 0;
        _pwmpin.write(-1*inputpwm);
    }   
}
