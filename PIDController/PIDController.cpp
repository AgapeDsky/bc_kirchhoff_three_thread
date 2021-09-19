#include "PIDController.h"


PIDController::PIDController()
{
    c1_ = 0.;
    c2_ = 0.;
    d0_ = 0.;
    d1_ = 0.;
    d2_ = 0.;
    prev_err_[0] = 0.;
    prev_err_[1] = 0.;
    prev_out_[0] = 0.;
    prev_out_[1] = 0.;
}

void PIDController::init(int mode, float kp, float ti, float td, float ff, float fc, float cp, bool is_active)
{
    double Kp = kp;

    double KiTs = Kp / ti * cp;
    c1_ = 1.;
    d0_ = Kp + KiTs;
    d1_ = -Kp;
    
    is_active_ = is_active;
}

double PIDController::clamp(double val, double min, double max)
{
    if (val >= max)
    {
        return max;
    }
    else if (val <= min)
    {
        return min;
    }
    else
    {
        return val;
    }
}
double PIDController::compute_action(double target, double feedback, float ff)
{
    double err = target - feedback;
    double out = 0.;
    
    if(is_active_)
    {
        out = c1_ * prev_out_[0] + c2_ * prev_out_[1] + d0_ * err + d1_ * prev_err_[0] + d2_ * prev_err_[1];
        out = clamp(out, -1., 1.);
    }

    prev_err_[1] = prev_err_[0];
    prev_err_[0] = err;
    prev_out_[1] = prev_out_[0];
    prev_out_[0] = out;

    return clamp(out + ff * target, -1., 1.);
}

void PIDController::setActive(bool command_active) {
    is_active_ = command_active;
}