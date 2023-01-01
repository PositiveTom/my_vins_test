#include "loss_function.h"


void HuberLoss::Compute(double err2, Eigen::Vector3d& rho) const
{
    double delta_2 = delta_ * delta_;
    if(err2 <= delta_2)
    {
        //  rho
        rho[0] = err2;
        //  rho'
        rho[1] = 1;
        //  rho''
        rho[2] = 0;
    }
    else
    {
        double err = sqrt(err2);
        //  rho
        rho[0] = 2*delta_*err - delta_2;
        //  rho'
        rho[1] = delta_ / err;
        //  rho''
        rho[2] = -0.5 * rho[1] / err2;
    }
}
