#ifndef LOSS_FUNCTION_H
#define LOSS_FUNCTION_H

#include <iostream>
#include <Eigen/Eigen>

using namespace std;

class LossFunction
{   
public:
    //  计算 rho(s)
    virtual void Compute(double err2, Eigen::Vector3d& rho) const = 0;

};

//  huber 核
//  huber(e) = e^2               if e<=delta
//  huber(e) = delta*(2*e-delta) if e>delta
class HuberLoss:public LossFunction
{
public:
    explicit HuberLoss(double delta):delta_(delta){}
    virtual void Compute(double err2, Eigen::Vector3d& rho) const;
private:
    double delta_;
};


#endif
