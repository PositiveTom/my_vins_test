#ifndef MCAMERA_H
#define MCAMERA_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <Eigen/Eigen>

class mCamera
{
public:
    virtual void liftProjective(Eigen::Vector2d &uv, Eigen::Vector3d &xy) = 0;

};

#endif