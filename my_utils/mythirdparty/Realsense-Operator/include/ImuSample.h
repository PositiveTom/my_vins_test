#ifndef IMUSAMPLE_H_
#define IMUSAMPLE_H_

#include <ros/ros.h>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
// #include <boost.h>

using namespace std;

class mySampleRealsense
{
public:
    mySampleRealsense();
    mySampleRealsense(bool call, ros::NodeHandle& n);
    // ~mySampleRealsense();
    static void fromMsec(uint32_t& sec, uint32_t& nsec, double t)
    {
        // [1]. 把double类型时间转为有符号64整型
        int64_t t_int64 = (int64_t) floor(t);
        // [2]. 从毫秒中提取出s
        sec = (uint32_t) (t_int64 / 1000);
        //  [3]. 计算出ns
        nsec = (uint32_t) ((t - sec*1e3) * 1e6);
    }

    bool SampleImu(sensor_msgs::ImuPtr imu);

    bool SamplePhoto(sensor_msgs::ImagePtr img);

    static void streamCallback(const rs2::frame& frame);

    void GetExtrinsics();

    void GetIntrinsics();
    
private:
    rs2::pipeline* pipe;
    rs2::config* cfg;
    rs2::pipeline_profile pipe_profile;
};



#endif