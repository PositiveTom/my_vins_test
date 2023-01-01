#include <ros/ros.h>
#include "ImuSample.h"
#include <sensor_msgs/Image.h>
#include <memory>
#include <chrono>

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_sample_cam_imu");
    ros::NodeHandle n;
    shared_ptr<mySampleRealsense> Msr = make_shared<mySampleRealsense>(mySampleRealsense());
    // sensor_msgs::ImagePtr msgptr = boost::make_shared<sensor_msgs::Image>();
    // while(ros::ok())
    // {
    //     chrono::system_clock::time_point start = chrono::system_clock::now();
    //     Msr->SamplePhoto(msgptr);
    //     chrono::system_clock::time_point end = chrono::system_clock::now();
    //     chrono::duration<double> cost = chrono::duration_cast<chrono::duration<double>>(end-start);
    //     cout << "FPS:" << int(1.0/cost.count()) << endl;
    // }
    Msr->GetIntrinsics();
    ros::spin();
    return 0;
}