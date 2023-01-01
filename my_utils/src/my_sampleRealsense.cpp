#include "ImuSample.h"
#include <ros/ros.h>
#include <memory>
#include <chrono>
// #include <boost.h>

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_sampleRealsense");
    ros::NodeHandle n;
    shared_ptr<mySampleRealsense> Msr = make_shared<mySampleRealsense>(mySampleRealsense());
    sensor_msgs::ImuPtr imu = boost::make_shared<sensor_msgs::Imu>(sensor_msgs::Imu());
    imu->header.frame_id = "/imu";
    imu->header.seq = 0;
    ros::Publisher pub = n.advertise<sensor_msgs::Imu>("/my_imu", 1000);
    ros::Rate rate(200);
    bool flag;
    while(ros::ok())
    {
        flag = Msr->SampleImu(imu);
        if(flag)
            pub.publish(*imu);
        rate.sleep();
    }
    return 0;
}

        // chrono::steady_clock::time_point start = chrono::steady_clock::now();
        // ros::Duration(0.1).sleep();
        // chrono::steady_clock::time_point end = chrono::steady_clock::now();
        // chrono::duration<double> cost = chrono::duration_cast<chrono::duration<double>>(end-start);
        // cout << "FPS:" << int(1.0/cost.count()) << "hz" << endl;
        // cout << cost.count() << endl;

    //         double a_ms = 2331.123;
    // cout << "a_ms:" << a_ms << endl;
    // int64_t b_ms_int = (int64_t) floor(a_ms);
    // cout << "b_ms_int:" << b_ms_int << endl;   
    // uint32_t b_s_zheng = (uint32_t) (b_ms_int / 1000);
    // cout << "b_s_zheng:" << b_s_zheng << endl;   
    // uint32_t b_ns = (uint32_t)((a_ms-b_s_zheng*1e3)*1e6);
    // cout << "b_ns:" << b_ns << endl;   