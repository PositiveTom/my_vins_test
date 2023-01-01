#include <ros/ros.h>
#include "ORBextractor.h"
#include "opencv2/opencv.hpp"
#include <thread>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <chrono>

using namespace std;

ORB_SLAM2::ORBextractor orbextractor(150, 1.2, 8, 20, 7); 

void SubColorImage(const sensor_msgs::ImageConstPtr &msg)
{
    chrono::steady_clock::time_point start = chrono::steady_clock::now();
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    cv::Mat image = cv_ptr->image;
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    orbextractor(gray, cv::Mat(), keypoints, descriptors);

    chrono::steady_clock::time_point end = chrono::steady_clock::now();
    chrono::duration<double> cost = (end - start);
    cout << cost.count() * 1000 << "ms" << endl;
    cout << "keypoints.size():" << keypoints.size() << endl;
    int rows = gray.rows;
    int cols = gray.cols;
    for(auto &keypoint:keypoints)
    {
        const int r = 5;
        cv::Point2f pt1, pt2;
        pt1.x = keypoint.pt.x - 5;
        pt1.y = keypoint.pt.y - 5;
        pt2.x = keypoint.pt.x + 5;
        pt2.y = keypoint.pt.y + 5;

        if(pt1.x > 0 && pt1.y > 0 && pt2.x < cols && pt2.y < rows )
        {
            cv::rectangle(image, pt1, pt2, cv::Scalar(0,255,0));
            cv::circle(image, keypoint.pt, 2, cv::Scalar(0,255,0), -1);
        }
    }
    cv::namedWindow("photo", cv::WINDOW_NORMAL);
    cv::imshow("photo", image);
    cv::waitKey(1);
}

void OrbExtraCallback(ros::NodeHandle &&n)
{
//    ros::Subscriber sub_color_image = n.subscribe<sensor_msgs::Image>("/cam/image/raw", 10, SubColorImage);    


}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "OrbExtraCallback");
    ros::NodeHandle n;
    ros::Subscriber sub_color_image = n.subscribe<sensor_msgs::Image>("/camera/color/image_raw", 10, SubColorImage);    
    ROS_INFO("HELLO");
    // std::thread OrbThread(OrbExtraCallback, n);

    // OrbThread.join();
    ros::spin();

    return 0;
}
