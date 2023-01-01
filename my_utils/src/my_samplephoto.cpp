#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;

string cameraName;
string savePath;
bool isRealSense;
string camTopicName;
string windowName = "photo";

void savePhoto(cv::Mat& frame)
{
    
}

void subCallback(const sensor_msgs::ImageConstPtr& msg)
{
    static int i=1;
    cv_bridge::CvImagePtr cv_ptr = 
            cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_ptr->image;

    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::imshow(windowName, image);
    if(cv::waitKey(30) == 'q')
    {
        string name = savePath + "/" + to_string(i++) + ".jpg";
        cv::imwrite(name, image);
    }    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sample_photo");
    ros::NodeHandle n;
    n.getParam("/sample_photo/cameraName", cameraName);
    n.getParam("/sample_photo/savePath", savePath);
    n.getParam("/sample_photo/isRealSense", isRealSense);

    if(isRealSense)
    {
        cv::VideoCapture capture(cameraName);
        if(!capture.isOpened())
        {
            cout << "failed open" << endl;
            std::abort();
        }
        string windowName = "video";
        cv::namedWindow(windowName, cv::WINDOW_NORMAL);
        cv::Mat frame;
        int i = 1;
        while(ros::ok())
        {
            capture >> frame;
            if(frame.empty())
                break;
            cv::imshow(windowName, frame);
            if(cv::waitKey(30) == 'q')
            {
                string name = savePath + "/" + to_string(i++) + ".jpg";
                cv::imwrite(name, frame);
            }
        }
    }
    else
    {
        n.getParam("/sample_photo/camTopicName", camTopicName);
        ros::Subscriber sub = n.subscribe(camTopicName, 30 ,&subCallback);
    
        ros::spin();
    }
    return 0;
}

