#include "my_visualhendendtest.hpp"
#include <memory>
#include "OrbExtractor.h"
#include <chrono>

#define ORB

MyOrbExtractor orbExtractor;

void subCallback(const sensor_msgs::ImageConstPtr &msg)
{

    cv_bridge::CvImagePtr ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat img = ptr->image;
#ifdef HARRIS
    shared_ptr<feature> myfeature = shared_ptr<feature>(new Shi_Tomasi());
    // myfeature->myfeartureExtractor(img);
    // myfeature->featureExtractor(img);
    cv::namedWindow("test", cv::WINDOW_NORMAL);
    cv::imshow("test", img);
    cv::waitKey(1); 
#endif
#ifdef ORB
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    orbExtractor.FeatureExtractor(img, keypoints, descriptors);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::chrono::duration<double> cost = (end - start);
    cout << "cost:" << cost.count()*1000 << "ms" << endl;
    cout << "keypoints.size():" << keypoints.size() << endl;
    int rows = img.rows;
    int cols = img.cols;
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
            cv::rectangle(img, pt1, pt2, cv::Scalar(0,255,0));
            cv::circle(img, keypoint.pt, 2, cv::Scalar(0,255,0), -1);
        }
    }

    cv::namedWindow("photo", cv::WINDOW_NORMAL);
    cv::imshow("photo", img);
    cv::waitKey(1);
#endif

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_visualhendendtest");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/camera/color/image_raw", 100, &subCallback);
    
    ros::spin();

    return 0;
}