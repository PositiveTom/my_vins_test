#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <vector>

using namespace std;

class feature
{
public:
    enum feature_detect
    {
        Harris = 0,
        ORB,
        FAST
    };
    virtual void featureExtractor(cv::Mat& img) = 0;

    virtual void myfeartureExtractor(cv::Mat& img) = 0;
};

class harris: public feature
{
public:
    virtual void featureExtractor(cv::Mat& img)
    {
        chrono::steady_clock::time_point start =  chrono::steady_clock::now();

        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        cv::Mat harris_response=cv::Mat::zeros(gray.size(), CV_32FC1);
        //  计算harris角点，实际上只是计算梯度的幅值
        cv::cornerHarris(gray, harris_response, 5, 3, 0.05, cv::BORDER_DEFAULT);
        
        double max = -99999;
        double min = 99999;
        cv::Point maxpos;
        cv::Point minpos;

        cv::minMaxLoc(harris_response, &min, &max, &minpos, &maxpos, cv::Mat());
        // cout << "max:" << max << endl;
        // cout << "min:" << min << endl;

        cv::Mat dilated;
        cv::dilate(harris_response, dilated, cv::Mat());
        cv::Mat localMax(harris_response.rows, harris_response.cols, harris_response.type());
        for(int y=0; y<harris_response.rows; y++)
        {
            for(int x=0; x<harris_response.cols; x++)
            {
                if(harris_response.at<float>(y,x)==dilated.at<float>(y,x))
                {
                    localMax.at<float>(y,x) = harris_response.at<float>(y,x);
                }
                else
                {
                    localMax.at<float>(y,x) = 0.0f;
                }
            }
        }

        cv::Mat threshimg;
        cv::threshold(localMax, threshimg, 0.05*max, 255, cv::THRESH_BINARY);
        // cv::Mat normimg;
        // cv::normalize(harris_response, normimg, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
        // cv::Mat resultimg;
        // cv::convertScaleAbs(normimg, resultimg);
        cv::Mat imgclone = img.clone();
        for(int y=5; y<threshimg.rows-5; y++)
        {
            for(int x=5; x<threshimg.cols-5; x++)
            {
                if(threshimg.at<float>(y,x))
                {
                    cv::rectangle(imgclone, cv::Point(x-5,y-5), cv::Point(x+5,y+5), cv::Scalar(0,255,0));
                    cv::circle(imgclone, cv::Point(x,y), 2,cv::Scalar(0,255,0),-1);  
                }
            }
        }
        chrono::steady_clock::time_point end =  chrono::steady_clock::now();
        chrono::duration<double> time = chrono::duration_cast<chrono::duration<double>>(end-start);
        cout << "time:" << int(1.0/time.count()) << "FPS" << endl;
        cv::namedWindow("harris", cv::WINDOW_NORMAL);
        cv::imshow("harris", imgclone);
        cv::waitKey(1);       
    }

    virtual void myfeartureExtractor(cv::Mat& img)
    {
        chrono::steady_clock::time_point start =  chrono::steady_clock::now();

        // [1]. 产生高斯核
        cv::Mat gauss_vec = cv::getGaussianKernel(5,1,CV_32F);
        cv::Mat gauss_kernel = gauss_vec * gauss_vec.t();
        // cout << "gauss_kernal:" << endl << gauss_kernal << endl;

        //  [2]. 图像变为灰度图
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

        //  [3]. 创建IxIx,IyIy,IxIy幅值图
        cv::Mat IxIx(gray.rows, gray.cols, CV_8UC1);
        cv::Mat IyIy(gray.rows, gray.cols, CV_8UC1);
        cv::Mat IxIy(gray.rows, gray.cols, CV_8UC1);

        //  [4]. 遍历像素
        for(int y=1; y<img.rows; y++)
        {
            for(int x=1; x<img.cols; x++)
            {
                //  [5]. 卷积核计算梯度
                //  sobel 垂直卷积核 y 轴
                //  -1 0 1
                //  -2 0 2
                //  -1 0 1
                //  sobel 水平卷积核 x轴
                //  -1 -2 -1
                //   0  0  0
                //   1  2  1
                uchar ix = img.at<uchar>(y+1,x-1)-img.at<uchar>(y-1,x-1)+
                           2*(img.at<uchar>(y+1,x)-img.at<uchar>(y-1,x))+
                           img.at<uchar>(y+1,x+1)-img.at<uchar>(y-1,x+1);
                uchar iy = img.at<uchar>(y-1,x+1)-img.at<uchar>(y-1,x-1)+
                            2*(img.at<uchar>(y,x+1)-img.at<uchar>(y,x-1))+
                            img.at<uchar>(y+1,x+1)-img.at<uchar>(y+1,x-1);
                
                IxIx.at<uchar>(y,x) = ix*ix;
                IyIy.at<uchar>(y,x) = iy*iy;
                IxIy.at<uchar>(y,x) = ix*iy;
            }
        }
        //  [5]. 创建harris响应矩阵
        cv::Mat R(gray.rows, gray.cols, CV_32FC1);
        float maxResponse = 0.0f; // 最大响应值
        
        //  [6]. 计算M矩阵. //TODO: 为什么这里是这样算的？？？
        int blocksize = 5;
        float k = 0.05; // 0.04~0.06
        for(int y=0; y<=img.rows-blocksize; y++)
        {
            for(int x=0; x<=img.cols-blocksize; x++)
            {   
                //  M
                //  a c
                //  c b
                uchar a = 0;
                uchar b = 0;
                uchar c = 0;
                for(int m=0; m<blocksize; m++)
                {
                    for(int n=0; n<blocksize; n++)
                    {
                        a += IxIx.at<uchar>(m+y,n+x)*gauss_kernel.at<float>(m,n);
                        b += IyIy.at<uchar>(m+y,n+x)*gauss_kernel.at<float>(m,n);
                        c += IxIy.at<uchar>(m+y,n+x)*gauss_kernel.at<float>(m,n);
                    }
                }
                //  [7]. 计算当前像素的harris响应值
                // det M - k (trace M)
                // float t = (a*c-b*b) - k*(a+c)*(a+c); //TODO:???
                float t = a*b-c*c - k*(a+b);
                R.at<float>(y,x) = t;
                if(t > maxResponse)
                {
                    maxResponse = t;
                }
            }
        }

        //  [8]. 求R的局部极大值 //TODO: 为什么是极大值而不是极小值
        //  为了求得局部极大值, 先对R进行膨胀, 再与R进行比较, 相同的就是最大值
        cv::Mat dilated;
        cv::dilate(R, dilated, cv::Mat());
        cv::Mat localMax(R.rows, R.cols, R.type());
        for(int y=0; y<R.rows; y++)
        {
            for(int x=0; x<R.cols; x++)
            {
                if(R.at<float>(y,x)==dilated.at<float>(y,x))
                {
                    localMax.at<float>(y,x) = R.at<float>(y,x);
                }
                else
                {
                    localMax.at<float>(y,x) = 0.0f;
                }
            }
        }

        //  [9]. 剔除部分局部极大值
        cv::Mat threshold2;
        //  图像二值化函数，如果像素值大于 0.5*maxResponse 设置为(maxval)255,否则设置为 0
        //  THRESH_BINARY是二值化类型
        cv::threshold(localMax, threshold2, 0.5*maxResponse, 255, cv::THRESH_BINARY);
        
        //  [10]. 取出对应的角点坐标 //TODO: 为什么这里要-1
        vector<cv::Point2f> corners;
        corners.reserve((threshold2.rows-1)*(threshold2.cols-1)/2);
        for(int y=0; y<threshold2.rows-1; y++)
        {
            const float* rows = threshold2.ptr<float>(y);
            for(int x=0; x<threshold2.cols-1; x++)
            {
                if(rows[x])
                {
                    corners.push_back(cv::Point2f(x,y));
                }
            }
        }
        cv::Mat imgclone = img.clone();
        //  [11]. 绘制出角点
        for(auto& point:corners)
        {
            int x = point.x;
            int y = point.y;
            cv::rectangle(imgclone, cv::Point(x-5,y-5), cv::Point(x+5,y+5), cv::Scalar(0,255,0));
            cv::circle(imgclone, cv::Point(x,y), 2,cv::Scalar(0,255,0),-1);
        }
        chrono::steady_clock::time_point end =  chrono::steady_clock::now();
        chrono::duration<double> time = chrono::duration_cast<chrono::duration<double>>(end-start);
        cout << "time:" << time.count() << "s" << endl;

        cv::namedWindow("harris", cv::WINDOW_NORMAL);
        cv::imshow("harris", imgclone);
        cv::waitKey(1);
    }

};

class Shi_Tomasi: public feature
{
public:
    virtual void featureExtractor(cv::Mat& img)
    {
        chrono::steady_clock::time_point start = chrono::steady_clock::now();
        // 1. 灰度图
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

        //  2. Shi-Tomasi角点
        vector<cv::Point2f> corner_points;
        cv::goodFeaturesToTrack(
            gray,
            corner_points,
            500,                //  最大角点数
            0.01,               //  角点检测可接收的最小特征值 //TODO: 这个特征值是如何确定的？？
            10,                 //  角点之间的最小距离, 通过这个参数来缓解聚簇现象
            cv::Mat(),          //  感兴趣的区域
            3,                  //  blocksize的大小,用于计算M矩阵
            false,               //  不使用harris角点检测器
            0.04                //  k值，计算M矩阵需要的
        );

        // cv::rectangle(imgclone, cv::Point(x-5,y-5), cv::Point(x+5,y+5), cv::Scalar(0,255,0));
        // cv::circle(imgclone, cv::Point(x,y), 2,cv::Scalar(0,255,0),-1); 

        chrono::steady_clock::time_point end = chrono::steady_clock::now();
        chrono::duration<double> cost = chrono::duration_cast<chrono::duration<double>>(end-start);
        cout << "cost:" << int(1.0 / cost.count()) << "FPS" << endl;
        //  3. 绘制角点
        for(auto& point:corner_points)
        {
            cv::circle(img, point, 2, cv::Scalar(0,255,0),-1);
            cv::rectangle(img, cv::Point(point.x-5,point.y-5), cv::Point(point.x+5,point.y+5),
                    cv::Scalar(0,255,0));
        }

        cv::namedWindow("Shi-Tomasi", cv::WINDOW_NORMAL);
        cv::imshow("Shi-Tomasi", img);
        cv::waitKey(1);
    }

    virtual void myfeartureExtractor(cv::Mat& img)
    {

    }

};
