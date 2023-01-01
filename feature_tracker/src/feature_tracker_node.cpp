#include <ros/ros.h>
#include "ORBextractor.h"
#include "feature_tracker.h"
#include "parameters.h"
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <geometry_msgs/Point32.h>

ros::Publisher pub_restart_flag;
ros::Publisher pub_img;

bool init_pub = false;
//  特征跟踪器
FeatureTracker tracker;
//  是否是第一帧图像
bool first_image_flag = true;
//  第一帧图像的时间戳
double first_image_time;
//  上一帧图像的时间戳
double last_image_time;
//  真正接收了的图像帧数, 主要是用于计算发布频率的作用
int pub_count;
//  事先申请mat内存
cv::Mat gray(640, 480, CV_8U), color(640, 480, CV_8UC3);

void subImageCallback(const sensor_msgs::ImageConstPtr &img_msg)
{
    //  [1]. 如果是第一帧图像, 则跳过, 同时记录第一帧图像的时间戳
    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = img_msg->header.stamp.toSec();
        last_image_time = first_image_time;
        return;
    }
    //  [2]. 检测相机流是否稳定
    if( img_msg->header.stamp.toSec() - last_image_time > 1.|| img_msg->header.stamp.toSec() < last_image_time )
    {
        ROS_WARN("image discontinue! reset the feature tracker!");
        //  [2.1] 相机流不稳定, 则复位相机流
        first_image_flag = true;
        last_image_time = 0;
        pub_count = 1;
        std_msgs::Bool restart_msg;
        restart_msg.data = true;
        pub_restart_flag.publish(restart_msg);
        return;
    }
    //  [3]. 记录当前帧的时间戳 给 存储上一帧时间戳的变量
    last_image_time = img_msg->header.stamp.toSec();

    //  [4]. 控制发布频率
    if(round(1.0 * pub_count / ( img_msg->header.stamp.toSec() - first_image_time )) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        //  实际发布频率与期望发布频率之差小于0.01*FREQ, 则将 first_image_time 复位为当前时间戳
        //  pub_count 复位为0
        //  TODO 这样做的目地是为了重新计算发布频率,避免因精度误差导致的发布频率计算不准确.
        if( (round(1.0 * pub_count / ( img_msg->header.stamp.toSec() - first_image_time ))-FREQ)<0.01*FREQ )
        {
            // ROS_INFO("----Reset Caculate Control Frequency!----");
            first_image_time = img_msg->header.stamp.toSec();
            pub_count = 0;
        }
        else
            ROS_INFO("----Not Reset Caculate Control Frequency!----");
    }
    else
    {
        ROS_WARN("----This frame is not published!----");
        PUB_THIS_FRAME = false;
    }

    //  [5]. 把sensor_msgs图像转为mat图像
    // cout << "img_msg->encoding:" << img_msg->encoding << endl;
    cv_bridge::CvImageConstPtr cvptr;
    if( img_msg->encoding == "8UC1" )
    {
        //  [5.1]. 如果图像原本就是灰度图
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        //  大端字节序flag, 是的话,高字节存放低地址
        img.is_bigendian = img_msg->is_bigendian;
        //  每行所占的字节数
        img.step = img_msg->step;
    }
    else
    {
        //  [5.1]. 如果图像原本是rgb8图
        cvptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
        //  [5.2]. 把rgb图转为灰度图
        color = cvptr->image;
        cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);
    }
    // cout << "img_msg->step"<< img_msg->step << endl;  // rgb图: 640 x 3 = 1920个字节

    // TicToc t_read;
    //  [6]. 开始处理图像
    tracker.readImage(gray, img_msg->header.stamp.toSec());
    // cout << t_read.toc() << "ms" << endl;

    //  [7]. 给特征点赋予id值
    for(ulong i=0; ;i++)
    {
        bool completed = false;
        completed |= tracker.updateID(i);
        if(!completed)
            break;
    }

    //  [8]. 发布特征数据
    if(PUB_THIS_FRAME)
    {
        pub_count++;
        sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
        sensor_msgs::ChannelFloat32 id_of_point;    //  特征点id
        sensor_msgs::ChannelFloat32 u_of_point;     //  特征点u轴像素坐标
        sensor_msgs::ChannelFloat32 v_of_point;     //  特征点v轴像素坐标
        sensor_msgs::ChannelFloat32 velocity_x_of_point;    //  特征点的光流速度,定义在齐次相机坐标上
        sensor_msgs::ChannelFloat32 velocity_y_of_point;

        feature_points->header = img_msg->header;
        feature_points->header.frame_id = "frame0_world";

        auto &un_pts = tracker.cur_un_pts;  //   无畸变特征点, 齐次相机坐标
        auto &cur_pts = tracker.cur_pts;    //   畸变特征点像素坐标
        auto &id = tracker.ids;             //   特征点的id
        auto &velocity = tracker.pts_velocity;
        auto &track_cnt = tracker.track_cnt;
        for(int i=0; i<id.size(); i++)
        {
            //  [8.1] 只发布被跟踪超过或等于两次的特征点
            if(tracker.track_cnt[i] > 1)
            {
                //  提取齐次相机坐标
                geometry_msgs::Point32 p;
                p.x = un_pts[i].x;
                p.y = un_pts[i].y;
                p.z = 1;

                feature_points->points.push_back(p);

                id_of_point.values.push_back(id[i]);
                u_of_point.values.push_back(cur_pts[i].x);
                v_of_point.values.push_back(cur_pts[i].y);
                velocity_x_of_point.values.push_back(velocity[i].x);
                velocity_y_of_point.values.push_back(velocity[i].y);
                // feature_points->channels.push_back()
            }
        }

        feature_points->channels.push_back(id_of_point);
        feature_points->channels.push_back(u_of_point);
        feature_points->channels.push_back(v_of_point);
        feature_points->channels.push_back(velocity_x_of_point);
        feature_points->channels.push_back(velocity_y_of_point);
        ROS_DEBUG("publish %f, at %f", feature_points->header.stamp.toSec(), ros::Time::now().toSec());

        if(!init_pub)
        {
            //  当特征点跟踪次数还没有超过或等于2次时,不发布这一次的消息
            init_pub = true;
        }
        else
            pub_img.publish(feature_points);

        if(showTrack)
        {
            cv::Mat image = color.clone();
            for(int j=0; j<cur_pts.size(); j++)
            {
                double len = std::min(1.0, 1.0 * track_cnt[j] / 20);
                //  红色代表新来的特征点
                cv::circle(image, cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
            }
            cv::namedWindow("showTrack", cv::WINDOW_NORMAL);
            cv::imshow("showTrack", image);
            cv::waitKey(1);
        }
    }
    // cv::namedWindow("gray", cv::WINDOW_NORMAL);
    // cv::imshow("gray", gray);
    // cv::waitKey(1);
}

// ^Czjj@Galaxy:~/MyCode/myVinsPrj$ rosmsg info sensor_msgs/PointCloud
// std_msgs/Header header
//   uint32 seq
//   time stamp
//   string frame_id
// geometry_msgs/Point32[] points
//   float32 x
//   float32 y
//   float32 z
// sensor_msgs/ChannelFloat32[] channels
//   string name
//   float32[] values

int main(int argc, char** argv)
{
    ros::init(argc, argv, "feature_tracker_node");
    ros::NodeHandle n("~");
    //  [1]. 设置日志记录器的级别, Debug是记录调试信息
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    //  [2]. 读取相关参数(图像话题名)
    // ROS_INFO("Begin");
    readParameters();
    //  [3]. 读取内参
    tracker.readIntrinsicParameter(INTRINSIC_ADDRESS);
    //  [4]. 创建订阅图像数据的订阅者
    ros::Subscriber sub_img = n.subscribe(IMAGE_TOPIC, 100, subImageCallback);
    //  [5]. 创建发布复位标志的客户端
    pub_restart_flag = n.advertise<std_msgs::Bool>("restart", 1000);
    //  [6]. 发布图像的特征点id,
    pub_img = n.advertise<sensor_msgs::PointCloud>("feature", 1000);

    ros::spin();
    return 0;
}
