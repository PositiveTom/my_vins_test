#include "parameters.h"

//  图像话题名
std::string IMAGE_TOPIC;
//  内参地址
std::string INTRINSIC_ADDRESS;
//  发布频率不能超过的数值
int FREQ;
//  是否进行直方图均衡化
bool EQUALIZE;
//  是否发布此帧的标志
bool PUB_THIS_FRAME;
//  每次跟踪的特征点数
int MAX_CNT;
//  
int ROW, COL;
//  ransac 基础矩阵的阈值
double F_THRESHOLD;
//  不要在光流跟踪到的特征点区域内提取shi-tomasi角点
int MIN_DIST;
void readParameters()
{
    //  读取图像话题名
    IMAGE_TOPIC = "/camera/color/image_raw";

    //  读取相机内参地址
    INTRINSIC_ADDRESS = "/home/zjj/MyCode/myVinsPrj/src/my_utils/data/IMU-CAM-Data/camchain-imucam-imu_tag.yaml";

    FREQ = 30;

    EQUALIZE = true;

    MAX_CNT = 150;

    ROW = 480;

    COL = 640;

    F_THRESHOLD = 1.;

    MIN_DIST = 30; // 两个特征点至少距离 30个像素
}