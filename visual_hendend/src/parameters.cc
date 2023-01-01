#include "parameters.h"

//  [1]. IMU话题名
std::string IMU_TOPIC_NAME;
//  [2]. IMU-CAM-Config
std::string IMU_CAM_CONFIG_FILE_PATH;
//  [3]. 外参 旋转矩阵
Eigen::Matrix3d RCI;
//  外参 平移向量
Eigen::Vector3d TCI;
//  [4]. 时间偏移
double TD;
//  [5]. 
double N_A;     //  加速度计随机噪声的标准差
double N_G;     //  陀螺仪随机噪声的标准差
double B_N_A;   //  加速度计随机游走项的导数的标准差
double B_N_G;   //  陀螺仪随机游走项的导数的标准差
//  [6]. 视差, TODO:?? 这里是需要调整的              !!!!!!!!!
double MIN_PARALLAX = 3;
// const int WINDOW_SIZE = 11;

void readParameters()
{
    //  [1]. 设置IMU话题名
    IMU_TOPIC_NAME = "/camera/imu";
    //  [2]. IMU-CAM 外参文件路径
    IMU_CAM_CONFIG_FILE_PATH = "/home/zjj/MyCode/myVinsPrj/src/my_utils/data/IMU-CAM-Data/camchain-imucam-imu_tag.yaml";
    //  [3]. 读取相机-IMU外参, 读取多维数组
    cv::FileStorage fs;
    if(!fs.open(IMU_CAM_CONFIG_FILE_PATH, cv::FileStorage::READ))
    {
        cout << "------ERROR------" << endl;
        std::abort();
    }
    std::vector<std::vector<double>> matTransformMatrix;
    cv::FileNode fn = fs["cam0"];
    fn["T_cam_imu"] >> matTransformMatrix;

    RCI << matTransformMatrix[0][0], matTransformMatrix[0][1], matTransformMatrix[0][2],
           matTransformMatrix[1][0], matTransformMatrix[1][1], matTransformMatrix[1][2],
           matTransformMatrix[2][0], matTransformMatrix[2][1], matTransformMatrix[2][2];
    TCI << matTransformMatrix[0][3], matTransformMatrix[1][3], matTransformMatrix[2][3];
    cout << "RCI:" << endl << RCI << endl;
    cout << "TCI:" << endl << TCI << endl;
    
    //  [4]. 读取时间偏移
    TD = (double)fn["timeshift_cam_imu"];
    // cout << "TD:" << TD << endl;
    ROS_INFO("TD:%lf", TD);
    fs.release();

    //  [5]. 读取IMU噪声模型的参数
    cv::FileStorage fs_imu("/home/zjj/MyCode/myVinsPrj/src/my_utils/data/IMU-CAM-Data/imu-imu_tag.yaml", cv::FileStorage::READ);
    if(!fs_imu.isOpened())
    {
        ROS_ERROR("Failed to read imu parameters!");
    }
    cv::FileNode fn_imu = fs_imu["imu0"];
    N_A = (double)fn_imu["accelerometer_noise_density"];
    B_N_A = (double)fn_imu["accelerometer_random_walk"];
    N_G = (double)fn_imu["gyroscope_noise_density"];
    B_N_G = (double)fn_imu["gyroscope_random_walk"];
    fs_imu.release();
    ROS_INFO("N_A:%lf", N_A);
    ROS_INFO("B_N_A:%lf", B_N_A);
    ROS_INFO("N_G:%lf", N_G);
    ROS_INFO("B_N_G:%lf", B_N_G);
}