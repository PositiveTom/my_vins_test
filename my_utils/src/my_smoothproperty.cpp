#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sophus/so3.h>
#include <Eigen/Eigen>
#include <thread>

using namespace std;

void threadCallback()
{
    //  初始矩阵李群
    Eigen::Matrix3d RSO3_start = Eigen::Matrix3d::Identity();
    Sophus::SO3 R_start_SO3(RSO3_start);
    Eigen::Vector3d r_start_so3 = R_start_SO3.log();

    //  终止矩阵李群
    Eigen::Vector3d YPR(M_PI, M_PI_2, 0);
    Eigen::AngleAxisd R_yaw(YPR[0], Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd R_pitch(YPR[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd R_roll(YPR[2], Eigen::Vector3d::UnitX());
    Eigen::Matrix3d RSO3_end;
    RSO3_end = R_yaw * R_pitch * R_roll;
    Sophus::SO3 R_end_SO3(RSO3_end);
    Eigen::Vector3d r_end_so3 = R_end_SO3.log();

    double num = 100; // 插值100次
    int i = 0;

    for(;ros::ok();)
    {
        if(i>num)
        {
            i = 0;
        }
        //  [1]. 创建广播者
        static tf::TransformBroadcaster br;
        //  [2]. 创建存储的变换关系
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(3, 2, 1));
 
        Eigen::Vector3d r_mid_so3 = r_start_so3 + (r_end_so3 - r_start_so3) * (double(i++)/num);
        Sophus::SO3 R_mid_SO3 = Sophus::SO3::exp(r_mid_so3);
        Eigen::Quaterniond Q_mid = R_mid_SO3.unit_quaternion();
        tf::Quaternion q;
        q.setW(Q_mid.w());
        q.setX(Q_mid.x());
        q.setY(Q_mid.y());
        q.setZ(Q_mid.z());
        
        transform.setRotation(q);

        //  [3]. 发布变换关系, 子坐标到父坐标的变换，意味着是把子坐标变换到父坐标
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "camera"));

        ros::Duration(0.1).sleep();
    }
}

void checkYacobians()
{
    //  [1]. 设置一个相机系下坐标
    Eigen::Vector3d pc(2, 2, 2);

    //  [2]. 设置当前相机坐标系到世界坐标系的变换关系
    Eigen::Vector3d YPR(M_PI_4, M_PI_4, 0);
    Eigen::AngleAxisd R_yaw(YPR[0], Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd R_pitch(YPR[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd R_roll(YPR[2], Eigen::Vector3d::UnitX());
    Eigen::Matrix3d RSO3;
    RSO3 = R_yaw * R_pitch * R_roll;
    cout << "RSO3:" << endl << RSO3 << endl;

    //  [3]. 利用左扰动模型求出解析雅可比矩阵
    Eigen::Matrix3d analytic_yacobians = - Sophus::SO3::hat(RSO3 * pc);
    cout << "analytic_yacobians:" << endl << analytic_yacobians << endl;

    //  [4]. 构造李代数微小增量
    double epsilon = 1e-4;
    // Eigen::Vector3d delta_phi = Eigen::Vector3d::Ones()*epsilon;
    Eigen::Vector3d delta_phi(epsilon, 0, 0);
    Sophus::SO3 delta_SO3 = Sophus::SO3::exp(delta_phi);
    Eigen::Matrix3d delta_R = delta_SO3.matrix();

    //  [5]. 计算 R(\delta \phi)*R(\phi)*pc - R(\phi)*pc
    Eigen::Vector3d tmp = delta_R * RSO3 * pc - RSO3 * pc;
    // cout << "tmp:" << endl << tmp << endl;

    //  [6]. 计算数值雅可比矩阵的第一列
    Eigen::Matrix3d value_yacobians;
    value_yacobians.setZero();
    // value_yacobians.block<1,3>(0,0) = delta_phi.transpose().cwiseInverse() * tmp[0];
    // value_yacobians.block<1,3>(1,0) = delta_phi.transpose().cwiseInverse() * tmp[1];
    // value_yacobians.block<1,3>(2,0) = delta_phi.transpose().cwiseInverse() * tmp[2];
    value_yacobians.block<3,1>(0,0) = tmp / delta_phi[0];

    //  [7]. 计算数值雅可比矩阵的第二列
    delta_phi << 0, epsilon, 0;
    delta_SO3 = Sophus::SO3::exp(delta_phi);
    delta_R = delta_SO3.matrix();
    tmp = delta_R * RSO3 * pc - RSO3 * pc;
    value_yacobians.block<3,1>(0,1) = tmp / delta_phi[1];

    //  [8]. 计算数值雅可比矩阵的第三列
    delta_phi << 0, 0, epsilon;
    delta_SO3 = Sophus::SO3::exp(delta_phi);
    delta_R = delta_SO3.matrix();
    tmp = delta_R * RSO3 * pc - RSO3 * pc;
    value_yacobians.block<3,1>(0,2) = tmp / delta_phi[2];

    cout << "value_yacobians:" << endl << value_yacobians << endl;
}

Eigen::Matrix3d antiskew(Eigen::Vector3d &p)
{
    Eigen::Matrix3d p_hat;
    p_hat << 0, -p(2), p(1),
             p(2), 0, -p(0),
             -p(1), p(0), 0;
    return p_hat;
}

//  检查反对称矩阵的性质 Rp^a ?= -Ra^p
void checkAntisymmetric()
{
    //  [1]. 任意创建一个旋转矩阵
    Eigen::AngleAxisd R_angleAxise(M_PI_4, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d R_matrix(R_angleAxise);
    cout << "R_angleAxise:" << endl << R_matrix << endl;

    //  [2]. 任意创建一个向量a和一个向量p
    Eigen::Vector3d p(1, 2, 3);
    Eigen::Vector3d a(0.5, 0.3, 0.1);

    //  [3]. 创建反对称矩阵
    Eigen::Matrix3d p_hat = antiskew(p);
    Eigen::Matrix3d a_hat = antiskew(a);

    //  /*********************/
    //  Rp^a = -Ra^p = -(Ra)^Rp

    //  [4]. 比较结果
    Eigen::Vector3d result1 = R_matrix * p_hat * a; // 原始式子
    Eigen::Vector3d result2 = - R_matrix * a_hat * p;   //  变换形式1
    Eigen::Vector3d temp = R_matrix * a;
    Eigen::Vector3d result3 = - antiskew( temp ) * R_matrix * p;    //  变换形式2

    //  TODO: 竟然都相等,不可思议

    cout << "result1:" << endl << result1 << endl;
    cout << "result2:" << endl << result2 << endl;
    cout << "result3:" << endl << result3 << endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "testproperty");
    ros::NodeHandle n;
    
    thread my_thread(checkAntisymmetric);

    ros::spin();
    return 0;
}