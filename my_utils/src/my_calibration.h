#ifndef MY_CALIBRATION_H
#define MY_CALIBRATION_H

#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include <string>
#include <sys/types.h>
#include <dirent.h>
#include <Eigen/Eigen>
#include <map>
#include <memory>
#include <algorithm>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <fstream>
#include <sstream>
#include <Eigen/SVD>
#include <iostream>
#include <sophus/so3.h>

using namespace std;

namespace tool
{
    Eigen::Matrix3d antiSymmetric(Eigen::Vector3d &vec)
    {
        Eigen::Matrix3d sym;
        sym << 0, -vec[2], vec[1],
               vec[2], 0, -vec[0],
               -vec[1], vec[0],0;
        return sym;
    }
}

class reprojectCost: public ceres::SizedCostFunction<2, 5, 2, 7>
{
public:
    reprojectCost(double u, double v, double Xw_, double Yw_):u_mse(u), v_mse(v), Xw(Xw_), Yw(Yw_){}

    // ceres::Manifold* q;
    virtual bool Evaluate(double const* const* parameters,
                    double* residuals,
                    double** jacobians) const
    {
        double alpha = parameters[0][0];
        double beta = parameters[0][1];
        double gamma = parameters[0][2];
        double u0 = parameters[0][3];
        double v0 = parameters[0][4];

        double k0 = parameters[1][0];
        double k1 = parameters[1][1];

        Eigen::Map<const Eigen::Vector3d> Pcw(parameters[2]);
        // cout << "Pcw:" << Pcw << endl;
        //  外面传的参数是四元数，但是增量是李代数
        Eigen::Map<const Eigen::Quaterniond> Qcw(parameters[2]+3);
        // cout << "Qcw:" << Qcw.coeffs() << endl;
        Eigen::Matrix3d Rcw = Qcw.toRotationMatrix();

        //  [1]. 计算归一化相机坐标
        //  [1.1] 计算相机坐标
        Eigen::Vector3d worldCor(Xw, Yw, 0);
        Eigen::Vector3d cameraCorw = Rcw * worldCor + Pcw;
        //  [1.2] 计算归一化相机坐标
        Eigen::Vector3d cameraCor(cameraCorw);
        cameraCor /= cameraCorw[2];

        //  [2]. 计算畸变图像坐标
        double x2 = cameraCor[0]*cameraCor[0];
        double y2 = cameraCor[1]*cameraCor[1];
        double xy = cameraCor[0]*cameraCor[1];
        double r2 = x2 + y2;
        double r4 = r2 * r2;
        double result = 1.0 + k0*r2 + k1*r4;
        Eigen::Vector2d distortedCameraCor;
        distortedCameraCor[0] = cameraCor[0] * result;
        distortedCameraCor[1] = cameraCor[1] * result;

        //  [3]. 计算畸变像素坐标
        Eigen::Vector2d distortedPiexlCor;
        distortedPiexlCor[0] = distortedCameraCor[0] * alpha + distortedCameraCor[1] * gamma + u0;
        distortedPiexlCor[1] = distortedCameraCor[1] * beta + v0;
        
        //  [4]. 计算残差
        // cout << "new" << endl;
        residuals[0] = distortedPiexlCor[0] - u_mse;
        residuals[1] = distortedPiexlCor[1] - v_mse;

        //  [5]. 计算雅可比
        if(jacobians)
        {
            if(jacobians[0])
            {   
                // cout << "parameters[0]" << hex << parameters[0] << endl;
                // cout << "jacobians[0]" << hex << jacobians[0] << endl;
                Eigen::Map<Eigen::Matrix<double, 2, 5, Eigen::RowMajor>> J_interal(jacobians[0]);
                J_interal << distortedCameraCor[0], 0, distortedCameraCor[1], 1, 0,
                            0, distortedCameraCor[1], 0, 0, 1;
                // cout << "J_interal:" << J_interal << endl;
            }
            if(jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>> J_distorted(jacobians[1]);
                J_distorted << alpha*cameraCor[0]*r2+gamma*cameraCor[1]*r2, 
                            alpha*cameraCor[0]*r4+gamma*cameraCor[1]*r4,
                            beta*cameraCor[1]*r2, beta*cameraCor[1]*r4;
                // cout << "J distorted:" << J_distorted << endl;
            }
            if(jacobians[2])
            {
                Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> J_transform(jacobians[2]);
                J_transform.setZero();
                
                Eigen::Matrix2d deflector1;
                deflector1 << alpha, gamma, 0, beta;

                Eigen::Matrix2d deflector2;
                // k1*(x^2 + y^2)^2 + x*(2*k0*x + 4*k1*x*(x^2 + y^2)) + k0*(x^2 + y^2) + 1  &  x*(2*k0*y + 4*k1*y*(x^2 + y^2))
                //  y*(2*k0*x + 4*k1*x*(x^2 + y^2)) & k1*(x^2 + y^2)^2 + y*(2*k0*y + 4*k1*y*(x^2 + y^2)) + k0*(x^2 + y^2) + 1
                deflector2 << k1*r4 + 2*k0*x2 + 4*k1*x2*r2 + k0*r2 + 1, 2*k0*xy + 4*k1*xy*r2,
                            2*k0*xy + 4*k1*xy*r2, k1*r4 + 2*k0*y2 + 4*k1*y2*r2 + k0*r2 + 1;

                Eigen::Matrix<double, 2, 3> deflector3;
                deflector3 << 1.0/cameraCorw[2], 0, -cameraCorw[0]/(cameraCorw[2]*cameraCorw[2]),
                            0, 1.0/cameraCorw[2], -cameraCorw[1]/(cameraCorw[2]*cameraCorw[2]);

                Eigen::Matrix3d deflectorPcw = Eigen::Matrix3d::Identity();

                Eigen::Matrix3d deflectorRcw;
                Eigen::Vector3d temp = Rcw * worldCor;
                deflectorRcw = -tool::antiSymmetric(temp) * Rcw;

                J_transform.block<2,3>(0,0) = deflector1 * deflector2 * deflector3 * deflectorPcw;
                J_transform.block<2,3>(0,3) = deflector1 * deflector2 * deflector3 * deflectorRcw;

                // cout << "Jpose:" << endl << J_transform << endl;
            }
        }
        // double alpha = parameters[0][0];
        // double beta = parameters[0][1];
        // double gamma = parameters[0][2];
        // double u0 = parameters[0][3];
        // double v0 = parameters[0][4];

        // double k0 = parameters[1][0];
        // double k1 = parameters[1][1];

        // Eigen::Map<const Eigen::Vector3d> Pcw(parameters[2]);
        // //  外面传的参数是四元数，但是增量是李代数
        // Eigen::Map<const Eigen::Quaterniond> Qcw(parameters[2]+3);
        // Eigen::Matrix3d Rcw = Qcw.toRotationMatrix();

        // //  世界坐标
        // Eigen::Matrix<double, 4, 1> Pw(Xw, Yw, 0, 1);
        // Eigen::Vector3d caPw(Xw, Yw, 0);
        // //  Tcw
        // Eigen::Matrix<double, 3, 4> Tcw;
        // Tcw.block<3,3>(0,0) = Rcw;
        // Tcw.block<3,1>(0,3) = Pcw;
        // //  相机坐标
        // Eigen::Vector3d Pc;
        // Pc = Tcw * Pw;
        // //  齐次相机坐标
        // Eigen::Vector3d homo_Pc;
        // homo_Pc = Pc;
        // homo_Pc /= homo_Pc[2];
        // double r2 = homo_Pc[0]*homo_Pc[0] + homo_Pc[1]*homo_Pc[1];
        // double r4 = r2 * r2;
        // //  畸变齐次相机坐标
        // Eigen::Vector3d distor_homo_PC(0,0,1);
        // distor_homo_PC[0] = homo_Pc[0]*(1 + k0*r2 + k1*r4 );
        // distor_homo_PC[1] = homo_Pc[1]*(1 + k0*r2 + k1*r4 );
        // //  内参矩阵
        // Eigen::Matrix3d K;
        // K << alpha, gamma, u0, 0, beta, v0, 0, 0, 1;
        // //  像素坐标
        // Eigen::Vector3d uv;
        // uv = K * distor_homo_PC;

        // residuals[0] = (uv[0] - u_mse);
        // residuals[1] = (uv[1] - v_mse);

        // double x2 = homo_Pc[0]*homo_Pc[0];
        // double y2 = homo_Pc[1]*homo_Pc[1];
        // double xy = homo_Pc[0]*homo_Pc[1];

        // //  [5]. 计算雅可比
        // if(jacobians)
        // {
        //     if(jacobians[0])
        //     {
        //         Eigen::Map<Eigen::Matrix<double, 2, 5, Eigen::RowMajor>> J_interal(jacobians[0]);
        //         J_interal << distor_homo_PC[0], 0, distor_homo_PC[1], 1, 0,
        //                     0, distor_homo_PC[1], 0, 0, 1;
        //     }
        //     if(jacobians[1])
        //     {
        //         Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>> J_distorted(jacobians[1]);
        //         J_distorted << alpha*homo_Pc[0]*r2+gamma*homo_Pc[1]*r2, 
        //                        alpha*homo_Pc[0]*r4+gamma*homo_Pc[1]*r4,
        //                        beta*homo_Pc[1]*r2, beta*homo_Pc[1]*r4;
        //     }
        //     if(jacobians[2])
        //     {
        //         Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> J_transform(jacobians[2]);
        //         J_transform.setZero();
                
        //         Eigen::Matrix2d deflector1;
        //         deflector1 << alpha, gamma, 0, beta;

        //         Eigen::Matrix2d deflector2;
        //         // k1*(x^2 + y^2)^2 + x*(2*k0*x + 4*k1*x*(x^2 + y^2)) + k0*(x^2 + y^2) + 1  &  x*(2*k0*y + 4*k1*y*(x^2 + y^2))
        //         //  y*(2*k0*x + 4*k1*x*(x^2 + y^2)) & k1*(x^2 + y^2)^2 + y*(2*k0*y + 4*k1*y*(x^2 + y^2)) + k0*(x^2 + y^2) + 1
        //         deflector2 << k1*r4 + 2*k0*x2 + 4*k1*x2*r2 + k0*r2 + 1, 2*k0*xy + 4*k1*xy*r2,
        //                     2*k0*xy + 4*k1*xy*r2, k1*r4 + 2*k0*y2 + 4*k1*y2*r2 + k0*r2 + 1;

        //         Eigen::Matrix<double, 2, 3> deflector3;
        //         deflector3 << 1.0/Pc[2], 0, -Pc[0]/(Pc[2]*Pc[2]),
        //                     0, 1.0/Pc[2], -Pc[1]/(Pc[2]*Pc[2]);

        //         Eigen::Matrix3d deflectorPcw = Eigen::Matrix3d::Identity();

        //         Eigen::Matrix3d deflectorRcw;
        //         Eigen::Vector3d temp = Rcw * caPw;
        //         deflectorRcw = -tool::antiSymmetric(temp) * Rcw;

        //         J_transform.block<2,3>(0,0) = deflector1 * deflector2 * deflector3 * deflectorPcw;
        //         J_transform.block<2,3>(0,3) = deflector1 * deflector2 * deflector3 * deflectorRcw;
        //     }
        // }
        return true;
    }

    //  测试发现，你的雅可比矩阵没有任何问题
    bool debug(double parameters[][7], double jacobians[][14])
    {
        // cout << "parameters[0]" << hex << parameters[0] << endl;
        // cout << "jacobians[0]" << hex << jacobians[0] << endl;
        double residuals[2]={0};

        {
            double alpha = parameters[0][0];
            double beta = parameters[0][1];
            double gamma = parameters[0][2];
            double u0 = parameters[0][3];
            double v0 = parameters[0][4];

            double k0 = parameters[1][0];
            double k1 = parameters[1][1];

            Eigen::Map<const Eigen::Vector3d> Pcw(parameters[2]);
            // cout << "Pcw:" << Pcw << endl;
            //  外面传的参数是四元数，但是增量是李代数
            Eigen::Map<const Eigen::Quaterniond> Qcw(parameters[2]+3);
            // cout << "Qcw:" << Qcw.coeffs() << endl;
            Eigen::Matrix3d Rcw = Qcw.toRotationMatrix();

            //  [1]. 计算归一化相机坐标
            //  [1.1] 计算相机坐标
            Eigen::Vector3d worldCor(Xw, Yw, 0);
            Eigen::Vector3d cameraCorw = Rcw * worldCor + Pcw;
            //  [1.2] 计算归一化相机坐标
            Eigen::Vector3d cameraCor(cameraCorw);
            cameraCor /= cameraCorw[2];

            //  [2]. 计算畸变图像坐标
            double x2 = cameraCor[0]*cameraCor[0];
            double y2 = cameraCor[1]*cameraCor[1];
            double xy = cameraCor[0]*cameraCor[1];
            double r2 = x2 + y2;
            double r4 = r2 * r2;
            double result = 1.0 + k0*r2 + k1*r4;
            Eigen::Vector2d distortedCameraCor;
            distortedCameraCor[0] = cameraCor[0] * result;
            distortedCameraCor[1] = cameraCor[1] * result;

            //  [3]. 计算畸变像素坐标
            Eigen::Vector2d distortedPiexlCor;
            distortedPiexlCor[0] = distortedCameraCor[0] * alpha + distortedCameraCor[1] * gamma + u0;
            distortedPiexlCor[1] = distortedCameraCor[1] * beta + v0;
            
            //  [4]. 计算残差
            // cout << "new" << endl;
            residuals[0] = distortedPiexlCor[0] - u_mse;
            residuals[1] = distortedPiexlCor[1] - v_mse;

            //  [5]. 计算雅可比
            if(jacobians)
            {
                if(jacobians[0])
                {   
                    // cout << "parameters[0]" << hex << parameters[0] << endl;
                    // cout << "jacobians[0]" << hex << jacobians[0] << endl;
                    Eigen::Map<Eigen::Matrix<double, 2, 5, Eigen::RowMajor>> J_interal(jacobians[0]);
                    J_interal << distortedCameraCor[0], 0, distortedCameraCor[1], 1, 0,
                                0, distortedCameraCor[1], 0, 0, 1;
                    // cout << "J_interal:" << J_interal << endl;
                }
                if(jacobians[1])
                {
                    Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>> J_distorted(jacobians[1]);
                    J_distorted << alpha*cameraCor[0]*r2+gamma*cameraCor[1]*r2, 
                                alpha*cameraCor[0]*r4+gamma*cameraCor[1]*r4,
                                beta*cameraCor[1]*r2, beta*cameraCor[1]*r4;
                    // cout << "J distorted:" << J_distorted << endl;
                }
                if(jacobians[2])
                {
                    Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> J_transform(jacobians[2]);
                    J_transform.setZero();
                    
                    Eigen::Matrix2d deflector1;
                    deflector1 << alpha, gamma, 0, beta;

                    Eigen::Matrix2d deflector2;
                    // k1*(x^2 + y^2)^2 + x*(2*k0*x + 4*k1*x*(x^2 + y^2)) + k0*(x^2 + y^2) + 1  &  x*(2*k0*y + 4*k1*y*(x^2 + y^2))
                    //  y*(2*k0*x + 4*k1*x*(x^2 + y^2)) & k1*(x^2 + y^2)^2 + y*(2*k0*y + 4*k1*y*(x^2 + y^2)) + k0*(x^2 + y^2) + 1
                    deflector2 << k1*r4 + 2*k0*x2 + 4*k1*x2*r2 + k0*r2 + 1, 2*k0*xy + 4*k1*xy*r2,
                                2*k0*xy + 4*k1*xy*r2, k1*r4 + 2*k0*y2 + 4*k1*y2*r2 + k0*r2 + 1;

                    Eigen::Matrix<double, 2, 3> deflector3;
                    deflector3 << 1.0/cameraCorw[2], 0, -cameraCorw[0]/(cameraCorw[2]*cameraCorw[2]),
                                0, 1.0/cameraCorw[2], -cameraCorw[1]/(cameraCorw[2]*cameraCorw[2]);

                    Eigen::Matrix3d deflectorPcw = Eigen::Matrix3d::Identity();

                    Eigen::Matrix3d deflectorRcw;
                    Eigen::Vector3d temp = Rcw * worldCor;
                    deflectorRcw = -tool::antiSymmetric(temp) * Rcw;

                    J_transform.block<2,3>(0,0) = deflector1 * deflector2 * deflector3 * deflectorPcw;
                    J_transform.block<2,3>(0,3) = deflector1 * deflector2 * deflector3 * deflectorRcw;

                    // cout << "Jpose:" << endl << J_transform << endl;
                }
            }
        }   
        //  debug
        double oldresiduals[2]={0};
        std::copy(residuals, residuals+2, oldresiduals);
        /*
        {
            double epsilon = 0.01;

            Eigen::Map<const Eigen::Vector3d> Pcw(parameters[2]);
            //  外面传的参数是四元数，但是增量是李代数
            Eigen::Map<const Eigen::Quaterniond> Qcw(parameters[2]+3);
            Eigen::Matrix3d Rcw = Qcw.toRotationMatrix();

            //  验证 J_interal
            // Eigen::Map<Eigen::Matrix<double, 2, 5, Eigen::RowMajor>> J_interal(jacobians[0]);
            // J_interal.setZero();
            Eigen::Matrix<double, 2, 5> J;
            J.setZero();
            for(int i=0; i<5; i++)
            {
                parameters[0][i] += epsilon;
                double alpha = parameters[0][0];
                double beta = parameters[0][1];
                double gamma = parameters[0][2];
                double u0 = parameters[0][3];
                double v0 = parameters[0][4];

                double k0 = parameters[1][0];
                double k1 = parameters[1][1];

                //  世界坐标
                Eigen::Matrix<double, 4, 1> Pw(Xw, Yw, 0, 1);
                //  Tcw
                Eigen::Matrix<double, 3, 4> Tcw;
                Tcw.block<3,3>(0,0) = Rcw;
                Tcw.block<3,1>(0,3) = Pcw;
                //  相机坐标
                Eigen::Vector3d Pc;
                Pc = Tcw * Pw;
                //  齐次相机坐标
                Eigen::Vector3d homo_Pc;
                homo_Pc = Pc;
                homo_Pc /= homo_Pc[2];
                double r2 = homo_Pc[0]*homo_Pc[0] + homo_Pc[1]*homo_Pc[1];
                double r4 = r2 * r2;
                //  畸变齐次相机坐标
                Eigen::Vector3d distor_homo_PC(0,0,1);
                distor_homo_PC[0] = homo_Pc[0]*(1 + k0*r2 + k1*r4 );
                distor_homo_PC[1] = homo_Pc[1]*(1 + k0*r2 + k1*r4 );
                //  内参矩阵
                Eigen::Matrix3d K;
                K << alpha, gamma, u0, 0, beta, v0, 0, 0, 1;
                //  像素坐标
                Eigen::Vector3d uv;
                uv = K * distor_homo_PC;

                residuals[0] = (uv[0] - u_mse);
                residuals[1] = (uv[1] - v_mse);

                J(0,i) = (residuals[0] - oldresiduals[0])/epsilon;
                J(1,i) = (residuals[1] - oldresiduals[1])/epsilon;

                parameters[0][i] -= epsilon;
            }
            // J(0,0) = 1;
            cout << "delta J:" << J << endl;
        }
        */
        /*
        {
            double epsilon = 0.01;

            Eigen::Map<const Eigen::Vector3d> Pcw(parameters[2]);
            //  外面传的参数是四元数，但是增量是李代数
            Eigen::Map<const Eigen::Quaterniond> Qcw(parameters[2]+3);
            Eigen::Matrix3d Rcw = Qcw.toRotationMatrix();

            //  验证 J_interal
            // Eigen::Map<Eigen::Matrix<double, 2, 5, Eigen::RowMajor>> J_interal(jacobians[0]);
            // J_interal.setZero();
            Eigen::Matrix<double, 2, 2> J;
            J.setZero();
            for(int i=0; i<2; i++)
            {
                parameters[1][i] += epsilon;
                double alpha = parameters[0][0];
                double beta = parameters[0][1];
                double gamma = parameters[0][2];
                double u0 = parameters[0][3];
                double v0 = parameters[0][4];

                double k0 = parameters[1][0];
                double k1 = parameters[1][1];

                //  世界坐标
                Eigen::Matrix<double, 4, 1> Pw(Xw, Yw, 0, 1);
                //  Tcw
                Eigen::Matrix<double, 3, 4> Tcw;
                Tcw.block<3,3>(0,0) = Rcw;
                Tcw.block<3,1>(0,3) = Pcw;
                //  相机坐标
                Eigen::Vector3d Pc;
                Pc = Tcw * Pw;
                //  齐次相机坐标
                Eigen::Vector3d homo_Pc;
                homo_Pc = Pc;
                homo_Pc /= homo_Pc[2];
                double r2 = homo_Pc[0]*homo_Pc[0] + homo_Pc[1]*homo_Pc[1];
                double r4 = r2 * r2;
                //  畸变齐次相机坐标
                Eigen::Vector3d distor_homo_PC(0,0,1);
                distor_homo_PC[0] = homo_Pc[0]*(1 + k0*r2 + k1*r4 );
                distor_homo_PC[1] = homo_Pc[1]*(1 + k0*r2 + k1*r4 );
                //  内参矩阵
                Eigen::Matrix3d K;
                K << alpha, gamma, u0, 0, beta, v0, 0, 0, 1;
                //  像素坐标
                Eigen::Vector3d uv;
                uv = K * distor_homo_PC;

                residuals[0] = (uv[0] - u_mse);
                residuals[1] = (uv[1] - v_mse);

                J(0,i) = (residuals[0] - oldresiduals[0])/epsilon;
                J(1,i) = (residuals[1] - oldresiduals[1])/epsilon;

                parameters[1][i] -= epsilon;
            }
            // J(0,0) = 1;
            cout << "delta J_distorted:" << J << endl;
        }*/
         /*
        {
            double epsilon = 0.00001;

            //  验证 J_interal
            // Eigen::Map<Eigen::Matrix<double, 2, 5, Eigen::RowMajor>> J_interal(jacobians[0]);
            // J_interal.setZero();
            Eigen::Matrix<double, 2, 7> J;
            J.setZero();
            //  计算位置的偏导
            for(int i=0; i<3; i++)
            {
                parameters[2][i] += epsilon;
                double alpha = parameters[0][0];
                double beta = parameters[0][1];
                double gamma = parameters[0][2];
                double u0 = parameters[0][3];
                double v0 = parameters[0][4];

                double k0 = parameters[1][0];
                double k1 = parameters[1][1];

                Eigen::Map<const Eigen::Vector3d> Pcw(parameters[2]);
                //  外面传的参数是四元数，但是增量是李代数
                Eigen::Map<const Eigen::Quaterniond> Qcw(parameters[2]+3);
                Eigen::Matrix3d Rcw = Qcw.toRotationMatrix();

                //  世界坐标
                Eigen::Matrix<double, 4, 1> Pw(Xw, Yw, 0, 1);
                //  Tcw
                Eigen::Matrix<double, 3, 4> Tcw;
                Tcw.block<3,3>(0,0) = Rcw;
                Tcw.block<3,1>(0,3) = Pcw;
                //  相机坐标
                Eigen::Vector3d Pc;
                Pc = Tcw * Pw;
                //  齐次相机坐标
                Eigen::Vector3d homo_Pc;
                homo_Pc = Pc;
                homo_Pc /= homo_Pc[2];
                double r2 = homo_Pc[0]*homo_Pc[0] + homo_Pc[1]*homo_Pc[1];
                double r4 = r2 * r2;
                //  畸变齐次相机坐标
                Eigen::Vector3d distor_homo_PC(0,0,1);
                distor_homo_PC[0] = homo_Pc[0]*(1 + k0*r2 + k1*r4 );
                distor_homo_PC[1] = homo_Pc[1]*(1 + k0*r2 + k1*r4 );
                //  内参矩阵
                Eigen::Matrix3d K;
                K << alpha, gamma, u0, 0, beta, v0, 0, 0, 1;
                //  像素坐标
                Eigen::Vector3d uv;
                uv = K * distor_homo_PC;

                residuals[0] = (uv[0] - u_mse);
                residuals[1] = (uv[1] - v_mse);

                J(0,i) = (residuals[0] - oldresiduals[0])/epsilon;
                J(1,i) = (residuals[1] - oldresiduals[1])/epsilon;

                parameters[2][i] -= epsilon;
            }
            //  计算李代数的偏导
            Eigen::Vector3d so3delta;
            so3delta.setZero();
            for(int i=3; i<6; i++)
            {
                // parameters[2][i] += epsilon;
                double alpha = parameters[0][0];
                double beta = parameters[0][1];
                double gamma = parameters[0][2];
                double u0 = parameters[0][3];
                double v0 = parameters[0][4];

                double k0 = parameters[1][0];
                double k1 = parameters[1][1];

                Eigen::Map<const Eigen::Vector3d> Pcw(parameters[2]);
                //  外面传的参数是四元数，但是增量是李代数
                Eigen::Map<const Eigen::Quaterniond> Qcw(parameters[2]+3);
                Eigen::Matrix3d Rcw = Qcw.toRotationMatrix();

                //  右乘扰动模型
                so3delta[i-3] += epsilon;
                Sophus::SO3 delta_SO3 = Sophus::SO3::exp(so3delta);
                Eigen::Matrix3d delta_R = delta_SO3.matrix();
                Rcw = Rcw * delta_R;
                so3delta[i-3] -= epsilon;

                //  世界坐标
                Eigen::Matrix<double, 4, 1> Pw(Xw, Yw, 0, 1);
                //  Tcw
                Eigen::Matrix<double, 3, 4> Tcw;
                Tcw.block<3,3>(0,0) = Rcw;
                Tcw.block<3,1>(0,3) = Pcw;
                //  相机坐标
                Eigen::Vector3d Pc;
                Pc = Tcw * Pw;
                //  齐次相机坐标
                Eigen::Vector3d homo_Pc;
                homo_Pc = Pc;
                homo_Pc /= homo_Pc[2];
                double r2 = homo_Pc[0]*homo_Pc[0] + homo_Pc[1]*homo_Pc[1];
                double r4 = r2 * r2;
                //  畸变齐次相机坐标
                Eigen::Vector3d distor_homo_PC(0,0,1);
                distor_homo_PC[0] = homo_Pc[0]*(1 + k0*r2 + k1*r4 );
                distor_homo_PC[1] = homo_Pc[1]*(1 + k0*r2 + k1*r4 );
                //  内参矩阵
                Eigen::Matrix3d K;
                K << alpha, gamma, u0, 0, beta, v0, 0, 0, 1;
                //  像素坐标
                Eigen::Vector3d uv;
                uv = K * distor_homo_PC;

                residuals[0] = (uv[0] - u_mse);
                residuals[1] = (uv[1] - v_mse);

                J(0,i) = (residuals[0] - oldresiduals[0])/epsilon;
                J(1,i) = (residuals[1] - oldresiduals[1])/epsilon;

            }            
            cout << "delta Jpose:"<< endl << J << endl;
        }*/
        return true;
    }
private:
    double u_mse, v_mse, Xw, Yw;
};

//  参数顺序：p q （位置，四元数）, 采用右乘的形式
class mypose: public ceres::LocalParameterization
{   
public:
    //  TODO 怎么确定这里的x第一个元素是实数还是虚数w, 根据外部决定
    //  这里的delta是李代数的增量，要把李代数的增量转化为四元数的增量
    virtual bool Plus(const double* x,
                    const double* delta,
                    double* x_plus_delta) const
    {
        // Eigen::Quaterniond q(x[0], x[1], x[2], x[3]);，这种赋值方式 x[0]代表实部w
        // Eigen::Map<const Eigen::Quaterniond> q(x); // 这种赋值方式 x[3]代表实部w
        
        //  因为数组的内存空间是连续的，因此对照搬移过来，w对应的位置就是数组中最后一个位置
        Eigen::Map<const Eigen::Vector3d> pos(x);
        Eigen::Map<const Eigen::Quaterniond> q(x+3);

        Eigen::Map<const Eigen::Vector3d> dpos(delta);
        //  TODO 你这里的增量有错！！！ 现在对了
        Eigen::Map<const Eigen::Quaterniond> dso3(delta+3);
        Eigen::Quaterniond dq;
        dq.w() = 1;
        dq.x() = dso3.x() / 2.0;
        dq.y() = dso3.y() / 2.0;
        dq.z() = dso3.z() / 2.0;

        Eigen::Map<Eigen::Vector3d> pos_plus_delta(x_plus_delta);
        Eigen::Map<Eigen::Quaterniond> q_plus_delta(x_plus_delta+3);
    
        pos_plus_delta = pos + dpos;
        //  TODO 这里的dq为什么乘右边呢？？？ 通篇的右乘和左乘要统一
        q_plus_delta = (q * dq).normalized();
        return true;
    }

    //  TODO: 其实还是不太懂这个含义, 7代表更新后的参数维数，6代表实际更新的参数增量的维数
    virtual bool ComputeJacobian(const double* x, double* jacobian) const
    {
        Eigen::Map<Eigen::Matrix<double, 7, 6>> J(jacobian);
        J.topRows<6>().setIdentity();
        J.bottomRows<1>().setZero();
        return true;
    }

    virtual int GlobalSize() const
    {
        return 7;
    }

    virtual int LocalSize() const
    {
        return 6;
    }
};

//  自动类型求导仿函数优化单应性矩阵
class myAutoCost
{
public:
    myAutoCost(double X, double Y, double u_mse, double v_mse)
    {
        m_X = X;
        m_Y = Y;
        m_umse = u_mse;
        m_vmse = v_mse;
    }
    template<typename T>
    bool operator()(const T* const parameters ,T* residual) const
    {
        T h11 = parameters[0];
        T h12 = parameters[1];
        T h13 = parameters[2];
        T h21 = parameters[3];
        T h22 = parameters[4];
        T h23 = parameters[5];
        T h31 = parameters[6];
        T h32 = parameters[7];
        T h33 = parameters[8];

        T s = h31*m_X + h32*m_Y + h33; 

        residual[0] = ((h11*m_X+h12*m_Y+h13)/s - m_umse);
        residual[1] = ((h21*m_X+h22*m_Y+h23)/s - m_vmse);
        return true;
    }
private:
    double m_X, m_Y, m_umse, m_vmse;
};

//  手动类型求导，优化单应性矩阵，雅可比确认没有问题(Failded!!!)
class myCostFunctor: public ceres::SizedCostFunction<2, 9>
{
public:
    myCostFunctor(double X, double Y, double u_mse, double v_mse)
        :m_X(X), m_Y(Y), m_umse(u_mse), m_vmse(v_mse) {}

    virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const
    {
        double h11 = parameters[0][0];
        double h12 = parameters[0][1];
        double h13 = parameters[0][2];
        double h21 = parameters[0][3];
        double h22 = parameters[0][4];
        double h23 = parameters[0][5];
        double h31 = parameters[0][6];
        double h32 = parameters[0][7];
        double h33 = parameters[0][8];

        double m = h11*m_X+h12*m_Y+h13;
        double n = h21*m_X+h22*m_Y+h23;
        double s = h31*m_X + h32*m_Y + h33; 

        residuals[0] = (m/s - m_umse) ;
        residuals[1] = (n/s - m_vmse) ;

        //  TODO 这里的雅可比一定需要赋值，否则会出错
        if(jacobians)
        {
            // if(jacobians[0])
            {
                jacobians[0][0] = jacobians[0][12] = m_X/s;
                jacobians[0][1] = jacobians[0][13] = m_Y/s;
                jacobians[0][2] = jacobians[0][14] = 1/s;
                jacobians[0][6]  = -m*m_X/(s*s);
                jacobians[0][15] = -n*m_X/(s*s);

                jacobians[0][7]  = -m*m_Y/(s*s);
                jacobians[0][16] = -n*m_Y/(s*s);

                jacobians[0][8] = -m/(s*s);
                jacobians[0][17] = -n/(s*s);

                jacobians[0][3] = jacobians[0][4] = jacobians[0][5] 
                    = jacobians[0][9] = jacobians[0][10] = jacobians[0][11] = 0;
            }
        }
    }

    static void show(double data)
    {
        std::cout << data << std::endl;
    }

    void check(double **parameters, double residuals[2], double jacobians[1][18])
    {
        double h11 = parameters[0][0];
        double h12 = parameters[0][1];
        double h13 = parameters[0][2];
        double h21 = parameters[0][3];
        double h22 = parameters[0][4];
        double h23 = parameters[0][5];
        double h31 = parameters[0][6];
        double h32 = parameters[0][7];
        double h33 = parameters[0][8];

        double m = h11*m_X+h12*m_Y+h13;
        double n = h21*m_X+h22*m_Y+h23;
        double s = h31*m_X + h32*m_Y + h33; 

        residuals[0] = (m/s - m_umse) ;
        residuals[1] = (n/s - m_vmse) ;

        double original0 = residuals[0];
        double original1 = residuals[1];

        //  TODO 这里的雅可比一定需要赋值，否则会出错
        if(jacobians)
        {
            jacobians[0][0] = jacobians[0][12] = m_X/s;
            jacobians[0][1] = jacobians[0][13] = m_Y/s;
            jacobians[0][2] = jacobians[0][14] = 1/s;
            jacobians[0][6]  = -m*m_X/(s*s);
            jacobians[0][15] = -n*m_X/(s*s);

            jacobians[0][7]  = -m*m_Y/(s*s);
            jacobians[0][16] = -n*m_Y/(s*s);

            jacobians[0][8] = -m/(s*s);
            jacobians[0][17] = -n/(s*s);

            jacobians[0][3] = jacobians[0][4] = jacobians[0][5] 
                = jacobians[0][9] = jacobians[0][10] = jacobians[0][11] = 0;
        }
        std::for_each(*(jacobians), *(jacobians)+18,  show);

        //  通过给一个微量来验证导数
        double epsilon = 1e-4;
        double newjacobians[1][18] = {0};
        for(int i=0; i<9; i++)
        {
            // double 
            parameters[0][i] += epsilon;

            h11 = parameters[0][0];
            h12 = parameters[0][1];
            h13 = parameters[0][2];
            h21 = parameters[0][3];
            h22 = parameters[0][4];
            h23 = parameters[0][5];
            h31 = parameters[0][6];
            h32 = parameters[0][7];
            h33 = parameters[0][8];

            m = h11*m_X+h12*m_Y+h13;
            n = h21*m_X+h22*m_Y+h23;
            s = h31*m_X + h32*m_Y + h33; 

            residuals[0] = (m/s - m_umse) ;
            residuals[1] = (n/s - m_vmse) ;

            newjacobians[0][i] = (residuals[0] - original0) / epsilon;
            newjacobians[0][i+9] = (residuals[1] - original1) / epsilon;

            parameters[0][i] -= epsilon;
        }
        std::cout << "varify jacobians" << std::endl;
        std::for_each(*(newjacobians), *(newjacobians)+18,  show);
    }
private:
    double m_X, m_Y, m_umse, m_vmse;
    // double k; // 单应矩阵的系数, 又感觉不用考虑这个 // TODO:
};

class my_calibr
{
public:
    typedef std::vector<cv::Point2f> pixelCordinate;
    typedef std::vector<cv::Point3d> wordlCordinate;
    typedef std::pair<pixelCordinate, wordlCordinate> mypair;
    typedef std::shared_ptr<mypair> m_pair_ptr;

public:
    my_calibr(ros::NodeHandle& n);

    void testRepro();
    //  官方例程
    void officialCalib();

    //  标定主线程
    void myCalibrationProcess();

    //  读取图片路径
    void readPhotosPath(std::string& imgsPath);

    //  棋盘角点检测
    void cornerDectection();

    //  计算单应性矩阵的初值
    void caculateHomoInit();

    //  计算归一化矩阵 并计算归一化坐标
    void caculateNormalizedMatrix
        (my_calibr::mypair& points, Eigen::Matrix3d &Nu, Eigen::Matrix3d& Nx);

    //  基于DLT，利用线性最小二乘SVD求H的初值
    void caculateHWithSVD(my_calibr::mypair& points, Eigen::Matrix3d& H_);

    //  非线性优化H
    bool nonlinearOptimizeH();

    //  求解内参矩阵
    void solveInertialMatrix();

    //  求解外参矩阵
    void caculateExternalMatrix();

    //  通过PnP求解外参
    void caculateExternalMatrixPnP();

    //  估计畸变系数
    void caculateDistortionFactor();

    //  debug显示图片
    void myShowPhoto(const char* name, cv::Mat& image, int time);

    //  检查homo矩阵的精度
    void checkHomo();

    //  当作曲线拟合一样求解
    void optimizedH();

    //  自动类型求导优化单应性矩阵
    void optimizeAutoH();

    //  重投影误差优化所有参数
    void optimizeReprojection();

    void myNLPoptimized();
private:
    std::string m_CALIBRATION_IMAGE_PATH;
    int m_points_per_row;
    int m_points_per_col;
    double m_square_size;

    std::vector<std::string> m_files_path;
    std::map<int, mypair > m_corre;
    std::vector<Eigen::Matrix3d> homoSets;
    std::vector<Eigen::Matrix<double,3,4>> externalMatrixSets;
    Eigen::Matrix3d K;
    Eigen::Matrix3d orignalK;
    double k0, k1;
};





#endif