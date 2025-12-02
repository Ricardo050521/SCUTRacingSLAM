#ifndef  FITTED_CURVE_HPP
#define FITTED_CURVE_HPP

#include <iostream>
#include <vector>
#include <math.h>
#include "geometry_msgs/PolygonStamped.h"
#include "Eigen/Dense"

namespace ns_frenet_coordinate
{
    class Fitted
    {
        public:
            Fitted(geometry_msgs::PolygonStamped center_line);
            ~Fitted();
            
            void run_algorithm();
            void cal_factors();//计算法方程
            void cal_fittes_course();//计算插值
            geometry_msgs::PolygonStamped getpath() {return fitted_center_line;}
            // geometry_msgs::PolygonStamped getcurvature() {return r_;}
            double calcd(double x);
            // 计算二阶导
            double calcdd(double x);
        private:
            int n_;
            int a_ ;//几次多项式
            Eigen::VectorXd x_;//solve
            Eigen::MatrixXd A_;//法方程左边的系数矩阵
		    Eigen::VectorXd B_;//法方程右边的系数向量
            Eigen::MatrixXd Phi;//不同坐标下基函数的值

            Eigen::VectorXd xx_v;//已知点的x坐标
            Eigen::VectorXd yy_v;//已知点的y坐标

            geometry_msgs::PolygonStamped fitted_center_line;//最终路径
            // geometry_msgs::PolygonStamped r_;

            Eigen::MatrixXd _cal_primary_function_A(const int a);//计算法方程左边的系数
            Eigen::VectorXd _cal_primary_function_B(const int a);//计算法方程右边的矩阵
            Eigen::VectorXd _cal_improve_squate(const Eigen::MatrixXd &A, const Eigen::VectorXd &B);//求解法方程
            Eigen::VectorXd _cal_gaussin_L(const Eigen::MatrixXd &A, const Eigen::VectorXd &B);
    };
}

#endif