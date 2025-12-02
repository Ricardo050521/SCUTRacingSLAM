#ifndef CUBIC_SPLINE_INTERPOLATION_HPP
#define CUBIC_SPLINE_INTERPOLATION_HPP

#include <iostream>
#include <vector>
#include <math.h>
#include "geometry_msgs/PolygonStamped.h"
#include "Eigen/Dense"

namespace ns_frenet_coordinate
{
    class Spline
	{
	public:
		Spline(geometry_msgs::PolygonStamped center_line, int num);
		~Spline();

        void run_algorithm();

        double cal_spline_course(int match_index,double s);

        geometry_msgs::PolygonStamped getpath() {return spline_center_line;}
		geometry_msgs::PolygonStamped getcurvature() {return r_;}
        void cal_factors();
		//计算一阶导
		double calcd(int i, double dx_);
		// 计算二阶导
		double calcdd(int i, double dx_);



	private:
		int n_;
		Eigen::VectorXd a_;
		Eigen::VectorXd b_;
		Eigen::VectorXd c_;
		Eigen::VectorXd d_;
		Eigen::MatrixXd A_;
		Eigen::MatrixXd B_;

        Eigen::MatrixXd dx_;

        Eigen::VectorXd xx_v;
        Eigen::VectorXd yy_v;

        Eigen::VectorXd rk_;

        geometry_msgs::PolygonStamped spline_center_line;
		geometry_msgs::PolygonStamped spline_center_line2;
		geometry_msgs::PolygonStamped r_;
		//private method
		Eigen::MatrixXd _cal_A(Eigen::VectorXd h);
		Eigen::MatrixXd _cal_B(Eigen::VectorXd h);
		Eigen::VectorXd _tdma(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B);
		int _search_index(double x);
	};

}
#endif