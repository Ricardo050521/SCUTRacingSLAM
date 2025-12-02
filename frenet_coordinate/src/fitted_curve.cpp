#include "fitted_curve.hpp"
#include <iomanip>

namespace ns_frenet_coordinate
{
    Fitted::Fitted(geometry_msgs::PolygonStamped center_line )
    {
        std::vector<double> x_v;
        std::vector<double> y_v;
        for (int i = 0; i < center_line.polygon.points.size(); i ++)
        {
            x_v.push_back((double)center_line.polygon.points[i].x);
            y_v.push_back((double)center_line.polygon.points[i].y);
        }
        xx_v = Eigen::VectorXd::Map(&x_v[0], x_v.size());
        yy_v = Eigen::VectorXd::Map(&y_v[0], y_v.size());
    }

    Fitted::~Fitted()
	{
		std::cout << "Fitted END!" << std::endl;
	}

    void Fitted::run_algorithm()
    {
        cal_factors();
        cal_fittes_course();
    }

    void Fitted::cal_factors()
    {
        n_ = xx_v.size();
        a_ = 4;//三次多项式
        Phi = Eigen::MatrixXd::Zero(a_+1, n_);
        /**计算法方程*/
        for(int i = 0; i <= a_; i++)
        {
            if(i != a_)
            {
                for(int j = 0; j < n_; j++)
                {
                    Phi(i,j) = pow(xx_v(j),i);
                }
            }
            else
            {
                for(int j = 0; j < n_; j++)
                {
                    Phi(i,j) = yy_v(j);
                }
            }
        }
        A_ = _cal_primary_function_A(a_);
        B_ = _cal_primary_function_B(a_);
        //x_ = _cal_improve_squate(A_,B_);//改进的平方根法
        x_ = _cal_gaussin_L(A_,B_);//列主元高斯消去法
    }

    Eigen::MatrixXd Fitted::_cal_primary_function_A(const int a)
    {
        Eigen::MatrixXd A =  Eigen::MatrixXd::Zero(a, a);
        for(int i= 0; i < a; i++)
        {
            for(int j = i; j < a; j++)
            {
                double result = 0;
                for(int m = 0; m < n_; m++)
                {
                    result += Phi(i,m) * Phi(j,m);
                }
                if(i == j)
                {
                    A(i,j) = result;
                }
                else
                {
                    A(i,j) = A(j,i) = result;
                }
            }
        }
        return A;
    }

    Eigen::VectorXd Fitted::_cal_primary_function_B(const int a)
    {
        Eigen::VectorXd B =  Eigen::VectorXd::Zero(a);
        for(int i = 0; i < a; i++)
        {
            double result = 0;
            for(int m = 0; m < n_; m++)
            {
                result += Phi(i,m) * Phi(a,m);
            }
            B(i) = result;
        }
        return B;
    }
 
    Eigen::VectorXd Fitted::_cal_improve_squate(const Eigen::MatrixXd &A, const Eigen::VectorXd &B)
    {
        //改进平方根算法
        Eigen::MatrixXd t =  Eigen::MatrixXd::Zero(a_, a_);
        Eigen::MatrixXd l =  Eigen::MatrixXd::Zero(a_, a_);
        double d[a_] = {};
        int i, j, k;

        //求解过程
        for (i = 0;i < a_; i++)
        {  
            double sum1 = 0;
            for (j = 0; j <= i - 1; j++)
            {
                for (k = 0; k <= j - 1; k++)
                {
                    sum1 += t(i,k) * l(j,k);
                }
                t(i,j) = A(i,j) - sum1;
                l(i,j) = t(i,j) / d[j];
            }
            double sum2 = 0;
            for (k = 0; k <= i - 1; k++)
            {
                sum2 += t(i,k) * l(i,k);
            }
            d[i] = A(i,j) - sum2;
        }

        //打印输出中间矩阵D


        //Ly=b；
        double y[a_] = {};
        y[0] = B(0);
        for (i = 1; i < a_; i++)
        {
            double sum3 = 0;
            for (k = 0; k <= i - 1; k++)
            {
                sum3 += l(i,k) * y[k];
            }
            y[i] = B(i) - sum3;
        }



        //求解D(L转置)x=y
        Eigen::VectorXd x = Eigen::VectorXd::Zero(a_);
        x(a_ - 1) = y[a_ - 1] / d[a_ - 1];
        for (i = a_ - 2; i >= 0; i--)
        {
            double sum4 = 0;
            for (k = i + 1; k < a_; k++)
            {
                sum4 += l(k,i) * x(k);
            }
            x(i) = y[i] / d[i] - sum4;
        }
        return x;
    }

    Eigen::VectorXd Fitted::_cal_gaussin_L(const Eigen::MatrixXd &A, const Eigen::VectorXd &B)
    {
        int i,j,k;
        double d;
        int m = a_;
        int n = a_;
        double a[a_][a_ + 1];
        double temp[a_ + 1];
    
        for (i = 0; i < m; i++)
        {
            for (j = 0; j < n; j++)
            {
                a[i][j] = A(i,j);
            }
        }
            for (i = 0; i < m; i++)
            {
                a[i][n] = B(i);
            }

		for (k = 0; k < n - 1; k++) //找列主元最大值
		{
			double max = 0;
			int hang=0,num=0;
			for (i = k; i < n; i++)
			{
				if (fabs(a[i][k]) > max)
				{
					max = fabs(a[i][k]);
					hang = i;
				}
			 }
			if (a[hang][k] == 0)
			{
				std::cout << "无法计算" << std::endl;
			}
			if (k != hang) //换行
			{
				for (i = 0; i < m+1; i++)
				{
					temp[i] = a[k][i];
					a[k][i] = a[hang][i];
					a[hang][i] = temp[i];
				}
			}
			for (i = k + 1; i < m; i++) //消元
			{
				d = a[i][k] / a[k][k];
				for (j = 0; j < n + 1; j++)
				{
					a[i][j] = a[i][j] - d * a[k][j];
				}
			}
		}
		memset(temp, 0, (a_+1) * sizeof(double)); //将temp清0，准备存放解向量
		for (i = m-1; i >= 0; i--) //求解向量
		{		  
				d = 0;
			for (k = 0; k < n; k++)
			{
				d = d + temp[k] * a[i][k];
			}
			temp[i] = (a[i][n] - d) / a[i][i];
		}
        Eigen::VectorXd x = Eigen::VectorXd::Zero(a_);
        for(int i = 0; i < a_; i++)
        {
            x(i) = temp[i];
        }
        return x;
    }

    void Fitted::cal_fittes_course()//计算曲率
    {
        fitted_center_line.polygon.points.clear();
        for(int i = 1; i < n_; i++)
        {
            const double dx = xx_v(i) - xx_v(i - 1);
            const double dy = yy_v(i) - yy_v(i - 1);
            const double d = std::hypot(dx,dy);

            const int add_num = floor(d / 2.0);//TODO
            for(int j = 0; j < add_num; j ++)
            {
                geometry_msgs::Point32 new_p;
                new_p.x = xx_v(i - 1);
                new_p.x += 0.1* j * dx / d;
                new_p.y = x_(0) + x_(1) * new_p.x + x_(2) * pow(new_p.x, 2) + x_(3) * pow(new_p.x,3);
                fitted_center_line.polygon.points.push_back(new_p);
                // const double dy_ = calcd(new_p.x);
                // const double ddy_ = calcd(new_p.x);
                // double r = ddy_ / pow(1+ pow(dy_, 2),1.5 );//正方向盘打左，负方向打右
                // double yaw = atan(dy_);
                // new_p.x = r;
                // new_p.y = yaw;
                // r_.polygon.points.push_back(new_p);
                
            }
        }
    }

    double Fitted::calcd(double x)
    {
        double result = x_(1) + 2.0 * x_(2) * x + 3.0 * x_(3) * x * x;//一阶导数
		return result;
    }

    double Fitted::calcdd(double x)
	{
        double result = 2.0 * x_(2) + 6.0 * x_(3) * x;
		return result;
    }
}