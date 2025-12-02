    #include "cubic_spline_interpolation.hpp"


namespace ns_frenet_coordinate
{
    Spline::Spline(geometry_msgs::PolygonStamped center_line, int num)
    {
        std::vector<double> x_v;
        std::vector<double> y_v;
        for (int i = 0; i < num; i ++)
        {
            x_v.push_back((double)center_line.polygon.points[i].x);
            y_v.push_back((double)center_line.polygon.points[i].y);
        }
        
        xx_v = Eigen::VectorXd::Map(&x_v[0], x_v.size());
        yy_v = Eigen::VectorXd::Map(&y_v[0], y_v.size());
    }

    Spline::~Spline()
	{
		std::cout << "Spline END!" << std::endl;
	}



    void Spline::run_algorithm()
    {
        cal_factors();
        // cal_spline_course();
    }



    double Spline::cal_spline_course(int match_index,double s)
    {
		double dx_ = s - xx_v(match_index - 1);
		double y = a_(match_index - 1) + b_(match_index - 1) * dx_ + c_(match_index - 1) * dx_ * dx_ + d_(match_index - 1) * dx_ * dx_ * dx_;
        return y;
    }

    double Spline::calcd(int i,  double dx)
    {
        double result = b_(i - 1) + 2.0 * c_(i - 1) * dx + 3.0 * d_(i - 1) * dx * dx;//一阶导数

		return result;
    }

    double Spline::calcdd(int i, double dx)
	{
        double result = 2.0 * c_(i - 1) + 6.0 * d_(i - 1) * dx;
		return result;
    }

    void Spline::cal_factors()
    {
        n_ = xx_v.size();
        Eigen::VectorXd h(n_ - 1);

        for(int i = 1; i < n_; i ++)
        {
            h(i - 1) = xx_v(i) - xx_v(i-1);
            if(h(i - 1) == 0)
            {
                h(i - 1) = 1e-4;
            }
        }
        a_.resize(n_);
        for(int i = 0; i < n_; i++)
        {
            a_(i) = yy_v(i);
        }

        A_ = _cal_A(h);
        B_ =  _cal_B(h);

        c_ = _tdma(A_, B_);
        b_.resize(n_ - 1);
        d_.resize(n_ - 1);
        for(int i = 0; i < n_ - 1; ++i)
        {
            d_(i) = (c_(i + 1) - c_(i)) / (3.0 * h(i));
            b_(i) = (a_(i + 1) - a_(i)) / h(i) - h(i) * (c_(i + 1) + 2.0 * c_(i)) / 3.0;
        }
    }
    
    
    Eigen::MatrixXd Spline::_cal_A(Eigen::VectorXd h)
    {
        Eigen::MatrixXd A =  Eigen::MatrixXd::Zero(n_, n_);
		A(0, 0) = 1;
		for (int i = 0; i < n_ - 1; i++)
		{
			if (i != n_ - 2)
				A(i + 1, i + 1) = 2.0 * (h(i) + h(i + 1));
			A(i + 1, i) = h(i);
			A(i, i + 1) = h(i);
		}
		A(0, 1) = 0;
		A(n_ - 1, n_ - 1) = 1.0;
		A(n_ - 1, n_ - 2) = 0.0;
		return A;
    }

    Eigen::MatrixXd Spline::_cal_B(Eigen::VectorXd h)
	{
		Eigen::MatrixXd B = Eigen::MatrixXd::Zero(n_, 1);
		for (int i = 1; i < n_ - 1; i++)
		{
			B(i) = 3.0 * (a_(i + 1) - a_(i)) / h(i) - 3.0 * (a_(i) - a_(i - 1)) / h(i - 1);
		}

		return B;
	}

    	Eigen::VectorXd Spline::_tdma(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B)
	{
		// Thomas Algorithm(TDMA) solve linear equation
		Eigen::VectorXd x = Eigen::VectorXd::Zero(n_);
		double c[n_ - 1];
		c[0] = A(0, 1) / A(0, 0);
		x(0) = B(0) / A(0, 0);

		// chase
		for (size_t n = 1; n < n_; ++n)
		{
			double m = 1.0f / (A(n, n) - A(n, n - 1) * c[n - 1]);
			if (n != n_ - 1)
				c[n] = A(n, n + 1) * m;
			x(n) = (B(n) - A(n, n - 1) * x(n - 1)) * m;
		}

		// catch up
		for (size_t n = n_ - 1; n-- > 0;)
			x(n) = (x(n) - c[n] * x(n + 1)) / 2;
		return x;
	}



}

