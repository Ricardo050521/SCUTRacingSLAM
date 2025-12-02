#ifndef BASIC_STRUCT_HPP
#define BASIC_STRUCT_HPP

#include <iostream>
#include <algorithm>
#include <vector>
#include <string>
#include <set>
#include <queue>
#include <map>
#include <math.h>

#define PI 3.14159265358979323846
#define COST_INF 9999999

// ROS type

struct frenetbase
{
	double rx;
	double ry;
	double rs;
	double rtheta;
	double k;
	frenetbase(double a, double b, double c, double d,double e)
	{
		rx = a; 
		ry = b;
		rs = c;
		rtheta = d;
		k = e;
	}
};

struct frenetcone
{
	double rx;
	double ry;
	//double rs;
	double d;
	frenetcone(double a, double b, double c)
	{
		rx = a; 
		ry = b;
		d = c;
	}
};

#endif // BASIC_STRUCT_HPP