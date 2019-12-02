#include <array>
#include "dubins.h"
#include "constants.h"
#include <math.h>
#include "NodeClass.h"
#include "Line_Angle.h"

using namespace std;

array<double, 3> dubins_LSL(double alpha, double beta, double d);
array<double, 3> dubins_LSR(double alpha, double beta, double d);
array<double, 3> dubins_RSL(double alpha, double beta, double d);
array<double, 3> dubins_RSR(double alpha, double beta, double d);
array<double, 3> dubins_RLR(double alpha, double beta, double d);
array<double, 3> dubins_LRL(double alpha, double beta, double d);

param dubins_core(array<double, 3> p1, array<double, 3> p2, double r)
{
	param param;
	//% the return parameter
	param.p_init = p1; //% the initial configuration
	param.seg_param = { 0, 0, 0 }; //% the lengths of the three segments
	param.r = r; //% model forward velocity / model angular velocity turning radius
	param.type = -1; //% path type.one of LSL, LSR, ...
	param.flag = 0;

	double dx = p2[0] - p1[0];
	double dy = p2[1] - p1[1];
	double D = sqrt(pow(dx, 2) + pow(dy, 2));
	double d = D / r;

	if (r <= 0)
	{
		param.flag = -1;
		return param;
	}
	double theta = fmod((atan2(dy, dx)) , (2 * pi));
	double alpha = fmod((p1[2] - theta) , (2 * pi));
	double beta =  fmod((p2[2] - theta) , (2 * pi));

	double best_word = -1;
	double best_cost = -1;
	array<array < double, 3>, 6> test_param;
	array<double, 3> a = dubins_LSL(alpha, beta, d);
	test_param[0] = a;
	array<double, 3> b = dubins_LSR(alpha, beta, d);
	test_param[1] = b;
	array<double, 3> c = dubins_RSL(alpha, beta, d);
	test_param[2] = c;
	array<double, 3> g = dubins_RSR(alpha, beta, d);
	test_param[3] = g;
	array<double, 3> e = dubins_RLR(alpha, beta, d);
	test_param[4] = e;
	array<double, 3> f = dubins_LRL(alpha, beta, d);
	test_param[5] = f;

	for (int i = 0; i < 6; i++)
	{
		if (fabs(test_param[i][0] - (-1)) < 10e-5)
		{
			double cost = test_param[i][0] + test_param[i][1] + test_param[i][2];
			if ((cost < best_cost) || (fabs(best_cost - (-1)) < 10e-5))
			{
				best_word = i;
				best_cost = cost;
				param.seg_param = test_param[i];
				param.type = i;
			}
		}
	}

	if (fabs(best_word - (-1)) < 10e-5)
	{
		param.flag = -2;
		return param;
	}
	else
	{
		return param;
	}
}

array<double, 3> dubins_LSL(double alpha, double beta, double d)
{
	array<double, 3> param;
	double tmp0 = d + sin(alpha) - sin(beta);
	double p_squared = 2.0 + d*d - (2.0*cos(alpha-beta)) + (2.0*d*(sin(alpha) - sin(beta)));
	if (p_squared < 0)
	{
		param = { -1, -1, -1 };
		return param;
	}
	else
	{
		double tmp1 = atan2((cos(beta) - cos(alpha)), tmp0);
		double t = fmod((-alpha + tmp1), 2 * pi);
		double p = sqrt(p_squared);
		double q = fmod((beta - tmp1), 2 * pi);
		param[0] = t;
		param[1] = p;
		param[2] = q;
		return param;
	}
}

array<double, 3> dubins_LSR(double alpha, double beta, double d)
{
	array<double, 3> param;
	//double tmp0 = d + sin(alpha) - sin(beta);
	double p_squared = 2.0 + d * d + (2.0 * cos(alpha - beta)) + (2.0 * d * (sin(alpha) + sin(beta)));
	if (p_squared < 0)
	{
		param = { -1, -1, -1 };
		return param;
	}
	else
	{
		double p = sqrt(p_squared);
		double tmp2 = atan2((-cos(alpha) - cos(beta)), (d + sin(alpha) + sin(beta))) - atan2(-2.0, p);
		double t = fmod((-alpha + tmp2), 2 * pi);
		double q = fmod((-fmod((beta), 2 * pi) + tmp2), 2 * pi);

		param[0] = t;
		param[1] = p;
		param[2] = q;
		return param;
	}
}

array<double, 3> dubins_RSL(double alpha, double beta, double d)
{
	array<double, 3> param;
	//double tmp0 = d + sin(alpha) - sin(beta);
	double p_squared = (d * d) - 2.0 + (2 * cos(alpha - beta)) - (2 * d * (sin(alpha) + sin(beta)));
	if (p_squared < 0)
	{
		param = { -1, -1, -1 };
		return param;
	}
	else
	{
		double p = sqrt(p_squared);
		double tmp2 = atan2((cos(alpha) + cos(beta)), (d - sin(alpha) - sin(beta))) - atan2(2.0, p);
		double t = fmod((alpha - tmp2), 2 * pi);
		double q = fmod((beta - tmp2), 2 * pi);

		param[0] = t;
		param[1] = p;
		param[2] = q;
		return param;
	}
}

array<double, 3> dubins_RSR(double alpha, double beta, double d)
{
	array<double, 3> param;
	double tmp0 = d - sin(alpha) + sin(beta);	
	double p_squared = 2.0 + (d * d) - (2.0 * cos(alpha - beta)) + (2.0 * d * (sin(beta) - sin(alpha)));
	if (p_squared < 0)
	{
		param = { -1, -1, -1 };
		return param;
	}
	else
	{
		double tmp1 = atan2((cos(alpha) - cos(beta)), tmp0);
		double t = fmod((alpha - tmp1), 2.0 * pi);
		double p = sqrt(p_squared);		
		double q = fmod((-beta + tmp1), 2.0 * pi);

		param[0] = t;
		param[1] = p;
		param[2] = q;
		return param;
	}
}

array<double, 3> dubins_RLR(double alpha, double beta, double d)
{
	array<double, 3> param;
	double tmp_rlr = (6.0 - d * d + 2.0 * cos(alpha - beta) + 2.0 * d * (sin(alpha) - sin(beta))) / 8.0;
	if (fabs(tmp_rlr) > 1)
	{
		param = { -1, -1, -1 };
		return param;
	}
	else
	{
		double p = fmod((2.0 * pi - acos(tmp_rlr)), 2 * pi);
		double t = fmod((alpha - atan2(cos(alpha) - cos(beta), d - sin(alpha) + sin(beta)) + fmod(p / 2.0, 2.0 * pi)), 2.0 * pi);
		double q = fmod((alpha - beta - t + fmod(p, 2.0 * pi)), 2.0 * pi);

		param[0] = t;
		param[1] = p;
		param[2] = q;
		return param;
	}
}

array<double, 3> dubins_LRL(double alpha, double beta, double d)
{
	array<double, 3> param;
	double tmp_lrl = (6.0 - d * d + 2.0 * cos(alpha - beta) + 2.0 * d * (-sin(alpha) + sin(beta))) / 8.0;
	if (fabs(tmp_lrl) > 1)
	{
		param = { -1, -1, -1 };
		return param;
	}
	else
	{
		double p = fmod((2 * pi - acos(tmp_lrl)), 2 * pi);
		double t = fmod((-alpha - atan2(cos(alpha) - cos(beta), d + sin(alpha) - sin(beta)) + p / 2.0), 2.0 * pi);
		double q = fmod((fmod(beta, 2.0 * pi) - alpha - t + fmod(p, 2.0 * pi)), 2.0 * pi);

		param[0] = t;
		param[1] = p;
		param[2] = q;
		return param;
	}
}

//Purpose: get the dubins curve length
//return: dubins curve length
double dubins_curve_simple(NodeClass point1, NodeClass point2, double TurnRadius)
{
	array<double, 3> p1;
	array<double, 3> p2;

	double p1k = point1.lineafter[0]; // angle theta of
	double p2k = point2.lineafter[0]; // angle theta of

	double p1angle = atan(p1k) * 180 / pi;
	p1angle = change_angle_range(p1angle);

	double p2angle = atan(p2k) * 180 / pi;
	p2angle = change_angle_range(p2angle);

	p1 = { point1.coord[0], point1.coord[1], p1angle * pi / 180 };
	p2 = { point2.coord[0], point2.coord[1], p2angle * pi / 180 };

	param param = dubins_core(p1, p2, TurnRadius);
	double curve_length = (param.seg_param[0] + param.seg_param[1] + param.seg_param[2]) * param.r;
	return curve_length;
}