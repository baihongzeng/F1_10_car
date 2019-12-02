#include <array>
#include "NodeClass.h"
#include "curve.h"
#include "constants.h"
#include "Line_Angle.h"

//Purpose:obtain curve properties including the gradient of curve, curve length
//FORMER "CURVE_LEN" IN MATLAB VERSION
curve_property curve_prop(COORD B0, COORD B1, COORD B2, COORD B3, COORD E0, COORD E1, COORD E2, COORD E3, std::array <double, t_size> t)
{
	
	curve_property curve_prop = { 0, 0, 0, 0, 0, 0};
	//std::array <double, t_size> Bcurve_x_prime;
	//std::array <double, t_size> Bcurve_y_prime;
	//std::array <double, t_size> Ecurve_x_prime;
	//std::array <double, t_size> Ecurve_y_prime;
	double dt = 1.0 / (t_size - 1.0);
	std::array <double, t_size> step_length_before;
	std::array <double, t_size> step_length_after;
	//double Bcurve_length = 0.0;
	//double Ecurve_length = 0.0;
	for (int i = 0; i < t_size; i++)
	{
		curve_prop.Bcurve_x_prime[i] = (-3) * B0[0] * pow(1 - t[i], 2) + 3 * B1[0] * (3 * pow(t[i], 2) - 4 * t[i] + 1) + 3 * B2[0] * ((-3) * pow(t[i], 2) + 2 * t[i]) + 3 * B3[0] * pow(t[i], 2);
		curve_prop.Bcurve_y_prime[i] = (-3) * B0[1] * pow(1 - t[i], 2) + 3 * B1[1] * (3 * pow(t[i], 2) - 4 * t[i] + 1) + 3 * B2[1] * ((-3) * pow(t[i], 2) + 2 * t[i]) + 3 * B3[1] * pow(t[i], 2);

		curve_prop.Ecurve_x_prime[i] = (-3) * E0[0] * pow(1 - t[i], 2) + 3 * E1[0] * (3 * pow(t[i], 2) - 4 * t[i] + 1) + 3 * E2[0] * ((-3) * pow(t[i], 2) + 2 * t[i]) + 3 * E3[0] * pow(t[i], 2);
		curve_prop.Ecurve_y_prime[i] = (-3) * E0[1] * pow(1 - t[i], 2) + 3 * E1[1] * (3 * pow(t[i], 2) - 4 * t[i] + 1) + 3 * E2[1] * ((-3) * pow(t[i], 2) + 2 * t[i]) + 3 * E3[1] * pow(t[i], 2);

		step_length_before[i] = sqrt(pow(curve_prop.Bcurve_x_prime[i], 2) + pow(curve_prop.Bcurve_y_prime[i], 2)) * dt;
		step_length_after[i] = sqrt(pow(curve_prop.Ecurve_x_prime[i], 2) + pow(curve_prop.Ecurve_y_prime[i], 2)) * dt;
		curve_prop.Bcurve_length = curve_prop.Bcurve_length + step_length_before[i];
		curve_prop.Ecurve_length = curve_prop.Ecurve_length + step_length_after[i];
	}
	return { curve_prop.Bcurve_x_prime, curve_prop.Bcurve_y_prime, curve_prop.Ecurve_x_prime, curve_prop.Ecurve_y_prime, curve_prop.Bcurve_length, curve_prop.Ecurve_length };
}

//Purpose: get 2d cubic bezier curve
coord_2d cubicbezier2d(COORD p0, COORD p1, COORD p2, COORD p3, std::array <double, t_size> t)
{
	//std::array <double, t_size> x;
	//std::array <double, t_size> y;
	coord_2d x_and_y = {0,0};
	for (int i = 0; i < t_size; i++)
	{
		x_and_y.x[i] = p0[0] * pow((1 - t[i]), 3) + 3 * p1[0] * t[i] * pow((1 - t[i]), 2) + 3 * p2[0] * pow(t[i], 2) * (1 - t[i]) + p3[0] * pow(t[i], 3);
		x_and_y.y[i] = p0[1] * pow((1 - t[i]), 3) + 3 * p1[1] * t[i] * pow((1 - t[i]), 2) + 3 * p2[1] * pow(t[i], 2) * (1 - t[i]) + p3[1] * pow(t[i], 3);
	}
	return { x_and_y.x, x_and_y.y };
}

//Purpose: get the curvature of cubic bezier curve along the curve and sum them up.
//return: the summation of curvature
double cal_curvature(COORD p0, COORD p1, COORD p2, COORD p3, std::array <double, t_size> t, std::array <double, t_size> x_p, std::array <double, t_size> y_p)
{
	std::array <double, t_size> x_pp;
	std::array <double, t_size> y_pp;
	std::array <double, t_size> K;
	double sum_K = 0.0;
	for (int i = 0; i < t_size; i++)
	{
		x_pp[i] = p0[0] * 6 * (1 - t[i]) + p1[0] * 3 * (6 * t[i] - 4) + p2[0] * 3 * (2 - 6 * t[i]) + p3[0] * 6 * t[i];
		y_pp[i] = p0[1] * 6 * (1 - t[i]) + p1[1] * 3 * (6 * t[i] - 4) + p2[1] * 3 * (2 - 6 * t[i]) + p3[1] * 6 * t[i];

		K[i] = fabs(x_p[i] * y_pp[i] - y_p[i] * x_pp[i]) / pow((pow(x_p[i], 2) + pow(y_p[i], 2)), 1.5);
		sum_K = sum_K + K[i];
	}

	return sum_K;
}

//Purpose: get the maximum allowable angle
//return: gamma_max
double cal_gamma_max_from_d(double & c4, double & kmax, double & stepsize)
{
	double gamma_max = 0;
	double a = kmax * stepsize / c4;
	double b = 1;
	double c = -(kmax * stepsize / c4);
	const int root_number = 2;
	std::array<double, root_number> root = { (-b + sqrt(pow(b, 2) - 4 * a * c)) / (2 * a) , (-b - sqrt(pow(b, 2) - 4 * a * c)) / (2 * a) }; //roots of quadratic equation, where m=sin(x)
	for (int i = 0; i < root_number; i++)
	{
		if (root[i] >= -1 && root[i] <= 1)
		{
			gamma_max = 2 * asin(root[i]) * 180 / pi;
		}
	}

	return gamma_max;
}

//Purpose: get the coordinates of eight control points for calculating one cubic bezier curve
eight_points_outputs cal_eight_control_points(COORD W2, std::array<double, 2> u1, std::array<double, 2> u2, double c2, double c3, double beta, double d)
{
	//% define the constant values
	double hb = c3 * d;
	double he = c3 * d;
	double gb = c2 * c3 * d;
	double ge = c2 * c3 * d;
	//% kb = (6 * c3 * cos(beta) * d) / (c2 + 4); % origin in No.6 paper
	//% ke = kb; % origin in No.6 paper
	double kb = fabs((6.0 * c3 * cos(beta) * d)) / (c2 + 4.0);// modified
	double ke = kb;// modified
	eight_points_outputs eight_points;
	eight_points.B0[0] = W2[0] + d * u1[0];
	eight_points.B0[1] = W2[1] + d * u1[1];
	eight_points.B1[0] = eight_points.B0[0] - gb * u1[0];
	eight_points.B1[1] = eight_points.B0[1] - gb * u1[1];
	eight_points.B2[0] = eight_points.B1[0] - hb * u1[0];
	eight_points.B2[1] = eight_points.B1[1] - hb * u1[1];

	eight_points.E0[0] = W2[0] + d * u2[0];
	eight_points.E0[1] = W2[1] + d * u2[1];
	eight_points.E1[0] = eight_points.E0[0] - ge * u2[0];
	eight_points.E1[1] = eight_points.E0[1] - ge * u2[1];
	eight_points.E2[0] = eight_points.E1[0] - he * u2[0];
	eight_points.E2[1] = eight_points.E1[1] - he * u2[1];

	std::array<double, 2> ud = GetVector(eight_points.B2, eight_points.E2);//unit vector from B2 to E2
	eight_points.B3[0] = eight_points.B2[0] + kb * ud[0];
	eight_points.B3[1] = eight_points.B2[1] + kb * ud[1];
	eight_points.E3[0] = eight_points.E2[0] - ke * ud[0];
	eight_points.E3[1] = eight_points.E2[1] - ke * ud[1];

	return eight_points;
}