#pragma once
#include <array>
#include "NodeClass.h"
#include "constants.h"

struct coord_2d
{
	std::array <double, t_size> x;
	std::array <double, t_size> y;
};

struct curve_property
{
	std::array <double, t_size> Bcurve_x_prime;
	std::array <double, t_size> Bcurve_y_prime;
	std::array <double, t_size> Ecurve_x_prime;
	std::array <double, t_size> Ecurve_y_prime;
	double Bcurve_length;
	double Ecurve_length;
};

//Purpose: get the maximum allowable angle
//return:gamma_max
double cal_gamma_max_from_d(double& c4, double& kmax, double& stepsize);

//Purpose:obtain curve properties including the gradient of curve, curve length
//FORMER "CURVE_LEN" IN MATLAB VERSION
curve_property curve_prop(COORD B0, COORD B1, COORD B2, COORD B3, COORD E0, COORD E1, COORD E2, COORD E3, std::array <double, t_size> t);

//Purpose: get 2d cubic bezier curve
coord_2d cubicbezier2d(COORD p0, COORD p1, COORD p2, COORD p3, std::array <double, t_size> t);

//Purpose: get the curvature of cubic bezier curve along the curve and sum them up.
//return: the summation of curvature
double cal_curvature(COORD p0, COORD p1, COORD p2, COORD p3, std::array <double, t_size> t, std::array <double, t_size> x_p, std::array <double, t_size> y_p);

struct eight_points_outputs
{
	COORD B0 = {{ 0 }};
	COORD B1 = {{ 0 }};
	COORD B2 = {{ 0 }};
	COORD B3 = {{ 0 }};
	COORD E0 = {{ 0 }};
	COORD E1 = {{ 0 }};
	COORD E2 = {{ 0 }};
	COORD E3 = {{ 0 }};
};

//Purpose: get the coordinates of eight control points for calculating one cubic bezier curve
eight_points_outputs cal_eight_control_points(COORD W2, std::array<double, 2> u1, std::array<double, 2> u2, double c2, double c3, double beta, double d);