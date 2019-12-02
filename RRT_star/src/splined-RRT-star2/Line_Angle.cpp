#include <tuple>
#include "NodeClass.h"
#include <array>
#include "constants.h"
#include "Line_Angle.h"

//Purpose: get gradient k and interceptor b of the line between p1 and p2
//input: line starts from p1 to p2
//output : k : gradient of the line; b : intercept of the line
get_k_b_output Get_k_b(COORD p1, COORD p2)
{
	get_k_b_output k_and_b;
	k_and_b.k = (p2[1] - p1[1]) / (p2[0] - p1[0]);
	k_and_b.b = p1[1] - p1[0] * k_and_b.k;
	//std::tuple<double, double> k_and_b = std::make_tuple(k, b);
	return k_and_b;
}

//purpose: get the dot product of two unit vector u1 and u2
double dot_product(std::array<double, 2> & u1, std::array<double, 2> & u2)
{
	return u1[0] * u2[0] + u1[1] * u2[1];
}

//purpose:get the L2norm of a vector
double norm2d(std::array<double, 2> & u1)
{
	return sqrt(pow(u1[0], 2) + pow(u1[1], 2));
}

//purpose: get the cross product of two vectors
double cross_product(std::array<double, 2> & u1, std::array<double, 2> & u2)
{
	return u1[0] * u2[1] - u1[1] * u2[0];
}

//purpose:get the angle between two vectors
double Angle_Between_2dVector(std::array<double, 2> & u1, std::array<double, 2> & u2)
{
	double dot = dot_product(u1, u2);
	double norm1 = norm2d(u1);
	double norm2 = norm2d(u2);
	double mod_multiplication = norm1 * norm2;
	double no_orient_angle_rad = acos(dot / mod_multiplication);

	double cross = cross_product(u1, u2);

	if (cross < 0)
	{
		no_orient_angle_rad = -no_orient_angle_rad;
	}

	double orient_angle_rad = no_orient_angle_rad;
	double orient_angle_deg = orient_angle_rad * 180 / pi;

	return orient_angle_deg;
}

//purpose:get the distance between two points (point1, point2)
double dist(std::array<double, 2> point1, std::array<double, 2> point2)
{
	return sqrt(pow(point1[0] - point2[0], 2) + pow(point1[1] - point2[1], 2));
}

//purpose: change input angle range from [-180, 180] to [0, 360]
double change_angle_range(double angle)
{
	if (-180 < angle && angle < 0)
	{
		angle = angle + 360;
	}
	return angle;
}

//purpose:obtain unit vector goes from p1 to p2
std::array<double, 2> GetVector(COORD p1, COORD p2)
{
	std::array<double, 2> unit_vec;
	unit_vec[0] = (p2[0] - p1[0]) / sqrt(pow((p1[0] - p2[0]), 2) + pow((p1[1] - p2[1]), 2));// unit vector from qnear to qrand
	unit_vec[1] = (p2[1] - p1[1]) / sqrt(pow((p1[0] - p2[0]), 2) + pow((p1[1] - p2[1]), 2));

	return unit_vec;
}