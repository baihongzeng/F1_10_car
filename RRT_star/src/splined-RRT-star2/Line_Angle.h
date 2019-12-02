#include <tuple>
#include <array>
#include "NodeClass.h"
struct get_k_b_output
{
	double k;
	double b;
};

//Purpose: get gradient k and interceptor b of the line between p1 and p2
//input: line starts from p1 to p2
//output : k : gradient of the line; b : intercept of the line
get_k_b_output Get_k_b(COORD p1, COORD p2);

//purpose: get the dot product of two unit vector u1 and u2
double dot_product(std::array<double, 2> & u1, std::array<double, 2> & u2);

//purpose:get the L2norm of a vector
double norm2d(std::array<double, 2> & u1);

//purpose: get the cross product of two vectors
double cross_product(std::array<double, 2> & u1, std::array<double, 2> & u2);

//purpose:get the angle between two vectors
double Angle_Between_2dVector(std::array<double, 2> & u1, std::array<double, 2> & u2);

//purpose:get the distance between two points (point1, point2)
double dist(std::array<double, 2> point1, std::array<double, 2> point2);

//purpose: change input angle range from [-180, 180] to [0, 360]
double change_angle_range(double angle);

//purpose:obtain unit vector goes from p1 to p2
std::array<double, 2> GetVector(COORD p1, COORD p2);