#pragma once
#include <array>
#include "NodeClass.h"
using namespace std;

struct param
{
	array<double, 3> p_init;
	array<double, 3> seg_param;
	double r;
	int type;
	int flag;
};

//Purpose: get the dubins curve length
//return: dubins curve length
double dubins_curve_simple(NodeClass point1, NodeClass point2, double TurnRadius);