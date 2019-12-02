#pragma once
#include "constants.h"
#include <array>
#include <vector>
#include <math.h>

typedef std::array<double, 2> COORD;

struct BEcurve
{
	std::array<double, t_size> x {{ 0 }};
	std::array<double, t_size> y {{ 0 }};
	std::array<double, t_size> x_prime = {{ 0 }};
	std::array<double, t_size> y_prime = {{ 0 }};
	double length = 0.0;
	double sum_K = 0.0;
};

class NodeClass
{
public:
	COORD coord = {{ 0 }};
	int parent = -1;
	std::vector<int> son;
	COORD midcoord = coord;
	std::array<BEcurve, 2> curve;//already initialized
	std::array<double, 2>linebefore = {{ 0 }};
	std::array<double, 2>lineafter = {{ 0 }};
	double t_cost = 0;
	double s_cost = 0;
	double dubins_goal_dist = 0;
	double Astar_goal_dist = 0;
	double final_dubins_close_cost = 0;

	NodeClass(std::array<double, 2> coordinate);//constructor with parameters
	NodeClass(); //overloaded constructor

};