#pragma once
#include "NodeClass.h"
#include <vector>

struct gamma_property
{
	double gamma_deg = 0.0;
	int gamma_compare = 0;
};

// gamma compare : 1 means gamma_deg is within the range of gamma_max.
// 0                 goes out of the range of gamma_max.
// Purpose: detect whether the random points comply with the allowable angle constraint.
gamma_property AngleErrorDetection(std::array<double, 2> & u1, std::array<double, 2> & u2, double & gamma_max);

// Purpose: check whether the angle between the edge of qson - qparent and the
// edge of qparent - qparent.parent complies with gamma_max or not.If it is,
// return distance between qson and qparent, also qparent.index.
// return : Edge_Validity : 1 or 0. 1 means the input nodes comply with constraints
int Dist_Angle_Check_Single(NodeClass q_son, NodeClass q_parent, std::array<NodeClass, tree_size> & Tree, double gamma_max);

struct noCollisionLine_outputs
{
	double k;
	double b;
	int noCollision;
};

//Purpose: check if there is no collision between p1 (point) and obs
//return: 1: no colli, 0: colli.
int noCollisionPoint(COORD p1, std::vector <std::array<double, 3>> obs);

//Purpose: check if there is no collision between p1-p2 (line) and obs
//return: 1: no colli, 0: colli.
noCollisionLine_outputs noCollisionLine(COORD p1, COORD p2, int interpolation_num, std::vector <std::array<double, 3>> obs);

//Purpose: detect whether the needed step length is within the step size
//% input: W3: qnew, W2 : qnear, W1 : qnear's parent, d: required step length for constructing two curves.
//% Return :
//	% dcompare = 1 : required step length is shorter than the given fixed step length, valid qnew.
//	% dcompare = 0 : ... longer ..., invalid qnew.
int StepErrorDetection(COORD W1, COORD W2, COORD W3, double d);