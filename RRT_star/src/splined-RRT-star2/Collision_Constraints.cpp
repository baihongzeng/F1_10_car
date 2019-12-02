#include "NodeClass.h"
#include "Collision_Constraints.h"
#include "Line_Angle.h"
#include "constants.h"
#include <iostream>

// gamma compare : 1 means gamma_deg is within the range of gamma_max.
// 0                 goes out of the range of gamma_max.
// Purpose: detect whether the random points comply with the allowable angle constraint.
gamma_property AngleErrorDetection(std::array<double, 2> & u1, std::array<double, 2> & u2, double & gamma_max)
{
	gamma_property deg_comp;
	std::array<double, 2> reverse_u1 = { -u1[0], -u1[1] };
	deg_comp.gamma_deg = Angle_Between_2dVector(reverse_u1, u2);
	// self change u1[0] into - u1[0]
	// angle between(-u1) and u2, range here : [-180, 180] , anticlockwise is positive.
	// for the following code / output, valid range : [0, 360]

	if (0 <= deg_comp.gamma_deg && deg_comp.gamma_deg <= 180)
	{
		if (deg_comp.gamma_deg <= gamma_max)
		{	
			// gamma_deg = gamma_deg; % did not use since value didn't change
			deg_comp.gamma_compare = 1;
		}
	}
	else if (-180 < deg_comp.gamma_deg && deg_comp.gamma_deg < 0)
	{
		deg_comp.gamma_deg = deg_comp.gamma_deg + 360;
		if (360 - gamma_max < deg_comp.gamma_deg && deg_comp.gamma_deg <= 360)
		{
			deg_comp.gamma_compare = 1;
		}
	}
	else
	{
		deg_comp.gamma_compare = 0;
	}

	//return { deg_comp.gamma_deg, deg_comp.gamma_compare };
	return deg_comp;
}


// Purpose: check whether the angle between the edge of qson - qparent and the
// edge of qparent - qparent.parent complies with gamma_max or not.If it is,
// return distance between qson and qparent, also qparent.index.
// return : Edge_Validity : 1 or 0. 1 means the input nodes comply with constraints
int Dist_Angle_Check_Single(NodeClass q_son, NodeClass q_parent, std::array<NodeClass, tree_size> & Tree, double gamma_max)
{
	double k_parent = q_parent.lineafter[0]; // gradient of (qnear, qnear.parent) edge
	double angle_parent = atan(k_parent) * 180.0 / pi; // angle in degree
	angle_parent = change_angle_range(angle_parent); // adjust angle range from [-180, 180] to [0, 360]

	gamma_property gamma_deg_comp;
	std::array<double, 2> uvec_tree;
	std::array<double, 2> uvec_qrand;

	if (angle_parent == 0 && q_parent.coord[0] == Tree[0].coord[0] && q_parent.coord[1] == Tree[0].coord[1]) // it is the case of qstart
	{
		gamma_deg_comp.gamma_compare = 1;
	}
	else //allnodes except q_start
	{
		uvec_tree[0] = (Tree[q_parent.parent].coord[0] - q_parent.coord[0]) / sqrt(pow((Tree[q_parent.parent].coord[0] - q_parent.coord[0]), 2) + pow((Tree[q_parent.parent].coord[1] - q_parent.coord[1]), 2));// unit vector from W2 to W1
		uvec_tree[1] = (Tree[q_parent.parent].coord[1] - q_parent.coord[1]) / sqrt(pow((Tree[q_parent.parent].coord[0] - q_parent.coord[0]), 2) + pow((Tree[q_parent.parent].coord[1] - q_parent.coord[1]), 2));// unit vector from W2 to W1
		uvec_qrand[0] = (q_son.coord[0] - q_parent.coord[0]) / sqrt(pow((q_son.coord[0] - q_parent.coord[0]), 2) + pow((q_son.coord[1] - q_parent.coord[1]), 2));// unit vector from W2 to W3
		uvec_qrand[1] = (q_son.coord[1] - q_parent.coord[1]) / sqrt(pow((q_son.coord[0] - q_parent.coord[0]), 2) + pow((q_son.coord[1] - q_parent.coord[1]), 2));// unit vector from W2 to W3
		gamma_deg_comp.gamma_compare = 0;
		gamma_deg_comp = AngleErrorDetection(uvec_tree, uvec_qrand, gamma_max);
	}

	int Edge_Validity;
	if (gamma_deg_comp.gamma_compare && (q_son.coord[0] != q_parent.coord[0]))
	{
		Edge_Validity = 1;
	}
	else
	{
		Edge_Validity = 0;
	}

	return Edge_Validity;
}

//Purpose: check if there is no collision between p1 (point) and obs
//return: 1: no colli, 0: colli.
int noCollisionPoint(COORD p1, std::vector <std::array<double, 3>> obs)
{
	int noCollision = 1;
	if (obs.empty() != true)
	{
		int NumObs = obs.size();
		for (int Num = 0; Num < NumObs; Num++)
		{
			COORD p_obs = { obs[Num][0], obs[Num][1] };
			double dist_between_point_obs = dist(p1, p_obs);
			if (dist_between_point_obs < obs[Num][2])
			{
				noCollision = 0;
				break;
			}
		}
	}

	return noCollision;
}

//Purpose: check if there is no collision between p1-p2 (line) and obs
//return: 1: no colli, 0: colli.
noCollisionLine_outputs noCollisionLine(COORD p1, COORD p2, int interpolation_num, std::vector <std::array<double, 3>> obs)
{
	/*% input: line starts from p1 to p2
		%
		% interpolation_num : Num of interpolation points for collision detection
		% detect collision of linear lines
		%reverse collision check order of edges, since the furtherest point has the
		% highest probability to be in obstacles.*/

	noCollisionLine_outputs noColl = {0.0, 0.0, 0};
	get_k_b_output k_and_b = Get_k_b(p1, p2);
	noColl.k = k_and_b.k;
	noColl.b = k_and_b.b;

	if (p2[0] > p1[0])//%gradient is positive
	{
		double dx = (p2[0] - p1[0]) / (interpolation_num-1.0);
		for (int i = 0; i < interpolation_num; i++) //reverse collision check order of edges
		{
			double x = p2[0] + i * dx;
			double y = k_and_b.k * x + k_and_b.b;
			COORD p_current = { x, y };
			noColl.noCollision = noCollisionPoint(p_current, obs);//1, no collision; 0, collision
			if (noColl.noCollision == 0)
			{
				break;
			}
		}
	}
	else if (p2[0] < p1[0]) // gradient is negative
	{
		double dx = (p1[0] - p2[0]) / (interpolation_num - 1.0);//reverse collision check order of edges
		for (int i = 0; i < interpolation_num; i++)
		{
			double x = p1[0] + i * dx;
			double y = k_and_b.k * x + k_and_b.b;
			COORD p_current = { x, y };
			noColl.noCollision = noCollisionPoint(p_current, obs);//1, no collision; 0, collision
			if (noColl.noCollision == 0)
			{
				break;
			}
		}
	}

	return { noColl.k, noColl.b, noColl.noCollision };
}

//Purpose: detect whether the needed step length is within the step size
//% input: W3: qnew, W2 : qnear, W1 : qnear's parent, d: required step length for constructing two curves.
//% Return :
//	% dcompare = 1 : required step length is shorter than the given fixed step length, valid qnew.
//	% dcompare = 0 : ... longer ..., invalid qnew.
int StepErrorDetection(COORD W1, COORD W2, COORD W3, double d)
{
	int d_compare = 0;
	if (d > sqrt(pow((W1[0] - W2[0]), 2) + pow((W1[1] - W2[1]), 2)) || d > sqrt(pow((W3[0] - W2[0]), 2) + pow((W3[1] - W2[1]), 2)))
	{
		d_compare = 0;
	}
	else
	{
		d_compare = 1;//valid d
	}
	return d_compare;
}


