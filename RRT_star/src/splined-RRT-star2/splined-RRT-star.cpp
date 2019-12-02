
#include <math.h>
#include "splined-RRT-star.h"
#include "Collision_Constraints.h"
#include "Line_Angle.h"
#include <algorithm>
#include "curve.h"
#include <fstream>
#include "plot.h"
#include <string>
#include "kdtree.h"
#include <random>
#include "dubins.h"
#include <float.h>

//purpose: find q_new which locates between q_rand and q_near
NodeClass FindQnew(NodeClass & q_rand, NodeClass & q_near, double & min_dist, double & stepsize)
{
	NodeClass q_new({ 0,0 });
	if (min_dist > stepsize)
	{
		std::array<double, 2> unit_vec = GetVector(q_near.coord, q_rand.coord);
		q_new.coord[0] = q_near.coord[0] + unit_vec[0] * stepsize;
		q_new.coord[1] = q_near.coord[1] + unit_vec[1] * stepsize;
	}
	else
	{
		q_new.coord = q_rand.coord;
	}

	return q_new;
}

//purpose:output the nearest neighbor that comply with the constraints
// index_mat will not contain Tree(1)
// return:
// dis_ang_ind(1, :) : 1st row, dist between qrand and allnods
// dis_ang_ind(2, :) : 2nd row, angle between the edge of(qrand, qnear) and
//(qnear, qnear.parents)
// dis_ang_ind(3, :); % index: the index of qnear in the tree
// Num_neibors_of_qrand : # of columns in dis_ang_ind matrix
Near_Neighbor_outputs Nearest_Neighbor(NodeClass& q_rand, std::array<NodeClass, tree_size> & Tree, std::vector<int>& index_mat, double& gamma_max, int kdtree_built_flag, int rewire_flag)
{
	int index_mat_size = index_mat.size();
	for (int index_find = 0; index_find < index_mat_size; index_find++)
	{
		if (index_mat[index_find] == 0)
		{
			std::cout << "error in Nearest Neighbor, Nearest_Neighbor should not connect q_rand with Tree(1),i.e. q_start" << std::endl;
		}
	}

	int i = 0;
	int have_satisfied_angle_flag = 0;
	Near_Neighbor_outputs NN_outputs;
	//dis_ang_ind_str dis_ang_ind_single = { 0.0, 0.0, 0 };
	//NN_outputs.dis_ang_ind.push_back(dis_ang_ind_single);
	//std::vector<dis_ang_ind_str> dis_ang_ind;


	for (int j = 0; j < (int)index_mat.size(); j++)//get the angle difference between two points and save the valid one in dis_ang_ind
	{
		if (q_rand.coord[0] == Tree[index_mat[j]].coord[0])// avoid when edge between qrandand allnodes is vertical, i.e.avoiding gradient = Inf situation.
		{
			continue;
		}
		//std::array<double, 2> uvec_tree;
		//std::array<double, 2> uvec_qrand;
		std::array<double, 2> uvec_tree = GetVector(Tree[index_mat[j]].coord, Tree[Tree[index_mat[j]].parent].coord);
		std::array<double, 2> uvec_qrand = GetVector(Tree[index_mat[j]].coord, q_rand.coord);

		gamma_property gamma_prop = AngleErrorDetection(uvec_tree, uvec_qrand, gamma_max);

		if (gamma_prop.gamma_compare == 1) // if angle within gamma_max, add
		{
			i++;
			double one = dist(q_rand.coord, Tree[index_mat[j]].coord);
			dis_ang_ind_str dis_ang_ind_single = { one, gamma_prop.gamma_deg, index_mat[j] };
			NN_outputs.dis_ang_ind.push_back(dis_ang_ind_single);
			have_satisfied_angle_flag++;
		}

		
	}

	// when no connection between allnodesand qrand comply with constraints, output this.
	if (have_satisfied_angle_flag == 0)
	{
		NN_outputs.Num_neibors_of_qrand = 0;
		return NN_outputs;
		//return{ NN_outputs.dis_ang_ind, NN_outputs.Num_neibors_of_qrand };
	}
	NN_outputs.Num_neibors_of_qrand = NN_outputs.dis_ang_ind.size();

	for (int m = 0; m < NN_outputs.Num_neibors_of_qrand; m++)
	{
		for (int n = 0; n < (NN_outputs.Num_neibors_of_qrand - m); n++)
		{
			if(n>0)
			{
				if (NN_outputs.dis_ang_ind[(int)n-1].dist > NN_outputs.dis_ang_ind[n].dist) //swap
				{
					dis_ang_ind_str temp = NN_outputs.dis_ang_ind[n ];
					NN_outputs.dis_ang_ind[n ] = NN_outputs.dis_ang_ind[(int)n-1];
					NN_outputs.dis_ang_ind[(int)n-1] = temp;
				}
			}

		}
	}
	return NN_outputs;
	//return{ NN_outputs.dis_ang_ind, NN_outputs.Num_neibors_of_qrand };
}

//purpose:get the curve starting from W1, turns at W2, and ends at W3.
mainCommonCurve_outputs mainCommonCurve(COORD & W1, COORD& W2, COORD& W3, double& kmax, double& gamma_max, std::array <double, t_size>& t, std::vector <std::array<double, 3>>& obs, int mode, int & interpolation_num)
{
	//initialization
	mainCommonCurve_outputs mCC;
	mCC.curve_valid_ind = 1;
	int gamma_compare = 0;


	std::array <double, 2> u1 = GetVector(W2, W1);//unit vector from W2 to W1
	std::array <double, 2> u2 = GetVector(W2, W3);//unit vector from W2 to W1

	//exit function when angle larger than allowable angle
	gamma_property gamma_prop = AngleErrorDetection(u1, u2, gamma_max);
	if (gamma_prop.gamma_compare == 0)
	{
		mCC.curve_valid_ind = 0;
		if (mode == 0)
		{
			std::cout << "turning angle has exceeded the maximum allowable angle (gamma_max), curve unachievable" << std::endl;
		}
		return mCC;
	}

	double gamma = gamma_prop.gamma_deg * pi / 180.0;//input: degree, output: radian,% allowable angle
	double beta = gamma / 2.0;//degree, angle between (-u1) and ud
	double d = (c4 * sin(beta)) / (kmax * pow(cos(beta), 2));//minimum step length required

	//% when the gradient of the former lineafterand the current connection
	//	% between qnewand qnear is the same, d = 0, no curve exist, only two linear
	//	% line exist, get into this condition.

	//straight line, no curve
	if (fabs(d) <= 10e-5)
	{
		noCollisionLine_outputs noColl_after = noCollisionLine(W2, W3, interpolation_num, obs);
		mCC.k_after = noColl_after.k;
		mCC.b_after = noColl_after.b;
		if (noColl_after.noCollision == 0)//examine Lineafter first
		{
			mCC.curve_valid_ind = 0;
			if (mode == 0)
			{
				std::cout << "curve is straight,Linear Lineafter Collision, curve unachievable" << std::endl;
			}
			return mCC;
		}

		noCollisionLine_outputs noColl_before = noCollisionLine(W1, W2, interpolation_num, obs);
		mCC.k_before = noColl_before.k;
		mCC.b_before = noColl_before.b;
		if (noColl_before.noCollision == 0)//then examine Linebefore
		{
			mCC.curve_valid_ind = 0;
			if (mode == 0)
			{
				std::cout << "curve is straight, Linear Linebefore Collision, curve unachievable" << std::endl;
			}
			return mCC;
		}

		if (noColl_before.noCollision == 1 && noColl_after.noCollision == 1)//no collision between obs and two lines
		{
			mCC.curve_valid_ind = 1;
			mCC.Bcurve.x[0] = W2[0];
			mCC.Bcurve.y[0] = W2[1];
			mCC.Ecurve.x[0] = W2[0];
			mCC.Ecurve.y[0] = W2[1];
			mCC.sum_K = 0;
			COORD curve_end_B = W2;
			COORD curve_end_E = W2;
			double curve_length = 0.0;
			mCC.cur_line_length = dist(W1, curve_end_B) + dist(curve_end_E, W3) + curve_length;
			return mCC;
		}
	}
	else //d>10e_5, curve has length
	{/*%% has curve, not straight line
		% exit function when d > distance between W1 - W2, W2 - W3, i.e.curvature constraint not met*/
		int d_compare = StepErrorDetection(W1, W2, W3, d);
		if (d_compare == 0)
		{
			mCC.curve_valid_ind = 0;
			if (mode == 0)
			{
				std::cout << "Required step length (d) has exceeded the distance between W1-W2, W2-W3, curve unachievable" << std::endl;
			}
			return mCC;
		}

		eight_points_outputs eight_points = cal_eight_control_points(W2, u1, u2, c2, c3, beta, d);
		coord_2d Bcurve_x_y = cubicbezier2d(eight_points.B0, eight_points.B1, eight_points.B2, eight_points.B3, t);
		mCC.Bcurve.x = Bcurve_x_y.x;
		mCC.Bcurve.y = Bcurve_x_y.y;

		coord_2d Ecurve_x_y = cubicbezier2d(eight_points.E0, eight_points.E1, eight_points.E2, eight_points.E3, t);
		mCC.Ecurve.x = Ecurve_x_y.x;
		mCC.Ecurve.y = Ecurve_x_y.y;

		//detect collision
		//detect collision of linear lineafter
		//COORD Ecurve_start = { mCC.Ecurve.x[0], mCC.Ecurve.y[0] };
		noCollisionLine_outputs noColl_after = noCollisionLine({ mCC.Ecurve.x[0], mCC.Ecurve.y[0] }, W3, interpolation_num, obs);
		mCC.k_after = noColl_after.k;
		mCC.b_after = noColl_after.b;
		if (noColl_after.noCollision == 0)//then examine Linebefore
		{
			mCC.curve_valid_ind = 0;
			if (mode == 0)
			{
				std::cout << "curve is not straight, Linear Lineafter Collision, curve unachievable" << std::endl;
			}
			return mCC;
		}

		double d_length_t = (t_size - 1.0) * 1.0 / (interpolation_num - 1.0);
		int curve_interpo;
		for (int i = 0; i < interpolation_num; i++)
		{
			curve_interpo = (int) d_length_t * i;

			int noColl_curve_after = noCollisionPoint({ mCC.Ecurve.x[curve_interpo], mCC.Ecurve.y[curve_interpo] }, obs);//possible problems
			if (noColl_curve_after == 0)//then examine Linebefore
			{
				mCC.curve_valid_ind = 0;
				if (mode == 0)
				{
					std::cout << "Curve Collision, curve unachievable" << std::endl;
				}
				return mCC;
			}

			int noColl_curve_before = noCollisionPoint({ mCC.Bcurve.x[curve_interpo], mCC.Bcurve.y[curve_interpo] }, obs);//possible problems
			if (noColl_curve_before == 0)//then examine Linebefore
			{
				mCC.curve_valid_ind = 0;
				if (mode == 0)
				{
					std::cout << "Curve Collision, curve unachievable" << std::endl;
				}
				return mCC;
			}
		}

		//detect collision of linear LineAfter
		noCollisionLine_outputs noColl_before = noCollisionLine(W1, { mCC.Bcurve.x[0], mCC.Bcurve.y[0] }, interpolation_num, obs);
		mCC.k_before = noColl_before.k;
		mCC.b_before = noColl_before.b;
		if (noColl_before.noCollision == 0)//then examine Linebefore
		{
			mCC.curve_valid_ind = 0;
			if (mode == 0)
			{
				std::cout << "Linear linebefore Collision, curve unachievable" << std::endl;
			}
			return mCC;
		}

		//get curve length
		curve_property curve_properties = curve_prop(eight_points.B0, eight_points.B1, eight_points.B2, eight_points.B3, eight_points.E0, eight_points.E1, eight_points.E2, eight_points.E3, t);
		mCC.Bcurve.x_prime = curve_properties.Bcurve_x_prime;
		mCC.Bcurve.y_prime = curve_properties.Bcurve_y_prime;
		mCC.Bcurve.length = curve_properties.Bcurve_length;
		mCC.Ecurve.x_prime = curve_properties.Ecurve_x_prime;
		mCC.Ecurve.y_prime = curve_properties.Ecurve_y_prime;
		mCC.Ecurve.length = curve_properties.Ecurve_length;


		mCC.Bcurve.sum_K = cal_curvature(eight_points.B0, eight_points.B1, eight_points.B2, eight_points.B3, t, mCC.Bcurve.x_prime, mCC.Bcurve.y_prime);
		mCC.Ecurve.sum_K = cal_curvature(eight_points.E0, eight_points.E1, eight_points.E2, eight_points.E3, t, mCC.Ecurve.x_prime, mCC.Ecurve.y_prime);
		mCC.sum_K = mCC.Bcurve.sum_K + mCC.Ecurve.sum_K;

		COORD curve_end_B = { mCC.Bcurve.x[0], mCC.Bcurve.y[0] };
		COORD curve_end_E = { mCC.Ecurve.x[0], mCC.Ecurve.y[0] };
		double curve_length = mCC.Ecurve.length + mCC.Ecurve.length;
		mCC.cur_line_length = dist(W1, curve_end_B) + dist(curve_end_E, W3) + curve_length;

	}

	return mCC;
}

//purpose:perform RRT operation
pure_RRT_outputs pure_RRT(COORD & q_start, COORD & q_goal, NodeClass & q_rand, double & StepSize_start, double & StepSize_mid,
	Near_Neighbor_outputs & NN_outputs, std::vector <std::array<double, 3>> & obs, double & kmax, double & gamma_max,
	std::array <double, t_size> & t, int mode, double & K_coeffi, int & interpolation_num, std::array<NodeClass, tree_size> & Tree)
{
	pure_RRT_outputs pure_RRT;
	pure_RRT.add_new_node = 0; // indicate whether a node is added on the Tree or not, 1 is added, 0 is not added
	//pure_RRT.latest = Tree.size();
	pure_RRT.latest = 0; //Tree.size()
	for (int i = 0; i < (int)Tree.size(); i++)
	{
		if (Tree[i].parent == -1)
		{
			pure_RRT.latest = i; //tree.size()
			break;
		}
	}

	for (int dum2 = 0; dum2 < NN_outputs.Num_neibors_of_qrand; dum2++)
	{
		double min_dist = NN_outputs.dis_ang_ind[dum2].dist;
		int index = NN_outputs.dis_ang_ind[dum2].index;
		NodeClass q_nearest = Tree[index];

		if (index == 0) //detect the case when Tree[0] in index_mat
		{
			std::cout << "Nearest_Neighbor should not connect q_rand with Tree(1),i.e. q_start" << std::endl;
		}

		NodeClass q_new = FindQnew(q_rand, q_nearest, min_dist, StepSize_mid);//steering, qnew between q_rand and q_nearest

		int noCollisionQNew = noCollisionPoint( q_new.coord, obs);//discard the qnew which are in obstacles
		if (noCollisionQNew == 0 )//collision
		{
			continue; // back to the start of the new loop
		}
		q_new.midcoord[0] = (q_new.coord[0] - q_nearest.coord[0]) / 2 + q_nearest.coord[0];
		q_new.midcoord[1] = (q_new.coord[1] - q_nearest.coord[1]) / 2 + q_nearest.coord[1];
		//call curve
		COORD W1 = q_nearest.midcoord; // former points on the tree
		COORD W2 = q_nearest.coord;
		COORD W3 = q_new.midcoord; //current points

		mainCommonCurve_outputs mCC = mainCommonCurve(W1, W2, W3, kmax, gamma_max, t, obs, mode, interpolation_num);
		
		if (mCC.curve_valid_ind == 0)
		{
			continue;
		}
		
		//Tree.push_back(q_new); // add q_new into Tree
		//pure_RRT.latest++;//if latest++, index doesn't match
		//Tree[latest].coord = q_new.coord;
		Tree[pure_RRT.latest].coord = q_new.coord;
		Tree[pure_RRT.latest].parent = index;
		Tree[pure_RRT.latest].s_cost = mCC.cur_line_length + K_coeffi * mCC.sum_K;
		Tree[pure_RRT.latest].t_cost = (Tree[pure_RRT.latest].s_cost + Tree[index].t_cost);

		Tree[pure_RRT.latest].midcoord = q_new.midcoord;
		Tree[pure_RRT.latest].curve = { mCC.Bcurve, mCC.Ecurve};
		Tree[pure_RRT.latest].linebefore = { mCC.k_before, mCC.b_before };
		Tree[pure_RRT.latest].lineafter = { mCC.k_after, mCC.b_after };
		Tree[index].son.push_back(pure_RRT.latest); //'index' is 'latest's parent, so add 'latest' as 'index's son
		Tree[pure_RRT.latest].dubins_goal_dist = dubins_curve_simple(Tree[pure_RRT.latest], q_goal, kmax / 1); //have not introduced dubins curve yet

		pure_RRT.add_new_node = 1;
		break;
		
	}
	return { pure_RRT.add_new_node, pure_RRT.latest };
}

//purpose: plot / draw out the valid curves
void ValidCurve(COORD& W1, COORD& W2, COORD& W3, double& kmax, double& gamma_max, std::array <double, t_size>& t, double & k_before, double & b_before, double & k_after, double & b_after, int interpolation_num)
{


	int gamma_compare = 0;
	std::array <double, 2> u1 = GetVector(W2, W1);//unit vector from W2 to W1
	std::array <double, 2> u2 = GetVector(W2, W3);//unit vector from W2 to W3

	//exit function when angle larger than allowable angle
	gamma_property gamma_prop = AngleErrorDetection(u1, u2, gamma_max);

	double c1 = 7.2364;
	double c2 = 0.4 * (sqrt(6) - 1);
	double c3 = (c2 + 4.0) / (c1+6.0);
	double c4 = pow(c2 + 4.0, 2) / (54.0 * c3);

	double gamma = gamma_prop.gamma_deg * pi / 180.0;//input: degree, output: radian,% allowable angle
	double beta = gamma / 2.0;//degree, angle between (-u1) and ud
	double d = (c4 * sin(beta)) / (kmax * pow(cos(beta), 2));//minimum step length required

	if (fabs(d) <= 1e-5)
	{
		double L1_start_x = W1[0];
		double L1_end_x = W2[0];
		double L2_start_x = W2[0];
		double L2_end_x = W3[0];
		std::string filename = "valid_edges.dat";
		plot_two_lines(L1_start_x, L1_end_x, L2_start_x, L2_end_x, interpolation_num, k_before, b_before, k_after, b_after, filename);

	}
	else
	{
		eight_points_outputs eight_points = cal_eight_control_points( W2, u1, u2, c2, c3, beta, d);

		coord_2d Bcurve_x_y = cubicbezier2d(eight_points.B0, eight_points.B1, eight_points.B2, eight_points.B3, t);
		coord_2d Ecurve_x_y = cubicbezier2d(eight_points.E0, eight_points.E1, eight_points.E2, eight_points.E3, t);
		double L1_start_x = W1[0];
		double L1_end_x = Bcurve_x_y.x[0];
		double L2_start_x = Ecurve_x_y.x[0];
		double L2_end_x = W3[0];
		//const char* filename = "a.dat";
		std::string filename = "valid_edges.dat";
		plot_two_lines( L1_start_x,  L1_end_x, L2_start_x, L2_end_x, interpolation_num, k_before, b_before, k_after, b_after, filename);

		std::ofstream plot_curve_file; //instantiate a std::ofstream object

		plot_curve_file.open("valid_curves.dat", std::ios::app); // make a .dat file called "circle.dat" containing all obstacles
		for (int i = 0; i < t_size; i++)
		{
			plot_curve_file << Bcurve_x_y.x[i] << " " << Bcurve_x_y.y[i] << std::endl;
			plot_curve_file << Ecurve_x_y.x[i] << " " << Ecurve_x_y.y[i] << std::endl;
		}
		plot_curve_file.close();
	}


}

//purpose:perform RRT star operation
void pure_RRT_star(vector<kdtree_struct> & kdtree, COORD& q_start, COORD & q_goal, NodeClass & q_rand, double& StepSize_start, double& StepSize_mid,
	Near_Neighbor_outputs& NN_outputs, std::vector <std::array<double, 3>> & obs, double& kmax, double& gamma_max,
	std::array <double, t_size>& t, int mode, double& K_coeffi, double& min_cost, int & interpolation_num, std::array<NodeClass, tree_size> & Tree) //missing map
{
	//int latest = Tree.size();
	int latest = 0;
	for (int i = 0; i < (int)Tree.size(); i++)
	{
		if (Tree[i].parent == -1)
		{
			latest = i; //tree.size()
			break;
		}
	}
	int near_count = 0;
	int lowest_cost_found_flag = 0;
	double lowest_cost = DBL_MAX;
	std::vector<int> index_vals;
	vector<int> index_mat;
	std::vector<int> p_index_vals;
	vector<int> examining_pp_close;
	static std::array<double, 2> range = { -StepSize_mid, StepSize_mid };

	for (int dum1 = 0; dum1 < NN_outputs.Num_neibors_of_qrand; dum1++)
	{
		double min_dist = NN_outputs.dis_ang_ind[dum1].dist;
		int index = NN_outputs.dis_ang_ind[dum1].index;
		//wait for later usage
		//    if isempty(Tree(index).Astar_goal_dist) == true %calculate A* dist when needed
		//Tree(index).Astar_goal_dist = AStarGrid_simple(map, Tree(index).coord, q_goal.coord, resolution);
		//end
			
		NodeClass q_nearest = Tree[index];
		if (index == 0)//detect the case when Tree[0] in index_mat
		{
			std::cout << "Nearest_Neighbor should not connect q_rand with Tree[0],i.e. q_start" << std::endl;
		}
		/////////////////////////////////conditionally choose the q_rand
		//if isempty(min_cost)~= true
		//	if q_nearest.t_cost + q_nearest.dubins_goal_dist > min_cost || ...
		//		q_nearest.t_cost + q_nearest.Astar_goal_dist > min_cost
		//		continue
		//		end
		//		end
		//if (q_nearest.t_cost + q_nearest.dubins_goal_dist > min_cost)
		//{
		//	continue;
		//}

		NodeClass q_new = FindQnew(q_rand, q_nearest, min_dist, StepSize_mid);//steering, qnew between qrand and qnearest
		int noCollisionQNew = noCollisionPoint(q_new.coord, obs);//discard the qnew which are in obstacles
		if (noCollisionQNew == 0) //collision
		{
			continue; // back to the start of the new loop
		}


		index_vals.clear();
		index_mat.clear();

		kd_rangequery_test(index_vals, kdtree, q_new.coord, range, 0);
		for (vector<int>::iterator it = index_vals.begin(); it != index_vals.end(); )//delete Tree[0] from index_vals
		{
			if (*it == 0)
			{
				it = index_vals.erase(it);
			}
			else
			{
				++it;
			}
		}
		random_shuffle(index_vals.begin(), index_vals.end()); //shuffle index_vals

		//if (index_vals.size() != 0)
		//{
		//	index_mat = index_vals;
		//}
		//else
		//{
		//	continue;
		//}

		int index_chosen_num = 10;
		if ((int)index_vals.size() > index_chosen_num)
		{
			for (int i = 0; i < index_chosen_num; i++)
			{
				index_mat.push_back(index_vals[i]);
			}
		}
		else
		{
			index_mat = index_vals;
		}


		for (int i = 0; i < (int)index_mat.size(); i++)
		{
			int near_index = index_mat[i];

			if (dist(Tree[near_index].coord, q_new.coord) > StepSize_mid)
			{
				continue;
			}

			int valid_indicator = Dist_Angle_Check_Single(q_new, Tree[near_index], Tree, gamma_max);
			if (valid_indicator == 0)
			{
				continue;
			}
			//call curve
			std::array<double, 2> W1 = Tree[near_index].midcoord;// former points on the tree
			std::array<double, 2> W2 = Tree[near_index].coord;
			std::array<double, 2> W3;// updated midcoord, since parent is different
			W3[0] = (q_new.coord[0] - Tree[near_index].coord[0]) / 2.0 + Tree[near_index].coord[0];
			W3[1] = (q_new.coord[1] - Tree[near_index].coord[1]) / 2.0 + Tree[near_index].coord[1];

			mainCommonCurve_outputs mCC = mainCommonCurve(W1, W2, W3, kmax, gamma_max, t, obs, mode, interpolation_num);
			if (mCC.curve_valid_ind == 0)
			{
				continue;
			}
			near_count++;

			if (mCC.cur_line_length + Tree[near_index].t_cost < lowest_cost)
			{
				lowest_cost_found_flag = 1;
				lowest_cost = mCC.cur_line_length + Tree[near_index].t_cost;
				q_new.parent = near_index;
				q_new.s_cost = mCC.cur_line_length + K_coeffi * mCC.sum_K;
				q_new.t_cost = (q_new.s_cost + Tree[near_index].t_cost);
				q_new.midcoord = W3;
				q_new.curve = { mCC.Bcurve, mCC.Ecurve };
				q_new.linebefore = { mCC.k_before, mCC.b_before };
				q_new.lineafter = { mCC.k_after, mCC.b_after };
				q_new.dubins_goal_dist = dubins_curve_simple(q_new, q_goal, kmax / 1);
				//q_new.Astar_goal_dist = AStarGrid_simple(map, q_new.coord, q_goal.coord, resolution);*/

				if (q_new.t_cost > min_cost)
				{
					continue;
				}
			}
		}

		if (lowest_cost_found_flag == 1)
		{
			latest++;
			//Tree.push_back(q_new);
			Tree[latest - 1] = q_new;
			Tree[Tree[(int)latest - 1].parent].son.push_back(latest - 1);//initialize property 'son', index is self-index

			//rewire
			int examining_pp = latest;
			p_index_vals.clear();
			examining_pp_close.clear();
			kd_rangequery_test(p_index_vals, kdtree, Tree[(int)examining_pp - 1].coord, range, 0);
			for (vector<int>::iterator it = p_index_vals.begin(); it != p_index_vals.end(); )//delete Tree[0] from index_vals
			{
				if (*it == 0)
				{
					it = p_index_vals.erase(it);
				}
				else
				{
					++it;
				}
			}
			random_shuffle(p_index_vals.begin(), p_index_vals.end()); //shuffle index_vals

			//if (p_index_vals.size() != 0)
			//{
			//	examining_pp_close = p_index_vals;
			//}
			//else
			//{
			//	continue;
			//}

			int index_chosen_num = 10;
			if ((int)p_index_vals.size() > index_chosen_num)
			{
				for (int i = 0; i < index_chosen_num; i++)
				{
					examining_pp_close.push_back(p_index_vals[i]);
				}
			}
			else
			{
				examining_pp_close = p_index_vals;
			}

			for (int i = 0; i < (int)examining_pp_close.size(); i++)
			{
				int examining_p = examining_pp_close[i];

				if (dist(Tree[examining_p].coord, Tree[(int)examining_pp - 1].coord) > StepSize_mid)
				{
					continue;
				}

				int Num_valid_neibor_for_rewire = Dist_Angle_Check_Single(Tree[examining_p], Tree[(int)examining_pp - 1], Tree, gamma_max);
				if (Num_valid_neibor_for_rewire == 0) // constraints are not complied
				{
					continue;
				}

				//call curve
				std::array<double, 2> W1_near_new = Tree[(int)examining_pp - 1].midcoord;// former points on the tree
				std::array<double, 2> W2_near_new = Tree[(int)examining_pp - 1].coord;
				std::array<double, 2> W3_near_new;// updated midcoord, since parent is different
				W3_near_new[0] = (Tree[examining_p].coord[0] - Tree[(int)examining_pp - 1].coord[0]) / 2.0 + Tree[(int)examining_pp - 1].coord[0];
				W3_near_new[1] = (Tree[examining_p].coord[1] - Tree[(int)examining_pp - 1].coord[1]) / 2.0 + Tree[(int)examining_pp - 1].coord[1];

				mainCommonCurve_outputs mCC_near_new = mainCommonCurve(W1_near_new, W2_near_new, W3_near_new, kmax, gamma_max, t, obs, mode, interpolation_num);
				if (mCC_near_new.curve_valid_ind == 0)
				{
					continue;
				}

				double current_cost_for_p = Tree[examining_p].t_cost; // old cost
				double poss_lower_cost_for_p = mCC_near_new.cur_line_length + Tree[(int)examining_pp - 1].t_cost;

				//qnear can use qnew as parent and the cost is lower
				if (poss_lower_cost_for_p < current_cost_for_p)
				{
					if (Tree[examining_p].son.size() == 0)
					{
						if (mode == 0)
						{
							std::cout << " this rewired node has no son, can rewire freely" << std::endl;
						}
						//delete "rewire_near_index" from its former parent's son list	;
						//Tree[Tree[examining_p].parent].son(Tree(Tree(examining_p).parent).son == examining_p) = [];//δ???
						auto iter = std::remove(std::begin(Tree[Tree[examining_p].parent].son), std::end(Tree[Tree[examining_p].parent].son), examining_p); //remove examining_p
						Tree[Tree[examining_p].parent].son.erase(iter, std::end(Tree[Tree[examining_p].parent].son)); // remove empty element cuased by std::remove
						//update parent
						Tree[examining_p].parent = examining_pp - 1;
						//update new parent's son list
						Tree[(int)examining_pp - 1].son.push_back(examining_p);
						Tree[examining_p].s_cost = mCC_near_new.cur_line_length + K_coeffi * mCC_near_new.sum_K;
						Tree[examining_p].t_cost = Tree[examining_p].s_cost + Tree[(int)examining_pp - 1].t_cost;
						Tree[examining_p].midcoord = W3_near_new;
						Tree[examining_p].curve = { mCC_near_new.Bcurve, mCC_near_new.Ecurve };
						Tree[examining_p].linebefore = { mCC_near_new.k_before, mCC_near_new.b_before };
						Tree[examining_p].lineafter = { mCC_near_new.k_after, mCC_near_new.b_after };
						Tree[examining_p].dubins_goal_dist = dubins_curve_simple(Tree[examining_p], q_goal, kmax / 1);
						// Tree(examining_p).Astar_goal_dist = AStarGrid_simple(map, Tree(examining_p).coord, q_goal.coord, resolution);
					}
					else if (Tree[examining_p].son.size() != 0)
					{
						int deleted_son = 0;
						int dum_index = -1; //is "0" in matlab
						int son_num = Tree[examining_p].son.size();

						for (int dum_3 = 0; dum_3 < son_num; dum_3++)
						{
							dum_index++;
							dum_index = dum_index - deleted_son;

							if ((int)Tree[examining_p].son.size() < dum_index)
							{
								std::cout << "problem occurs, error" << std::endl;
							}

							int son_index = Tree[examining_p].son[dum_index];
							//try wire up qnear, qnear.son, qnear.parent
							std::array<double, 2> W1_near_son;
							W1_near_son[0] = (Tree[examining_p].coord[0] - Tree[(int)examining_pp - 1].coord[0]) / 2 + Tree[(int)examining_pp - 1].coord[0]; //% former points on the tree
							W1_near_son[1] = (Tree[examining_p].coord[1] - Tree[(int)examining_pp - 1].coord[1]) / 2 + Tree[(int)examining_pp - 1].coord[1]; //% former points on the tree
							std::array<double, 2> W2_near_son = Tree[examining_p].coord;
							std::array<double, 2> W3_near_son;
							W3_near_son[0] = (Tree[son_index].coord[0] - Tree[examining_p].coord[0]) / 2 + Tree[examining_p].coord[0]; //% updated midcoord, since parent is different
							W3_near_son[1] = (Tree[son_index].coord[1] - Tree[examining_p].coord[1]) / 2 + Tree[examining_p].coord[1]; //% updated midcoord, since parent is different
							mainCommonCurve_outputs mCC_near_son = mainCommonCurve(W1_near_son, W2_near_son, W3_near_son, kmax, gamma_max, t, obs, mode, interpolation_num);

							if (mCC_near_son.curve_valid_ind == 0)
							{
								deleted_son = 0;
								if (mode == 0)
								{
									std::cout << "rewire is invalid for rewire_near_index.son, use the former qnear.parent" << std::endl;
								}
							}
							else
							{
								if (mode == 0)
								{
									std::cout << "the new parent of q_near is valid for q_near.son, qnear can change parent" << std::endl;
								}
								deleted_son = 1;
								latest++;
								//NodeClass q_latest; //initialize a new node in Tree and push back into Tree
								//Tree.push_back(q_latest);
								auto iter = std::remove(std::begin(Tree[examining_p].son), std::end(Tree[examining_p].son), son_index); //remove examining_p
								Tree[examining_p].son.erase(iter, std::end(Tree[examining_p].son)); // remove empty element cuased by std::remove
								//% Tree(rewire_near_index).son = [Tree(rewire_near_index).son]; % son list remains unchanged
								Tree[(int)latest-1].coord = Tree[examining_p].coord;
								//% update "rewire_near_index" parent
								Tree[(int)latest-1].parent = examining_pp-1;
								//% update "new_parent.parent".son list
								Tree[Tree[(int)latest-1].parent].son.push_back(latest-1);
								Tree[(int)latest-1].s_cost = mCC_near_new.cur_line_length;
								Tree[(int)latest - 1].t_cost = poss_lower_cost_for_p;
								Tree[(int)latest - 1].midcoord = W3_near_new;
								Tree[(int)latest - 1].curve = { mCC_near_new.Bcurve, mCC_near_new.Ecurve };
								Tree[(int)latest - 1].linebefore = {mCC_near_new.k_before, mCC_near_new.b_before};
								Tree[(int)latest - 1].lineafter = { mCC_near_new.k_after, mCC_near_new.b_after };
								//% update new parent's son list
								Tree[(int)latest - 1].son.push_back(son_index);
								Tree[(int)latest - 1].dubins_goal_dist = dubins_curve_simple(Tree[(int)latest - 1], q_goal, kmax / 1);
								//Tree(latest).Astar_goal_dist = AStarGrid_simple(map, Tree(latest).coord, q_goal.coord, resolution);

								Tree[son_index].s_cost = mCC_near_son.cur_line_length + K_coeffi * mCC_near_son.sum_K;
								Tree[son_index].t_cost = Tree[son_index].s_cost + Tree[(int)latest - 1].t_cost;
								Tree[son_index].parent = latest-1; //% changed from rewire_near_index to valid_latest
								Tree[son_index].son = Tree[son_index].son;// % unchanged
								Tree[son_index].midcoord = W3_near_son; //% unchanged
								Tree[son_index].curve = { mCC_near_son.Bcurve, mCC_near_son.Ecurve }; //% unchanged
								Tree[son_index].linebefore = { mCC_near_son.k_before, mCC_near_son.b_before }; //% unchanged
								Tree[son_index].lineafter = { mCC_near_son.k_after, mCC_near_son.b_after }; //% unchanged
								Tree[son_index].dubins_goal_dist = dubins_curve_simple(Tree[son_index], q_goal, kmax / 1);
								//% Tree[son_index].Astar_goal_dist = AStarGrid_simple(map, Tree[son_index].coord, q_goal.coord, resolution);
								
								update_offspring_cost( Tree, son_index, son_index);

							}

						}

					}

				}
			}//end of rewire part
		}
		else
		{
			if (mode == 0)
			{
				std::cout <<  "did not find lowest cost" << std::endl;
			}
		}
		break; //a valid qnew added to Tree, stop trying new qnear as parent
	}
	

}

//purpose:update offsprings' cost once their root node's cost is changed
void update_offspring_cost(std::array<NodeClass, tree_size> & Tree, int cur_p, int first_p)
{
	if (Tree[cur_p].son.size() != 0)
	{
		for (int i = 0; i < (int)Tree[cur_p].son.size(); i++)
		{
			Tree[Tree[cur_p].son[i]].t_cost = Tree[Tree[cur_p].son[i]].s_cost + Tree[cur_p].t_cost;
			update_offspring_cost(Tree, Tree[cur_p].son[i], first_p);
		}
	}
	else if (Tree[cur_p].son.size() == 0)
	{
		return;
	}

	if (cur_p == first_p)
	{
		return;
	}
}

//purpose:get q_rand
NodeClass sampling_points(double goal_bias_prob, double& xmax, double& ymax, double& close_range, NodeClass& q_start, NodeClass& q_goal, double& stepsize, double& furtherest_tree) 
{
	NodeClass q_rand;
	///////for generating rand purpose
	std::random_device rd;  // ??????????????????????
	std::mt19937 gen(rd()); // ?? rd() ??????? mersenne_twister_engine
	std::uniform_real_distribution<> dis(0, 1);
	//////remove("randompoints.dat");
	////std::ofstream plot_random_file; //instantiate a std::ofstream object

	if (dis(gen) >= goal_bias_prob)
	{
		q_rand.coord = { dis(gen) * xmax,  dis(gen) * ymax };
		double q_rand_start_dist = dist(q_start.coord, q_rand.coord);

		if (q_rand_start_dist > furtherest_tree + stepsize)
		{
			q_rand.coord[0] = (q_rand.coord[0] / q_rand_start_dist) * furtherest_tree + stepsize;
			q_rand.coord[1] = (q_rand.coord[1] / q_rand_start_dist) * furtherest_tree + stepsize;
		}
	}
	else
	{
		q_rand.coord[0] = q_goal.coord[0] + close_range * (dis(gen) - 1.0);
		q_rand.coord[1] = q_goal.coord[1] + close_range * (dis(gen) - 1.0);

		//making sure qrand is sampled in the map but not out of map
		if (q_rand.coord[0] > xmax)
		{
			q_rand.coord[0] = xmax;
		}
		else if (q_rand.coord[0] < 0)
		{
			q_rand.coord[0] = 0;
		}

		if (q_rand.coord[1] > ymax)
		{
			q_rand.coord[1] = ymax;
		}
		else if (q_rand.coord[1] < 0)
		{
			q_rand.coord[1] = 0;
		}
	}
	return q_rand;
}

//purpose:define the search range of qrand
double search_range(double coefficient, int & tree_length, double & xmax, double & ymax, double & stepsize)
{
	//adjust the search rane of kd_rangequery"
	double kd_range = coefficient * pow((log(tree_length) / tree_length), 0.5);

	if ((kd_range >= 0.2 * xmax) || (kd_range >= 0.2 * ymax))
	{
		if (xmax > ymax)
		{
			kd_range = 0.2 * ymax;
		}
		else
		{
			kd_range = 0.2 * xmax;
		}
	}

	if (kd_range < stepsize)
	{
		kd_range = stepsize;
	}
	
	return kd_range;
}