#pragma once
#include "NodeClass.h"
#include <vector>
#include <array>
#include "constants.h"
#include "kdtree.h"
//class NodeClass;
//typedef NodeClass::COORD COORD;
//typedef std::array<double, 2> COORD;
using namespace std;

//purpose: find q_new which locates between q_rand and q_near
NodeClass FindQnew(NodeClass & q_rand, NodeClass & q_near, double & min_dist, double & stepsize);

struct dis_ang_ind_str
{
	double dist;
	double angle_diff;
	int index;
};

struct Near_Neighbor_outputs
{
	std::vector<dis_ang_ind_str> dis_ang_ind;
	int Num_neibors_of_qrand = 0;
};

struct mainCommonCurve_outputs
{
	int curve_valid_ind = 0;
	BEcurve Bcurve;
	BEcurve Ecurve;
	double k_before = 0.0;
	double b_before = 0.0;
	double k_after = 0.0;
	double b_after = 0.0;
	double cur_line_length = 0.0;
	double sum_K = 0.0;

};

//purpose:output the nearest neighbor that comply with the constraints
Near_Neighbor_outputs Nearest_Neighbor(NodeClass & q_rand, std::array<NodeClass, tree_size> & Tree, std::vector<int> & index_mat, double & gamma_max, int kdtree_built_flag, int rewire_flag);

//purpose:get the curve starting from W1, turns at W2, and ends at W3.
mainCommonCurve_outputs mainCommonCurve(COORD& W1, COORD& W2, COORD& W3, double& kmax, double& gamma_max, std::array <double, t_size>& t, std::vector <std::array<double, 3>> & obs, int mode, int& interpolation_num);

const double c1 = 7.2364;
const double c2 = 0.4 * (sqrt(6) - 1);
const double c3 = (c2 + 4) / (c1 + 6);
const double c4 = pow((c2 + 4), 2) / (54 * c3);

struct pure_RRT_outputs
{
	int add_new_node;
	int latest;
};

//purpose:perform RRT operation
pure_RRT_outputs pure_RRT(COORD& q_start, COORD& q_goal, NodeClass & q_rand, double& StepSize_start, double& StepSize_mid,
	Near_Neighbor_outputs& NN_outputs, std::vector <std::array<double, 3>> & obs, double& kmax, double& gamma_max,
	std::array <double, t_size>& t, int mode, double& K_coeffi, int& interpolation_num, std::array<NodeClass, tree_size>& Tree);

//purpose: plot / draw out the valid curves
void ValidCurve(COORD& W1, COORD& W2, COORD& W3, double& kmax, double& gamma_max, std::array <double, t_size>& t, double& k_before, double& b_before, double& k_after, double& b_after, int interpolation_num);

//purpose:perform RRT star operation
void pure_RRT_star(vector<kdtree_struct>& kdtree, COORD& q_start, COORD& q_goal, NodeClass& q_rand, double& StepSize_start, double& StepSize_mid,
	Near_Neighbor_outputs& NN_outputs, std::vector <std::array<double, 3>> & obs, double& kmax, double& gamma_max,
	std::array <double, t_size>& t, int mode, double& K_coeffi, double& min_cost, int& interpolation_num, std::array<NodeClass, tree_size> & Tree);

//purpose:update offsprings' cost once their root node's cost is changed
void update_offspring_cost(std::array<NodeClass, tree_size> & Tree, int cur_p, int first_p);

//purpose:get q_rand
NodeClass sampling_points(double goal_bias_prob, double & xmax, double & ymax, double & close_range, NodeClass & q_start, NodeClass & q_goal, double & stepsize, double & furtherest_tree);

//purpose:define the search range of qrand
double search_range(double coefficient, int& tree_length, double& xmax, double& ymax, double& stepsize);