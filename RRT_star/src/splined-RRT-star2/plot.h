#pragma once
#include "NodeClass.h"
#include <vector>
#include <array>
#include <tuple>
#include "constants.h"

//purpose: plot / store out obstacles, qstart, qgoal in .dat file
void plot_obs_start_goal(NodeClass q_start, NodeClass q_goal, double x_max, double y_max, std::vector <std::array<double, 3>> obs, double close_range);

//purpose: plot / store given node: "coordinate"
void scatter(COORD coordinate);

//purpose: plot / store the circles (i.e. obstacles, close range circle)
std::tuple<std::array<double, t_size>, std::array<double, t_size>> circle(double center_x, double center_y, double radius);

//purpose: plot all edges in the tree
void plot_all_edges(std::array<NodeClass, tree_size> & Tree, int& interpolation_num);

//purpose: plot two straight lines connecting every cubic bezier curves
void plot_two_lines(double& L1_start_x, double& L1_end_x, double& L2_start_x, double& L2_end_x, int& interpolation_num, double& k_before, double& b_before, double& k_after, double& b_after, std::string filename);