#pragma once
#include <string>
#include <vector>
#include "NodeClass.h"
using namespace std;

struct kdtree_struct
{
	string type = "leaf";
	int left = 0;
	int right = 0;
	array<double, 2> nodevector;
	int numpoint;
	int index;
	int parent;
	double splitval = 0;
	int splitdim = 0;
	int tree_index;

};

//return: a kd tree containing two nodes. Other nodes will be added into kdtree through kd_insert function
vector< kdtree_struct > kd_buildtree(std::vector <std::array<double, 2>> & X, int parent_number, int split_dimension);

//Purpose: find the point in the kdtree that are in the range of the investigated point
void kd_rangequery_test(vector<int> & index_vals, vector<kdtree_struct>& kdtree, array<double, 2> & point, array<double, 2> & range, int node_number);

//Purpose: insert new points into kdtree
void kd_insert(vector<kdtree_struct>& kdtree, array<double, 2> & point, int current_index);

//Purpose://purpose: build kdtree
vector<kdtree_struct> initialized_kdtree(std::array<NodeClass, tree_size> & Tree);