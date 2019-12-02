#include <array>
#include <vector>
#include "kdtree.h"
#include "NodeClass.h"

using namespace std;

//return: a kd tree containing two nodes. Other nodes will be added into kdtree through kd_insert function
vector< kdtree_struct> kd_buildtree(vector<array<double, 2>> & X, int parent_number, int split_dimension)
{
	//array<kdtree_struct, 2> kdtree;
	vector<kdtree_struct> kdtree;
	kdtree_struct tree1;
	kdtree_struct tree2;
	kdtree.push_back(tree1);
	kdtree.push_back(tree2);
	int root;
	int not_root;
	if (X[0][split_dimension] < X[1][split_dimension])
	{
		root = 0;
		not_root = 1;
	}
	else
	{
		root = 1;
		not_root = 0;
	}
	//kdtree[1].left = 1;
	kdtree[root].type = "node";
	kdtree[not_root].type = "leaf";

	kdtree[root].right = not_root;

	kdtree[root].nodevector = X[root];
	kdtree[not_root].nodevector = X[not_root];

	kdtree[root].numpoint = 2;
	kdtree[not_root].numpoint = 1;

	kdtree[root].index = 0;
	kdtree[not_root].index = 1;

	kdtree[root].parent = -1;
	kdtree[not_root].parent = 0;

	kdtree[0].splitval = X[0][split_dimension];

	kdtree[root].splitdim = split_dimension;

	kdtree[root].tree_index = 0;
	kdtree[not_root].tree_index = 1;

	return kdtree;
}

//Purpose: insert new points into kdtree
void kd_insert(vector<kdtree_struct>& kdtree, array<double, 2> & point, int current_index = 0)
{
	//define splitdim and splitval for 'leaf' node 
	int splitdim;
	if (kdtree[current_index].splitdim == 0)
	{
		kdtree[current_index].splitdim = current_index % 2; //define splitdim
		splitdim = kdtree[current_index].splitdim;
		kdtree[current_index].splitval = kdtree[current_index].nodevector[splitdim];
	}
	else
	{
		splitdim = kdtree[current_index].splitdim;
	}

	//as the point will be a offspring of the current kdtree node, numpoints+1
	kdtree[current_index].numpoint = kdtree[current_index].numpoint + 1;
	int col = kdtree.size();

	//point shall rest in left sub tree
	if (point[splitdim] <= kdtree[current_index].nodevector[splitdim])
	{
		//point's final location
		if (kdtree[current_index].left == 0)
		{
			kdtree_struct new_node;
			kdtree.push_back(new_node);
			kdtree[col].nodevector = point;
			kdtree[col].numpoint = 1;
			kdtree[col].index = col;
			kdtree[col].type = "leaf";
			kdtree[col].parent = kdtree[current_index].tree_index; // update parent
			kdtree[col].tree_index = col;

			kdtree[current_index].left = kdtree[col].tree_index; // update tree(current_index)'s left
			kdtree[current_index].type = "node";

			return;//end function
		}
		//next recursion, search in the current node's left sub-tree
		else
		{
			current_index = kdtree[current_index].left;
			kd_insert(kdtree, point, current_index);
		}
	}
	else
	{
		//point's final location
		if (kdtree[current_index].right == 0)
		{
			kdtree_struct new_node;
			kdtree.push_back(new_node);
			kdtree[col].nodevector = point;
			kdtree[col].numpoint = 1;
			kdtree[col].index = col;
			kdtree[col].type = "leaf";
			kdtree[col].parent = kdtree[current_index].tree_index; // update parent
			kdtree[col].tree_index = col;

			kdtree[current_index].right = kdtree[col].tree_index; // update tree(current_index)'s left
			kdtree[current_index].type = "node";

			return;//end function
		}
		//next recursion, search in the current node's left sub-tree
		else
		{
			current_index = kdtree[current_index].right;
			kd_insert(kdtree, point, current_index);
		}
	}
	return;
}

//Purpose: find the point in the kdtree that are in the range of the investigated point
void kd_rangequery_test(vector<int> & index_vals, vector<kdtree_struct>& kdtree, array<double, 2> & point, array<double, 2> & range, int node_number)//initial "node number" is 0
{
	if ((kdtree[node_number].nodevector[0] >= point[0] + range[0]) && (kdtree[node_number].nodevector[1] >= point[1] + range[0]) &&
		(kdtree[node_number].nodevector[0] <= point[0] + range[1]) && (kdtree[node_number].nodevector[1] <= point[1] + range[1]))
	{
		index_vals.push_back(kdtree[node_number].index);
	}

	if (kdtree[node_number].type == "leaf")
	{
		return;
	}

	//% if the current node is not a leaf
	//	% check to see if the range hypercuboid is to the left of the split
	//	%and in that case send the left node out for inquiry
	if (((point[kdtree[node_number].splitdim] + range[0]) <= kdtree[node_number].splitval) &&
		((point[kdtree[node_number].splitdim] + range[1]) <= kdtree[node_number].splitval))
	{
		if (kdtree[node_number].left != 0)
		{
			kd_rangequery_test(index_vals, kdtree, point, range, kdtree[node_number].left);
		}
	}

	//% if the current node is not a leaf
	//	% check to see if the range hypercuboid is to the right of the split
	//	%and in that case send the right node out for inquiry
	if (((point[kdtree[node_number].splitdim] + range[0]) > kdtree[node_number].splitval) &&
		((point[kdtree[node_number].splitdim] + range[1]) > kdtree[node_number].splitval))
	{
		if (kdtree[node_number].right != 0)
		{
			kd_rangequery_test(index_vals, kdtree, point, range, kdtree[node_number].right);
		}
	}


	//% if the current node is not a leaf
	//	% check to see if the range hypercuboid stretches from the left to the right of the split
	//	% in that case send the leftand the right node out for inquiry
	if (((point[kdtree[node_number].splitdim] + range[0]) <= kdtree[node_number].splitval) &&
		((point[kdtree[node_number].splitdim] + range[1]) > kdtree[node_number].splitval))
	{
		if (kdtree[node_number].left != 0)
		{
			kd_rangequery_test(index_vals, kdtree, point, range, kdtree[node_number].left);
		}

		if (kdtree[node_number].right != 0)
		{
			kd_rangequery_test(index_vals, kdtree, point, range, kdtree[node_number].right);
		}
	}
}

//purpose: build kdtree
vector<kdtree_struct> initialized_kdtree(std::array<NodeClass, tree_size>& Tree)
{
	vector<array<double, 2>> X;
	X.push_back({ Tree[0].coord });
	X.push_back({ Tree[1].coord });
	vector< kdtree_struct> kdtree = kd_buildtree(X, 0, 0);
	for (int i = 0; i < (int)kdtree.size(); i++)
	{
		kdtree[i].tree_index = i;
	}

	return kdtree; // kd_tree_built_flag = 1
}
