#include "splined-RRT-star.h"
#include <algorithm>
#include "curve.h"
#include <chrono>
#include  "Collision_Constraints.h"
#include "constants.h"
#include "plot.h"
#include <array>
#include "Line_Angle.h"

#include "splined-RRT-star.h"
#include <fstream>
#include <random>
#include "kdtree.h"
#include "dubins.h"
#include "NodeClass.h"
#include <float.h>
int main()
{
	array<double, 2> range; //iniitalize the search range of kd range query
	vector<int> index_vals; //initialize the vector for storing the index of the nodes which are withing "range"
	vector<int> index_mat; //the randomly chosen "index_vals"
	vector<kdtree_struct> kdtree;

	auto start_time = std::chrono::steady_clock::now(); //start counting time
	int mode = 1; //0: debug mode, 1: release mode
	double StepSize_start = 3; //% step size for the first edge / last edge
	double StepSize_mid = 2 * StepSize_start;//% step size for other intermediate edges
	double close_range = 3;//% within this range, try qrand = qgoal
	int MaxNumNode = tree_size; //% maximum number of random sampling node

	double TurnRadius = 3.13; //% maximum turning radius
	double kmax = 1 / TurnRadius;//% maximum curvature
	double c4 = 1.1226; //constants for calculating gamma_max
	// gamma_max has a relationship with d.
	double gamma_max = cal_gamma_max_from_d(c4, kmax, StepSize_start); //% maximum allowable angle, range: [0, 180]

	std::array <double, t_size> t; //get coefficient "t"
	double dt = 1 / ((t_size - 1) * 1.0);
	for (int i = 0; i < t_size; i++)
	{
		t[i] = i * dt;
	}

	//construct map and obstacles
	double x_max = 100; // map size
	double y_max = 100; // map size
 	std::vector <std::array<double, 3>> obs =  {
			//o(:,1):x_center; o(:,2):y_center; o(:,3):radius
		//{ 15, 10, 5 * sqrt(2)},
		//{30, 45, 8 * sqrt(2)},
		//{28.5, 18.5, 3.5 * sqrt(2)},
		//{42.5, 77.5, 12.5 * sqrt(2)},
		//{45, 25, 5 * sqrt(2)},
		//{59, 49, 9 * sqrt(2)},
		//{67.5, 12.5, 7.5 * sqrt(2)},
		//{92.5, 42.5, 7 * sqrt(2)},
		//{85, 85, 5 * sqrt(2)},
		//% use for right angle
		{7.5, 25, 7.0711 },
		{15,25,7.0711},
		{25,25,7.0711},
		{35,25,7.0711},
		{45, 25, 7.0711},
		{55,25,7.0711},
		{65,25,7.0711},
		{75,25,7.0711},
		{75,35,7.0711},
		{75,45,7.0711},
		{75,55,7.0711},
		{75,65,7.0711},
		{75,75,7.0711},
		{75,85,7.0711},
	} ;
	//double resolution = 0.5; // width and height of each cell in the discretized map
	//map AStar_discrete_map_with_obs (TO BE DONE) //Astar feature is not added yet

	//initialize NodeClass, and q_start, q_goal
	NodeClass q_start({ 0, 0 });
	NodeClass q_goal({ 100, 70 });

	q_goal.t_cost = 0;
	q_goal.s_cost = 0;
	q_goal.midcoord = q_goal.coord;
	q_goal.lineafter = { 1,0 };

	q_start.t_cost = 0;
	q_start.s_cost = 0;
	q_start.parent = 0;
	q_start.son.push_back(1);
	q_start.midcoord = q_start.coord;
	q_start.curve[0].x[0] = q_start.coord[0];
	q_start.curve[0].y[0] = q_start.coord[1];
	q_start.curve[1].x[0] = q_start.coord[0];
	q_start.curve[1].y[0] = q_start.coord[1];
	q_start.linebefore = { 0, 0 }; // first number: gradient k, second number: interceptor b.
	q_start.lineafter = { 1, 0 }; //% get angle of q_start from gradient
	q_start.dubins_goal_dist = dubins_curve_simple(q_start, q_goal, TurnRadius); //dubins curve is not added yet

	static std::array<NodeClass,tree_size> Tree;//the Tree recording all nodes
	Tree[0] = q_start;
	
	double q_start_angle_rad = atan(q_start.lineafter[0]);
	Tree[1].coord = { cos(q_start_angle_rad) * StepSize_start + Tree[0].coord[0], sin(q_start_angle_rad) * StepSize_start + Tree[0].coord[1] }; //Tree[1] = q_start_after
	Tree[1].t_cost = StepSize_start;
	Tree[1].s_cost = StepSize_start;
	Tree[1].parent = 0;
	//Tree[1].son = []; uninitialized son
	Tree[1].midcoord = q_start.coord;
	Tree[1].curve = q_start.curve;
	Tree[1].linebefore = q_start.linebefore;
	Tree[1].lineafter = q_start.lineafter; //% get angle
	Tree[1].dubins_goal_dist = dubins_curve_simple(Tree[1], q_goal, TurnRadius);
	//Tree[1].Astar_goal_dist = AStarGrid_simple(map, Tree[1].coord, q_goal.coord, resolution);

	plot_obs_start_goal(Tree[0], q_goal, x_max, y_max, obs, close_range);

	std::vector<int> oldparent;
	double K_coeffi = 0.01; //coefficient of curvature in cost
	double path_len_coeffi = 1.25;//coefficient to determine the terminate cost
	int interpolation_num = 5;//the interpolation number in collision checking
	std::vector<int> close_goal_index;//the index of nodes that are within 'close_range'
	std::vector<double> close_cost;//the cost of nodes which are in 'close_range'
	double min_cost = DBL_MAX;//the node having minimum cost within 'close_range'
	int min_cost_index = 0;//the index of the node having minimum cost within 'close_range' 
	int star_flag = 0; //indicator of RRT*. If =0, RRT, if !=0, RRT*.
	double old_min_cost = DBL_MAX;
	int kd_tree_built_flag = 0; //indicator of whether kdtree has been built. 0, not, 1, yes.
	double furtherest_Tree_node = 0; //the distance between qstart and the nodes on the tree which is the furtherest with qstart.
	double added_node_dist; //distance between the newly added node and qstart

	double MinTime = 0; // unit:second
	double MaxTime = 20000;

	std::vector<int> Path; // initialize final path

	//for generating rand purpose
	std::random_device rd;  // �����ڻ����������������
	std::mt19937 gen(rd()); // �� rd() ���ֵı�׼ mersenne_twister_engine
	std::uniform_real_distribution<> dis(0, 1);

	for (int NumSample = 0; NumSample < MaxNumNode; NumSample++)
	{
  		//std::cout << "Number of sampled points is: " << NumSample << std::endl;

		//exit when MaxTime is reached
		auto intermediate_time = std::chrono::steady_clock::now();
		double elapsedTime = std::chrono::duration<double>(intermediate_time - start_time).count();
		if (elapsedTime > MaxTime)
		{	std::cout << "no route found in the allowed time" << std::endl;
			break;
		}
		
		//get Tree size
		int Tree_size = 0;
		for (int i = 0; i < (int)Tree.size(); i++)
		{
			if (Tree[i].parent == -1)
			{
				Tree_size = i; //tree.size()
				break;
			}
		}

		//get q_rand
		NodeClass q_rand = sampling_points(0, x_max, y_max, close_range, q_start, q_goal, StepSize_mid, furtherest_Tree_node);


		index_vals.clear();
		index_mat.clear();

		int initial_tree_length = 0;
		if (NumSample == initial_tree_length)
		{
			kdtree = initialized_kdtree(Tree);//build kdtree
			kd_tree_built_flag = 1;//kdtree built indicator
		}

		//get index_mat
		if (NumSample <= initial_tree_length)
		{
			index_mat.push_back(1);//the initial close index before the tree is built
		}
		else
		{
			double kd_range = search_range(x_max, Tree_size, x_max, y_max, StepSize_mid); // set linear range of kd_rangequery
			range = { -kd_range, kd_range };//2d range of kd_range
			kd_rangequery_test(index_vals, kdtree, q_rand.coord, range, 0);//find points within kd range
			for (vector<int>::iterator it = index_vals.begin(); it != index_vals.end(); )
			{
				if (*it == 0)
				{
					it = index_vals.erase(it); //delete Tree[0](i.e. qstart) from index_vals
				}
				else
				{
					++it;
				}
			}
			random_shuffle(index_vals.begin(), index_vals.end()); //shuffle index_vals
			int index_chosen_num = 10;//interpolation number
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
		}

		//% save the node if the angle_diff between itselfand q_rand is within gamma_max
		//	% record: distance, angle_difference, node index on the Tree
		Near_Neighbor_outputs NN_outputs = Nearest_Neighbor(q_rand, Tree, index_mat, gamma_max, kd_tree_built_flag, star_flag);
		if (NN_outputs.Num_neibors_of_qrand == 0)
		{
			continue;//no nodes comply with constraints
		}

		//switch between RRT and RRT*
		pure_RRT_outputs pure_RRT_outputs;
		if (star_flag == 0 || kd_tree_built_flag == 0)
		{
			//std::cout << "pure RRT mode" << std::endl;
			pure_RRT_outputs = pure_RRT(q_start.coord, q_goal.coord, q_rand, StepSize_start, StepSize_mid,
				 NN_outputs, obs, kmax, gamma_max, t, mode, K_coeffi, interpolation_num,Tree);
			if (pure_RRT_outputs.add_new_node == 0)
			{
				continue;
			}
		}
		else
		{
			if (star_flag == 1)
			{
				std::cout << "get into rewire mode" << std::endl;
				star_flag++;
			}
			pure_RRT_star(kdtree, q_start.coord, q_goal.coord, q_rand, StepSize_start, StepSize_mid,
				 NN_outputs, obs, kmax, gamma_max, t, mode, K_coeffi, min_cost, interpolation_num, Tree);
		}

		//get new Tree size when new nodes inserted into the tree
		int latest = 0;
		for (int i = 0; i < (int)Tree.size(); i++)
		{
			if (Tree[i].parent == -1)
			{
				latest = i; //tree.size()
				break;
			}
		}


		for (int added_node = Tree_size; added_node < latest; added_node++) // record every node that close to the q_goal
		{
			//insert new nodes which are within close_range into close_goal_index
			if (dist(Tree[added_node].coord, q_goal.coord) <= close_range)
			{
				oldparent.push_back(Tree[added_node].parent);
				//%%%%%%%%%%%%%%%%%%%%%%%%%% dubins curve for points close to q_goal
				//	[dubins_path, Tree(added_node).final_dubins_length] = dubins_curve(Tree(added_node), q_goal, 3.13, -1, 1);
				//% plot(dubins_path(:, 1), dubins_path(:, 2));
				//%%%%%%%%%%%%%%%%%%%%%%%%%%%%
				star_flag++;
				close_goal_index.push_back(added_node);
			}

			//insert every new node into kdtree
			if (kd_tree_built_flag == 1)
			{
				kd_insert(kdtree, Tree[added_node].coord, 0);
			}

			//found the current furtherest node on the Tree, limit q)rand's sampling range 
			added_node_dist = dist(q_start.coord, Tree[added_node].coord);
			if (added_node_dist > furtherest_Tree_node)
			{
				furtherest_Tree_node = added_node_dist;
			}
		
		}
		
		//update parent of the nodes in close_goal_index if their parents is changed
		for (int i = 0; i < (int)close_goal_index.size(); i++)
		{
			if (Tree[close_goal_index[i]].parent != oldparent[i])
			{
				//[dubins_path, Tree(close_goal_index(i)).final_dubins_length] = dubins_curve(Tree(close_goal_index(i)), q_goal, TurnRadius, -1, 1);
				oldparent[i] = Tree[close_goal_index[i]].parent;
			}
		}

		//update min_cost at every iter after a initial path is generated
		if (star_flag != 0)
		{
			close_cost.clear();
			for (int i = 0; i < (int)close_goal_index.size(); i++)
			{
				close_cost.push_back(Tree[close_goal_index[i]].t_cost); // missing dubins curve length!
			}

			
			for (int i = 0; i < (int)close_cost.size(); i++)
			{
				if (close_cost[i] < min_cost)
				{
					min_cost = close_cost[i];
					min_cost_index = i;
				}
			}

			if (min_cost < old_min_cost)
			{
				std::cout << "path cost updated" << std::endl;
				std::cout << NumSample << std::endl;
				old_min_cost = min_cost;
			}
		}

		//exit condition
		if ((min_cost < dist(q_start.coord, q_goal.coord) * path_len_coeffi) || (NumSample == MaxNumNode))
		{
			std::cout << "path generated successfully!" << std::endl;
			auto intermediate_time = std::chrono::steady_clock::now();
			double elapsedTime = std::chrono::duration<double>(intermediate_time - start_time).count();
			if (elapsedTime < MinTime)
			{
				continue;
			}
			else
			{
				Path.push_back(close_goal_index[min_cost_index]);
				int pathsize = (int)Path.size();
				while (Path[pathsize -1] != 0)
				{
					Path.push_back(Tree[Path[pathsize - 1]].parent);
					pathsize = (int)Path.size();
				}
				pathsize = (int)Path.size();
				Path.push_back(Tree[Path[pathsize - 1]].parent); //connect with q_start
				break;
			}
		}
 	}

	//count time
	auto end_time = std::chrono::steady_clock::now();
	double elapsedTime = std::chrono::duration<double>(end_time - start_time).count();
	std::cout << "main function take:" << elapsedTime << "s" << std::endl;

	//get final tree length
	int tree_size = 0;
	for (int i = 0; i < (int)Tree.size(); i++)
	{
		if (Tree[i].parent == -1)
		{
			tree_size = i; //tree.size()
			break;
		}
	}

	//plot stuffs
	std::cout << "Tree size" << tree_size << std::endl;
	int linear_edge_plot_interval = 30;
	plot_all_edges(Tree, linear_edge_plot_interval);
	plot_obs_start_goal(q_start, q_goal, x_max, y_max, obs, close_range);

	remove("valid_curves.dat");
	remove("valid_edges.dat");

	//record nodes in the Path
	for (int m = Path.size() - 2; m > 0; m--)
	{
		std::array<double,2> W1 = Tree[Path[m]].midcoord;
		std::array<double, 2> W2 = Tree[Path[m]].coord;
		std::array<double, 2> W3 = Tree[Path[(int)m-1]].midcoord;

		double k_before = Tree[Path[(int)m - 1]].linebefore[0];
		double b_before = Tree[Path[(int)m - 1]].linebefore[1];
		double k_after = Tree[Path[(int)m - 1]].lineafter[0];
		double b_after = Tree[Path[(int)m - 1]].lineafter[1];

		ValidCurve(W1, W2, W3, kmax, gamma_max, t, k_before, b_before, k_after, b_after, linear_edge_plot_interval);

	}

	return 0;
}