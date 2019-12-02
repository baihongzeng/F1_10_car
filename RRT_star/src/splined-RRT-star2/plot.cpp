#include "NodeClass.h"
//#include "splined-RRT-star.h"
#include <vector>
#include <tuple>
#include <array>
#include <fstream>
#include "constants.h"
#include "plot.h"

//purpose: plot / store out obstacles, qstart, qgoal in .dat file
void plot_obs_start_goal(NodeClass q_start, NodeClass q_goal, double x_max, double y_max, std::vector <std::array<double, 3>> obs, double close_range)
{
	remove("circle.dat");
	remove("scatter.dat");
	if (obs.size() != 0)
	{
		int NumObs = obs.size();
		std::ofstream plot_circle_file; //instantiate a std::ofstream object
		for (int Num = 0; Num < NumObs; Num++)
		{
			plot_circle_file.open("circle.dat", std::ios::app); // make a .dat file called "circle.dat" containing all obstacles

			std::tuple<std::array<double, t_size>, std::array<double, t_size>> circle_tuple = circle(obs[Num][0], obs[Num][1], obs[Num][2]);

			for (int i = 0; i < t_size; i++) //assign values to x_final, y_final
			{
				plot_circle_file << std::get<0>(circle_tuple)[i] << " " << std::get<1>(circle_tuple)[i] << std::endl; //store values in "circle(number).dat"
			}
			plot_circle_file.close();
		}
	}
	std::ofstream close_range_logging;
	close_range_logging.open("circle.dat", std::ios::app);
	std::tuple<std::array<double, t_size>, std::array<double, t_size>> close_range_tuple = circle(q_goal.coord[0], q_goal.coord[1], close_range);
	for (int i = 0; i < t_size; i++) //assign values to x_final, y_final
	{
		close_range_logging << std::get<0>(close_range_tuple)[i] << " " << std::get<1>(close_range_tuple)[i] << std::endl; //store values in "circle(number).dat"
	}
	close_range_logging.close();

	//mark q_start, q_goal on the map
	scatter(q_start.coord);
	scatter(q_goal.coord);

}

//purpose: plot / store given node: "coordinate"
void scatter(COORD coordinate)
{
	std::ofstream plot_scatter_file; //instantiate a std::ofstream object

	plot_scatter_file.open("scatter.dat", std::ios::app); // make a .dat file called "circle(number).dat"
	plot_scatter_file << coordinate[0] << " " << coordinate[1] << std::endl;
	plot_scatter_file.close();

}

//purpose: plot / store the circles (i.e. obstacles, close range circle)
std::tuple<std::array<double, t_size>, std::array<double, t_size>> circle(double center_x, double center_y, double radius)
{
	double theta_discrete = 0;
	std::array<double, t_size> x_final = {}; //initialize array to store x values
	std::array<double, t_size> y_final = {}; //initialize array to store y values

	for (int i = 0; i < t_size; i++) //assign values to x_final, y_final
	{
		theta_discrete = i * 2.0 * pi / (t_size - 1.0);
		x_final[i] = radius * cos(theta_discrete) + center_x;
		y_final[i] = radius * sin(theta_discrete) + center_y;
	}

	//save the interpolation points(x_final, y_final) to "circle_tuple"
	std::tuple <std::array<double, t_size>, std::array<double, t_size>> circle_tuple = std::make_tuple(x_final, y_final);

	return circle_tuple;
}

//purpose: plot all edges in the tree
void plot_all_edges(std::array<NodeClass, tree_size> & Tree, int& interpolation_num)
{
	remove("curves.dat");
	remove("linear_edges.dat");
	int tree_size = 0;
	for (int i = 0; i < (int)Tree.size(); i++)
	{
		if (Tree[i].parent == -1)
		{
			tree_size = i; //tree.size()
			break;
		}
	}

	std::ofstream plot_curve_file; //instantiate a std::ofstream object
	for (int plot_index = 2; plot_index < tree_size; plot_index++)
	{
		plot_curve_file.open("curves.dat", std::ios::app); // make a .dat file called "circle.dat" containing all obstacles
		for (int i = 0; i < t_size; i++)
		{
			plot_curve_file << Tree[plot_index].curve[0].x[i] << " " << Tree[plot_index].curve[0].y[i] << std::endl;
			plot_curve_file << Tree[plot_index].curve[1].x[i] << " " << Tree[plot_index].curve[1].y[i] << std::endl;
			//Tree[plot_index].curve[0].x;
		}
		plot_curve_file.close();
		std::string filename = "linear_edges.dat";
		plot_two_lines(Tree[Tree[plot_index].parent].midcoord[0], Tree[plot_index].curve[0].x[0], Tree[plot_index].curve[1].x[0], Tree[plot_index].midcoord[0],
			interpolation_num, Tree[plot_index].linebefore[0], Tree[plot_index].linebefore[1], Tree[plot_index].lineafter[0], Tree[plot_index].lineafter[1], filename);

	}
}

//purpose: plot two straight lines connecting every cubic bezier curves
void plot_two_lines(double & L1_start_x, double & L1_end_x, double & L2_start_x, double & L2_end_x, 
	int & interpolation_num, double & k_before, double & b_before, double & k_after, double & b_after, std::string filename)
{
	
	std::ofstream plot_edge_file; //instantiate a std::ofstream object
	double x1;
	double y1;
	double dx1 = fabs(L1_end_x-L1_start_x) / (interpolation_num - 1.0);
	//const char filename = file_name;
	if (L1_end_x > L1_start_x)
	{
		plot_edge_file.open(filename, std::ios::app); // make a .dat file called "circle.dat" containing all obstacles
		for (int i = 0; i < interpolation_num; i++)
		{
			x1 = i * dx1 + L1_start_x;
			y1 = k_before * x1 + b_before;
			plot_edge_file << x1 << " " << y1 << std::endl;
		}
		plot_edge_file.close();
	}
	else if (L1_end_x <= L1_start_x)
	{
		plot_edge_file.open(filename, std::ios::app); // make a .dat file called "circle.dat" containing all obstacles
		for (int i = 0; i < interpolation_num; i++)
		{
			x1 = i * dx1 + L1_end_x;
			y1 = k_before * x1 + b_before;
			plot_edge_file << x1 << " " << y1 << std::endl;
		}
		plot_edge_file.close();
	}

	double x2;
	double y2;
	double dx2 = fabs(L2_end_x - L2_start_x) / (interpolation_num - 1.0);
	if (L2_end_x > L2_start_x)
	{
		plot_edge_file.open(filename, std::ios::app); // make a .dat file called "circle.dat" containing all obstacles
		for (int i = 0; i < interpolation_num; i++)
		{
			x2 = i * dx2 + L2_start_x;
			y2 = k_after * x2 + b_after;
			plot_edge_file << x2 << " " << y2 << std::endl;
		}
		plot_edge_file.close();
	}
	else if (L2_end_x <= L2_start_x)
	{
		plot_edge_file.open(filename, std::ios::app); // make a .dat file called "circle.dat" containing all obstacles
		for (int i = 0; i < interpolation_num; i++)
		{
			x2 = i * dx2 + L2_end_x;
			y2 = k_after * x2 + b_after;
			plot_edge_file << x2 << " " << y2 << std::endl;
		}
		plot_edge_file.close();
	}
	
}