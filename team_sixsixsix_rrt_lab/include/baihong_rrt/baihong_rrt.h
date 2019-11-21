// ESE 680
// RRT assignment
// Author: Hongrui Zheng

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf

// ros
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>

// standard
#include <math.h>
#include <vector>
#include <array>
#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <random>

// Eigen
#include </home/baihong/Documents/eigen-eigen-323c052e1731/Eigen/Dense>
#include </home/baihong/Documents/eigen-eigen-323c052e1731/Eigen/Geometry>

// Struct defining the Node object in the RRT tree.
// More fields could be added to thiis struct if more info needed.
// You can choose to use this or not
typedef struct Node {
    double x, y;
    double cost; // only used for RRT*
    int parent; // index of parent node in the tree vector
    bool is_root = false;
} Node;


class RRT {
public:
    RRT(ros::NodeHandle &nh);
    virtual ~RRT();
private:
    ros::NodeHandle nh_;

    // ros pub/sub
    // TODO: add the publishers and subscribers you need
    ros::Publisher env_pub;
    ros::Publisher dyn_pub;
    ros::Subscriber pf_sub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber map_sub;
    ros::Subscriber map_meta_sub;

    // tf stuff
    tf::TransformListener listener;

    // TODO: create RRT params

    // random generator, use this
    std::mt19937 gen;
    std::uniform_real_distribution<> x_dist;
    std::uniform_real_distribution<> y_dist;
    nav_msgs::OccupancyGrid env_msg;
    nav_msgs::OccupancyGrid dyn_msg;
    nav_msgs::MapMetaData env_metadata_msg;
    float map_resolution; // cell
    int map_width, map_height; // cells
    int row_col_size = 706;
    geometry_msgs::Pose map_origin; // cell(0,0), [m, m, rad]
    // first initialize a vector of int with given default value

    // Use above vector to initialize the two-dimensional vector
    // using the fill constructor
    float origin_x, origin_y;
    int INFLATION = 3;
    // callbacks
    // where rrt actually happens
    void pf_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
    // updates occupancy grid
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void map_meta_callback(const nav_msgs::MapMetaData::ConstPtr& map_meta_msg);
    std::vector<int> coord_2_cell_rol_col(double x, double y);
    bool out_of_bounds(int r, int c);
    // RRT methods
    std::vector<double> sample();
    int nearest(std::vector<Node> &tree, std::vector<double> &sampled_point);
    Node steer(Node &nearest_node, std::vector<double> &sampled_point);
    bool check_collision(Node &nearest_node, Node &new_node);
    bool is_goal(Node &latest_added_node, double goal_x, double goal_y);
    std::vector<Node> find_path(std::vector<Node> &tree, Node &latest_added_node);
    // RRT* methods
    double cost(std::vector<Node> &tree, Node &node);
    double line_cost(Node &n1, Node &n2);
    std::vector<int> near(std::vector<Node> &tree, Node &node);

};

