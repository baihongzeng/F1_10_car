// ESE 680
// RRT assignment
// Author: Hongrui Zheng

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
// Make sure you have read through the header file as well

#include "baihong_rrt/baihong_rrt.h"

// Destructor of the RRT class
RRT::~RRT() {
    // Do something in here, free up used memory, print message, etc.
    ROS_INFO("RRT shutting down");
}

// Constructor of the RRT class
RRT::RRT(ros::NodeHandle &nh): nh_(nh), gen((std::random_device())()) {

    // TODO: Load parameters from yaml file, you could add your own parameters to the rrt_params.yaml file
    std::string pose_topic, scan_topic;
    nh_.getParam("pose_topic", pose_topic);
    nh_.getParam("scan_topic", scan_topic);

    // ROS publishers
    // TODO: create publishers for the the drive topic, and other topics you might need
    env_pub = nh_.advertise<nav_msgs::OccupancyGrid>("env_layer", 10, true); //"env_layer" is a topic name
    dyn_pub = nh_.advertise<nav_msgs::OccupancyGrid>("dyn_layer", 10, true); //"dyn_layer" is a topic name

    // ROS subscribers
    // TODO: create subscribers as you need
    pf_sub_ = nh_.subscribe("/pf/pose/odom", 10, &RRT::pf_callback, this);
    scan_sub_ = nh_.subscribe("/scan", 10, &RRT::scan_callback, this);

    // TODO: create a occupancy grid
    map_sub = nh_.subscribe("/map", 10, &RRT::map_callback, this);
    map_meta_sub = nh_.subscribe("/map_metadata", 10, &RRT::map_meta_callback, this);

    ROS_INFO("Created new RRT Object.");
}

void RRT::map_meta_callback(const nav_msgs::MapMetaData::ConstPtr& map_meta_msg) {
    env_metadata_msg = *map_meta_msg;
    map_resolution = env_metadata_msg.resolution;
    ROS_INFO("resolu:%f",map_resolution);
    map_width = env_metadata_msg.width;
    map_height = env_metadata_msg.height;
    map_origin = env_metadata_msg.origin;
    geometry_msgs::Point origin = map_origin.position;
    origin_x = origin.x;
    origin_y = origin.y;
}
//assumption: the map is square
void RRT::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg) {

    env_msg = *map_msg;
    std::vector<int8_t> map_data = env_msg.data;
    // convert to int
    std::vector<int> map_data_int(map_data.begin(), map_data.end());//the environment map in 1d
    std::vector<std::vector<int>> map_data_int_2d (row_col_size, std::vector<int>(row_col_size, 0));
    for(int i = 0; i < row_col_size; i++){ //the environment in 2d
        for(int j = 0; j < row_col_size; j++) {
            int m = i * row_col_size + j;
            map_data_int_2d[i][j] = map_data_int[m];
        }
    }
    // save data to attribute
    // map value 100 if occupied, 0 if free

    ROS_INFO("Map rerouted.");
    env_pub.publish(env_msg);
}

void RRT::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
    // The scan callback, update your occupancy grid here
    // Args:
    //    scan_msg (*LaserScan): pointer to the incoming scan message
    // Returns:
    //
    // TODO: update your occupancy grid
    std::vector<float> ranges = scan_msg->ranges;
    int scan_count = ranges.size();
    std::vector<float> current_scan = ranges;
    std::vector<float> angles_vector(scan_count, 0.0);
    for (int i=0; i<scan_count; i++) {
        angles_vector[i] = scan_msg->angle_min+scan_msg->angle_increment*i;
    }
    std::vector<int> dyn_data_int(row_col_size*row_col_size, 0);//initialize dynamic data 1d array

    // put scan into dynamic layer
    for (int i=0; i< scan_count; i++) {
        double range = ranges[i];
        if (std::isnan(range) || std::isinf(range)) continue;//ignore invalid reading
        // these are in the frame of /laser
        double x = range*cos(angles_vector[i]), y = range*sin(angles_vector[i]);
        // transform into map frame
        geometry_msgs::PointStamped before_tf;
        before_tf.point.x = x;
        before_tf.point.y = y;
        before_tf.header.frame_id = "/laser";
        geometry_msgs::PointStamped after_tf;
        after_tf.header.frame_id = "/map";
        listener.transformPoint("/map", before_tf, after_tf);
        std::vector<int> laser_rc = coord_2_cell_rol_col(after_tf.point.x, after_tf.point.y);
        int laser_r = laser_rc[0];
        int laser_c = laser_rc[1];
        // check bounds
        if (out_of_bounds(laser_r, laser_c)) continue;

        // add inflation
        for (int i_f=-INFLATION; i_f<=INFLATION; i_f++) {
            for (int j_f=-INFLATION; j_f<=INFLATION; j_f++) {
                int current_r = laser_r - i_f, current_c = laser_c - j_f;
                if (out_of_bounds(current_r, current_c)) continue;
                //dynamic_layer[current_r][current_c] = 100;
                dyn_data_int[current_r*row_col_size+current_c] = 100;
            }
        }
    }

    std::vector<int8_t> dyn_data(dyn_data_int.begin(), dyn_data_int.end());
    dyn_msg.data = dyn_data;
    dyn_msg.info = env_metadata_msg;
    dyn_pub.publish(dyn_msg);

}

bool RRT::out_of_bounds(int r, int c) {
    return (r < 0 || c < 0 || r >= map_height || c >= map_width);
}

// returns rc index
std::vector<int> RRT::coord_2_cell_rol_col(double x, double y){
    std::vector<int> rc;
    int col = static_cast<int>((x - origin_x) / map_resolution);
    int row = static_cast<int>((y - origin_y) / map_resolution);
    rc.push_back(row);
    rc.push_back(col);
    return rc;
}

void RRT::pf_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {
    // The pose callback when subscribed to particle filter's inferred pose
    // The RRT main loop happens here
    // Args:
    //    pose_msg (*PoseStamped): pointer to the incoming pose message
    // Returns:
    //

    // tree as std::vector
    std::vector<Node> tree;

    // TODO: fill in the RRT main loop



    // path found as Path message

}

std::vector<double> RRT::sample() {
    // This method returns a sampled point from the free space
    // You should restrict so that it only samples a small region
    // of interest around the car's current position
    // Args:
    // Returns:
    //     sampled_point (std::vector<double>): the sampled point in free space

    std::vector<double> sampled_point;//x y coordinate
    // TODO: fill in this method
    // look up the documentation on how to use std::mt19937 devices with a distribution
    // the generator and the distribution is created for you (check the header file)
    
    return sampled_point;
}

int RRT::nearest(std::vector<Node> &tree, std::vector<double> &sampled_point) {
    // This method returns the nearest node on the tree to the sampled point
    // Args:
    //     tree (std::vector<Node>): the current RRT tree
    //     sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //     nearest_node (int): index of nearest node on the tree

    int nearest_node = 0;
    // TODO: fill in this method

    return nearest_node;
}

Node RRT::steer(Node &nearest_node, std::vector<double> &sampled_point) {
    // The function steer:(x,y)->z returns a point such that z is “closer” 
    // to y than x is. The point z returned by the function steer will be 
    // such that z minimizes ||z−y|| while at the same time maintaining 
    //||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0

    // basically, expand the tree towards the sample point (within a max dist)

    // Args:
    //    nearest_node (Node): nearest node on the tree to the sampled point
    //    sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //    new_node (Node): new node created from steering

    Node new_node;
    // TODO: fill in this method

    return new_node;
}

bool RRT::check_collision(Node &nearest_node, Node &new_node) {
    // This method returns a boolean indicating if the path between the 
    // nearest node and the new node created from steering is collision free
    // Args:
    //    nearest_node (Node): nearest node on the tree to the sampled point
    //    new_node (Node): new node created from steering
    // Returns:
    //    collision (bool): true if in collision, false otherwise

    bool collision = false;
    // TODO: fill in this method

    return collision;
}

bool RRT::is_goal(Node &latest_added_node, double goal_x, double goal_y) {
    // This method checks if the latest node added to the tree is close
    // enough (defined by goal_threshold) to the goal so we can terminate
    // the search and find a path
    // Args:
    //   latest_added_node (Node): latest addition to the tree
    //   goal_x (double): x coordinate of the current goal
    //   goal_y (double): y coordinate of the current goal
    // Returns:
    //   close_enough (bool): true if node close enough to the goal

    bool close_enough = false;
    // TODO: fill in this method

    return close_enough;
}

std::vector<Node> RRT::find_path(std::vector<Node> &tree, Node &latest_added_node) {
    // This method traverses the tree from the node that has been determined
    // as goal
    // Args:
    //   latest_added_node (Node): latest addition to the tree that has been
    //      determined to be close enough to the goal
    // Returns:
    //   path (std::vector<Node>): the vector that represents the order of
    //      of the nodes traversed as the found path
    
    std::vector<Node> found_path;
    // TODO: fill in this method

    return found_path;
}

// RRT* methods
double RRT::cost(std::vector<Node> &tree, Node &node) {
    // This method returns the cost associated with a node
    // Args:
    //    tree (std::vector<Node>): the current tree
    //    node (Node): the node the cost is calculated for
    // Returns:
    //    cost (double): the cost value associated with the node

    double cost = 0;
    // TODO: fill in this method

    return cost;
}

double RRT::line_cost(Node &n1, Node &n2) {
    // This method returns the cost of the straight line path between two nodes
    // Args:
    //    n1 (Node): the Node at one end of the path
    //    n2 (Node): the Node at the other end of the path
    // Returns:
    //    cost (double): the cost value associated with the path

    double cost = 0;
    // TODO: fill in this method

    return cost;
}

std::vector<int> RRT::near(std::vector<Node> &tree, Node &node) {
    // This method returns the set of Nodes in the neighborhood of a 
    // node.
    // Args:
    //   tree (std::vector<Node>): the current tree
    //   node (Node): the node to find the neighborhood for
    // Returns:
    //   neighborhood (std::vector<int>): the index of the nodes in the neighborhood

    std::vector<int> neighborhood;
    // TODO:: fill in this method

    return neighborhood;
}