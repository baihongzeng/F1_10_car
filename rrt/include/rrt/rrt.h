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
#include <ackermann_msgs/AckermannDrive.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

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
    std::vector<float> range_laser;


    int inflation_size;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    nav_msgs::OccupancyGrid occumap;
    nav_msgs::OccupancyGrid occumap_static;
    nav_msgs::OccupancyGrid occumap_dynamic;

    float step_size;
    float goal_threshold;
    int MAX_ITER;
    int interpolation_number;
    int sample_goal_prob_reciprocal;

    //waypoint stuff
    std::vector<double> waypoint_x_data;//the x-coordinate of waypoints
    std::vector<double> waypoint_y_data;//the y-coordinate of waypoints
    std::string temp;
    visualization_msgs::Marker marker;//self set to view the waypoints
    double roll, pitch, yaw;
    double delta_x, delta_y;
    double dis, gap, min_gap;
    int k = 0;
    double lookahead;
    double Kp;//proportional gain between curvature and steering angle
    double x_d, y_d, L_d_square, gamma;
    double angle;
    double velocity;
    ackermann_msgs::AckermannDriveStamped drive_msg;

    // ros pub/sub
    // TODO: add the publishers and subscribers you need

    ros::Publisher map_pub_;
    ros::Publisher vis_pub;
    ros::Publisher drive_pub;
    ros::Subscriber pf_sub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber meta_sub_;


    // tf stuff
    tf::TransformListener listener;

    // TODO: create RRT params

    // random generator, use this
    std::random_device rd;
    std::mt19937 gen{rd()};


    std::uniform_real_distribution<float> x_dist{0,  + 3.0f };
    std::uniform_real_distribution<float> y_dist{-1.0f,  + 1.0f };


    // callbacks
    // where rrt actually happens
    void pf_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
    // updates occupancy grid
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    //void meta_callback(const nav_msgs::MapMetaData::ConstPtr& meta_msg);

    void Update(float min_angle, float angle_inc);
    geometry_msgs::PointStamped transformFromLidar2Map(geometry_msgs::PointStamped lidar_pt);

    int getRowFromPoint(double y);
    int getColFromPoint(double x);

    int rc_2_ind(int r, int c);
    void inflation(int row, int col, float range);


    // RRT methods
    std::vector<double> sample(double& goal_x, double goal_y);
    int nearest(std::vector<Node> &tree, std::vector<double> &sampled_point);
    Node steer(Node &nearest_node, std::vector<double> &sampled_point, int& nearest_point_index);
    bool check_collision(Node &nearest_node, Node &new_node);
    bool is_goal(Node &latest_added_node, double goal_x, double goal_y);
    std::vector<Node> find_path(std::vector<Node> &tree, Node &latest_added_node);
    // RRT* methods
    double cost(std::vector<Node> &tree, Node &node);
    double line_cost(Node &n1, Node &n2);
    std::vector<int> near(std::vector<Node> &tree, Node &node);

};

