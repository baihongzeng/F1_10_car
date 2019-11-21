// ESE 680
// RRT assignment
// Author: Hongrui Zheng

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
// Make sure you have read through the header file as well

#include "rrt/rrt.h"

// Destructor of the RRT class
RRT::~RRT() {
    // Do something in here, free up used memory, print message, etc.
    ROS_INFO("RRT shutting down");
}

// Constructor of the RRT class
RRT::RRT(ros::NodeHandle &nh): nh_(nh), tfListener(tfBuffer), gen((std::random_device())()) {

    // TODO: Load parameters from yaml file, you could add your own parameters to the rrt_params.yaml file
    std::string pose_topic, scan_topic,map_topic;


    nh_.getParam("pose_topic", pose_topic);
    nh_.getParam("scan_topic", scan_topic);
    nh_.getParam("map_topic",map_topic );
    nh_.getParam("lookahead", lookahead);
    nh_.getParam("inflation_size",inflation_size);
    nh_.getParam("step_size", step_size);
    nh_.getParam("goal_threshold", goal_threshold);
    nh_.getParam("MAX_ITER", MAX_ITER);
    nh_.getParam("interpolation_number",interpolation_number);
    nh_.getParam("sample_goal_prob_reciprocal",sample_goal_prob_reciprocal);
    nh_.getParam("Kp",Kp);
    // ROS publishers
    // TODO: create publishers for the the drive topic, and other topics you might need
    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/occumap",1,true);
    drive_pub = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 10);
    // ROS subscribers
    // TODO: create subscribers as you need
    pf_sub_ = nh_.subscribe(pose_topic, 10, &RRT::pf_callback, this);
    scan_sub_ = nh_.subscribe(scan_topic, 10, &RRT::scan_callback, this);

   boost::shared_ptr<nav_msgs::MapMetaData const> map_ptr;
   nav_msgs::MapMetaData map_msg;

   map_ptr = ros::topic::waitForMessage<nav_msgs::MapMetaData>(map_topic);
   if(map_ptr != NULL){
       map_msg = *map_ptr;
   }

   boost::shared_ptr<nav_msgs::OccupancyGrid const> levine_map_ptr;
   levine_map_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map");


    occumap_static.data = levine_map_ptr->data;
    occumap.header.frame_id = occumap_static.header.frame_id = occumap_dynamic.header.frame_id ="map";
    occumap_static.header.stamp = occumap.header.stamp =occumap_dynamic.header.stamp = ros::Time::now();
    occumap_static.info.height =  occumap.info.height =occumap_dynamic.info.height = map_msg.height;
    occumap_static.info.origin = occumap.info.origin =occumap_dynamic.info.origin = map_msg.origin;
    occumap_static.info.width =occumap.info.width = occumap_dynamic.info.width = map_msg.width;
    occumap_static.info.resolution = occumap.info.resolution = occumap_dynamic.info.resolution = map_msg.resolution;
    occumap_static.info.map_load_time = occumap.info.map_load_time = occumap_dynamic.info.map_load_time = map_msg.map_load_time;

    occumap.data.assign(occumap.info.width*occumap.info.height, 0);

    occumap_dynamic.data.assign(occumap_dynamic.info.width*occumap_dynamic.info.height, 0);

    // TODO: create a occupancy grid

    ROS_INFO("Created new RRT Object.");

    //visualize waypoint
    vis_pub = nh_.advertise<visualization_msgs::Marker>("waypoints_marker", 1000);
    std::ifstream file("/home/baihong/baihong_ws/src/F1_10_car/rrt/waypoints.csv");

    while(file.good()){
        getline(file, temp, ',');
        waypoint_x_data.push_back(stod(temp));
        getline(file, temp, '\n');
        waypoint_y_data.push_back(stod(temp));
    }

    file.close();
    marker.header.frame_id = "map";
    marker.ns = "waypoints_vis";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    for (auto j=0; j<2; ++j){
        for (auto i=0; i<waypoint_x_data.size(); ++i){
            marker.id = i;
            marker.header.stamp = ros::Time::now();
            marker.pose.position.x = waypoint_x_data[i];
            marker.pose.position.y = waypoint_y_data[i];
            vis_pub.publish(marker);
            ros::Duration(0.01).sleep();
        }
    }

}

void RRT::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
    // The scan callback, update occupancy grid here
    // Args:
    //    scan_msg (*LaserScan): pointer to the incoming scan message

    range_laser = scan_msg->ranges;
    float min_angle = scan_msg->angle_min;
    float angle_inc = scan_msg->angle_increment;
    Update(min_angle,angle_inc);

    for(int i = 1; i<occumap_dynamic.info.width * occumap_dynamic.info.height; i++){

            occumap.data[i] = ((occumap_dynamic.data[i]  || occumap_static.data[i]) !=0) ? 100:0;

    }
    ROS_INFO("yo:%d", 1);
    map_pub_.publish(occumap);
    occumap_dynamic.data.assign(occumap_dynamic.info.width*occumap_dynamic.info.height, 0);

    // TODO: update your occupancy grid
}
//dynamic layer map update
void RRT::Update(float min_angle,float angle_inc){

    int array_size = 0;
    min_angle = min_angle + 60*M_PI/180;

    geometry_msgs::PointStamped lidar_pt;
    geometry_msgs::PointStamped map_pt;

    lidar_pt.header.frame_id = "laser";
    lidar_pt.point.z = 0;

    for(std::vector<float>::iterator it = range_laser.begin() + 180; it != range_laser.end() - 180; ++it) {

        array_size++;

        if (std::isnan(*it) || std::isinf(*it)) {
            range_laser.erase(it);
            continue;
        }
        lidar_pt.point.x = (*it)* cos(min_angle + (array_size - 1) * angle_inc);
        lidar_pt.point.y = (*it)* sin(min_angle + (array_size - 1) * angle_inc);

        map_pt = transformFromLidar2Map(lidar_pt);

        int col = getColFromPoint(map_pt.point.x);
        int row = getRowFromPoint(map_pt.point.y);

        inflation(row, col,(*it));
        }
}

int RRT::getRowFromPoint(double y){
    return static_cast<int>((y - occumap.info.origin.position.y) / occumap.info.resolution);
}

int RRT::getColFromPoint(double x){
    return static_cast<int>((x - occumap.info.origin.position.x) / occumap.info.resolution);
}

geometry_msgs::PointStamped RRT::transformFromLidar2Map(geometry_msgs::PointStamped lidar_pt){

    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::PointStamped map_pt;

    try {
        transformStamped = tfBuffer.lookupTransform("map", "laser", ros::Time(0));
        tf2::doTransform(lidar_pt, map_pt, transformStamped);

    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("Failure %s\n", ex.what());
        ros::Duration(1.0).sleep();
    }
    return map_pt;

}

void RRT::inflation(int row, int col, float range){

    int inf_side = 2*(inflation_size)+1;

    std::vector<int> cols(inf_side,0);
    std::vector<int> rows(inf_side,0);

    int ctmp = col-inflation_size;
    int rtmp = row-inflation_size;

    for(int i =0; i< inf_side;i++){

        cols[i] = ctmp;
        ctmp ++;
        rows[i] = rtmp;
        rtmp ++;

    }

   int start =  0 ;
   int end = rows.size();

       for (int i = start; i < end; i++) {
           for (int j = start; j < end; j++) {

               occumap_dynamic.data[rc_2_ind(rows[i], cols[j])] = 100;
           }
       }
   }

int RRT::rc_2_ind(int r, int c){
    return (r*occumap.info.width + c);
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
    std::vector<Node> path;
    Node start_Node;

    double goal_x;//in global frame
    double goal_y;

    start_Node.x = pose_msg->pose.position.x;
    start_Node.y = pose_msg->pose.position.y;
    start_Node.is_root = true;

    for(int i = 0; i < MAX_ITER; i++) {
        std::vector<double> sample_point = sample(goal_x, goal_y);
        int nearest_point_ind = nearest(tree, sample_point);
        Node new_point = steer(tree[nearest_point_ind], sample_point, nearest_point_ind);
        bool collision = check_collision(tree[nearest_point_ind], new_point);

        if (collision == true) {
            continue;
        } else {
            if( is_goal(new_point, goal_x, goal_y)){

                path = find_path(tree, new_point);
                break;
            }

        }
    }

//    tf::Quaternion q(
//            (pose_msg->pose).orientation.x,
//            (pose_msg->pose).orientation.y,
//            (pose_msg->pose).orientation.z,
//            (pose_msg->pose).orientation.w);
//
//    tf::Matrix3x3 m(q);
//    m.getRPY(roll, pitch, yaw);
//
//    std::vector<double> x_car(path.size()-1, 0);//path point coordinate in laser frame
//    std::vector<double> y_car(path.size()-1, 0);
//
//    // path found as Path message
//    for (auto i=1; i<path.size(); ++i){
//        delta_x = path[i].x - start_Node.x;
//        delta_y = path[i].y - start_Node.y;
//        y_car[i] = -delta_x*sin(yaw) + delta_y*cos(yaw);
//        x_car[i] = delta_x*cos(yaw) + delta_y*sin(yaw);
//    }
//    min_gap = DBL_MAX;//initialize min_gap
//    //coordinate of closest path point in laser frame
//    for (auto i=0; i<x_car.size(); ++i){
//        dis = sqrt(x_car[i]*x_car[i] + y_car[i]*y_car[i]);
//        gap = abs(dis-lookahead);
//        if ((gap < min_gap) && (x_car[i]>=0.0)){
//            k = i;
//            min_gap = gap;
//        }
//    }
//
//    //calculate arc radius gamma
//    std::cout << k << std::endl;
//    x_d = x_car[k];
//    y_d = y_car[k];
//    L_d_square = x_d*x_d + y_d*y_d;
//    gamma = 2*y_d/(L_d_square);
//    //should have another variable:curvature = 1 / gamma;
//    angle = Kp*gamma;//steering angle is proportional to the curvature
//    angle = std::min(angle, 0.4189);
//    angle = std::max(angle, -0.4189);
//
//    if (abs(angle*180/M_PI)<10){
//        velocity = 4.5;
//    }else if (abs(angle*180/M_PI)<20){
//        velocity = 2;
//    }else{
//        velocity = 1.5;
//    }
//
//    drive_msg.header.stamp = ros::Time::now();
//    drive_msg.header.frame_id = "laser";
//    drive_msg.drive.steering_angle = angle;
//    drive_msg.drive.speed = velocity;
//
//    drive_pub.publish(drive_msg);
}

std::vector<double> RRT::sample(double& goal_x, double goal_y) {
    // This method returns a sampled point from the free space
    // You should restrict so that it only samples a small region
    // of interest around the car's current position
    // Args:
    // Returns:
    //sampled_point (std::vector<double>): size:2; the sampled point in free space
    // TODO: fill in this method
    // look up the documentation on how to use std::mt19937 devices with a distribution
    // the generator and the distribution is created for you (check the header file)
    std::vector<double> sampled_point;

    geometry_msgs::PointStamped lidar_pt;
    geometry_msgs::PointStamped map_pt;
    lidar_pt.header.frame_id = "laser";
    int free_indicator = 0;
    int prob_indi = 0;
    while(free_indicator == 0){
        prob_indi++;
        //with certain probability, the sample point is goal point
        if(prob_indi % sample_goal_prob_reciprocal == 0){
            lidar_pt.point.x = goal_x;
            lidar_pt.point.y = goal_y;
            lidar_pt.point.z = 0;
        }
        else{
            lidar_pt.point.x = x_dist(gen);
            lidar_pt.point.y = y_dist(gen);
            lidar_pt.point.z = 0;
        }


        map_pt = transformFromLidar2Map(lidar_pt);
        double x_g = map_pt.point.x;
        double y_g = map_pt.point.y;
        int index = rc_2_ind(getRowFromPoint(y_g),getColFromPoint(x_g));

        if(occumap.data[index]==0){//free space
            sampled_point.push_back(x_g);
            sampled_point.push_back(y_g);
            free_indicator = 1;
        }
    }
    return sampled_point;//expect to be never reached
}

int RRT::nearest(std::vector<Node> &tree, std::vector<double> &sampled_point) {
    // This method returns the nearest node on the tree to the sampled point
    // Args:
    //     tree (std::vector<Node>): the current RRT tree
    //     sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //     nearest_node (int): index of nearest node on the tree

    int nearest_node = 0;
    float cur_dist;
    float min_dist = FLT_MAX;
    for(int i = 0; i < tree.size(); i++) {
        cur_dist = pow(tree[i].x - sampled_point[0], 2) + pow(tree[i].y - sampled_point[1], 2);
        if (cur_dist < min_dist) {
            nearest_node = i;
            min_dist = cur_dist;
        }
    }
    return nearest_node;
}

Node RRT::steer(Node &nearest_node, std::vector<double> &sampled_point, int& nearest_point_index) {
    // The function steer:(x,y)->z returns a point such that z is “closer” 
    // to y than x is. The point z returned by the function steer will be 
    // such that z minimizes ||z−y|| while at the same time maintaining 
    //||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0

    // y is on the tree and x is the sampled point, z is the point along x-y direction which will
    // be added on the tree. basically, expand the tree towards the sample point (within a max dist)

    // Args:
    //    nearest_node (Node): nearest node on the tree to the sampled point
    //    sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //    new_node (Node): new node created from steering
    Node new_node;
    float dist = sqrt(pow(nearest_node.x - sampled_point[0], 2) + pow(nearest_node.y - sampled_point[1], 2));
    if(dist > step_size){
        new_node.x = (step_size / dist) * (sampled_point[0] - nearest_node.x) + nearest_node.x;
        new_node.y = (step_size / dist) * (sampled_point[1] - nearest_node.y) + nearest_node.y;
        new_node.is_root = false;
        new_node.parent = nearest_point_index;
    }
    else{
        new_node.x = sampled_point[0];
        new_node.y = sampled_point[1];
        new_node.is_root = false;
        new_node.parent = nearest_point_index;
    }
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
    for(int i = 0; i < interpolation_number; i++) {
        new_node.x = (float)i / (float) interpolation_number * (new_node.x - nearest_node.x) + nearest_node.x;
        new_node.y = (float)i / (float) interpolation_number * (new_node.y - nearest_node.y) + nearest_node.y;
        int index = rc_2_ind(getRowFromPoint(new_node.y), getColFromPoint(new_node.x));
        if (occumap.data[index] == 100) {
            collision = true;
            break;
        }
    }
    return collision;
}// add interpolation_number

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
    float dist = sqrt(pow(latest_added_node.x - goal_x, 2) + pow(latest_added_node.y - goal_y, 2));
    close_enough = (dist > goal_threshold) ? false : true;
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
    found_path.push_back(latest_added_node);
    int i = tree.size() - 1;
    while(i != 0){
        found_path.push_back(tree[tree[i].parent]);
        i = tree[i].parent;
    }
    std::reverse(found_path.begin(),found_path.end());
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