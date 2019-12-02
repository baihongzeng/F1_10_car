//#include <rrt/pure_pursuit.h>
//
////
//// Created by baihong on 11/21/19.
////
//p_pursuit::p_pursuit(ros::NodeHandle &nh) : nh_(nh), tfListener(tfBuffer), gen((std::random_device()) ()) {
//// TODO: Load parameters from yaml file, you could add your own parameters to the rrt_params.yaml file
//    std::string pose_topic, scan_topic, map_topic;
//    pose_topic = "/pf/pose/odom";
//    scan_topic = "/scan";
//    map_topic = "/map_metadata";
////    nh_.getParam("pose_topic", pose_topic);
////    nh_.getParam("scan_topic", scan_topic);
////    nh_.getParam("map_topic",map_topic );
////    nh_.getParam("lookahead_wp", lookahead_wp);
////    nh_.getParam("lookahead_path", lookahead_path);
////    nh_.getParam("inflation_size",inflation_size);
////    nh_.getParam("step_size", step_size);
////    nh_.getParam("goal_threshold", goal_threshold);
////    nh_.getParam("MAX_ITER", MAX_ITER);
////    nh_.getParam("interpolation_number",interpolation_number);
////    nh_.getParam("sample_goal_prob_reciprocal",sample_goal_prob_reciprocal);
////    nh_.getParam("Kp",Kp);
//
//
//// ROS publishers
//    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/occumap", 10, true);
//    drive_pub = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 10);
//    vis_pub = nh_.advertise<visualization_msgs::Marker>("/waypoints_marker", 10);
//    vis_tree_edge_pub = nh_.advertise<visualization_msgs::Marker>("/tree_line", 10);
//    vis_path_edge_pub = nh_.advertise<visualization_msgs::Marker>("/path_line", 10);
//
//// ROS subscribers
//    scan_sub_ = nh_.subscribe(scan_topic, 10, &RRT::scan_callback, this);
//    pf_sub_ = nh_.subscribe(pose_topic, 10, &RRT::pf_callback, this);
//
////save map meta data as 'map_msg', into 'occumap'
//    boost::shared_ptr < nav_msgs::MapMetaData const> map_ptr;
//    nav_msgs::MapMetaData map_msg;
//    map_ptr = ros::topic::waitForMessage<nav_msgs::MapMetaData>(map_topic);
//    if (map_ptr != NULL) {
//        map_msg = *map_ptr;
//    }
////get static map, save data into 'occumap'
//    levine_map_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map");
//    occumap.data = levine_map_ptr->data;
//    occumap.header.frame_id = "map";
//    occumap.header.stamp = ros::Time::now();
//    occumap.info.height = map_msg.height;
//    occumap.info.origin = map_msg.origin;
//    occumap.info.width = map_msg.width;
//    occumap.info.resolution = map_msg.resolution;
//    occumap.info.map_load_time = map_msg.map_load_time;
//
//    ROS_INFO("Created new RRT Object.");
//
////visualize waypoint
//    std::ifstream file("/home/baihong/baihong_ws/src/F1_10_car/RRT_star/waypoints.csv");
//
//    while (file.good()) {
//        getline(file, temp, ',');
//        wp_x_g.push_back(stod(temp));
//        getline(file, temp, '\n');
//        wp_y_g.push_back(stod(temp));
//    }
//
//    file.close();
//
////tree edge visualization marker
//    tree_line_marker.header.frame_id = "map";
//    tree_line_marker.ns = "tree_line_vis";
//    tree_line_marker.type = visualization_msgs::Marker::LINE_LIST;
//    tree_line_marker.action = visualization_msgs::Marker::ADD;
//    tree_line_marker.lifetime = ros::Duration(0.05);
//    tree_line_marker.pose.position.z = 0;
//    tree_line_marker.scale.x = 0.03;
//    tree_line_marker.scale.y = 0.03;
//    tree_line_marker.scale.z = 0.03;
//    tree_line_marker.color.a = 1.0;
//    tree_line_marker.color.r = 1.0;
//    tree_line_marker.color.g = 0.0;
//    tree_line_marker.color.b = 0.0;
//    tree_line_marker.id = 0;
//
////path edge visualization marker
//    path_line_marker.header.frame_id = "map";
//    path_line_marker.ns = "path_line_vis";
//    path_line_marker.type = visualization_msgs::Marker::LINE_STRIP;
//    path_line_marker.action = visualization_msgs::Marker::ADD;
//    path_line_marker.lifetime = ros::Duration(0.05);
//    path_line_marker.pose.position.z = 0;
//    path_line_marker.scale.x = 0.05;
//    path_line_marker.scale.y = 0.05;
//    path_line_marker.scale.z = 0.05;
//    path_line_marker.color.a = 1.0;
//    path_line_marker.color.r = 0.0;
//    path_line_marker.color.g = 1.0;
//    path_line_marker.color.b = 0.0;
//    path_line_marker.id = 0;
//
//}