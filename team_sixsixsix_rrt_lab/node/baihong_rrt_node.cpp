// ESE 680
// RRT assignment
// Author: Hongrui Zheng

// This file contains the node definition for RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf


#include "baihong_rrt/baihong_rrt.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "baihong_rrt");// "rrt" is the node name
    ros::NodeHandle nh;
    RRT rrt(nh);//instantiate a instance
    ros::spin();
    return 0;
}
