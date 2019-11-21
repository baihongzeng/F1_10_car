#include <ros/ros.h>
#include<fstream>
#include<iostream>
#include<math.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <tf/tf.h>

// TODO: include ROS msg type headers and libraries you need

using namespace std;


class PurePursuit {
private:
    ros::NodeHandle n;
    // TODO: create ROS subscribers and publishers
    ros::Publisher drive_pub;
    ros::Publisher vis_pub;
    ros::Subscriber pose_sub;

    visualization_msgs::Marker marker;//self setted to view the waypoints
    ackermann_msgs::AckermannDriveStamped drive_msg;

    vector<double> x_data;//the x-coordinate of waypoints
    vector<double> y_data;//the y-coordinate of waypoints
    string temp;

    double L = 1.3;//lookahead distance
    double p = 0.25;//proportional gain between curvature and steering angle
    double velocity;

    double roll, pitch, yaw;
    double x, y;
    double delta_x, delta_y;

    double dis, gap, min_gap;
    int k = 0;
    double x_d, y_d, L_d_square, gamma;    
    double angle;
    

public:
    PurePursuit() {
        n = ros::NodeHandle();

        // TODO: create ROS subscribers and publishers
        pose_sub = n.subscribe("pf/pose/odom", 100, &PurePursuit::pose_callback, this);
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 10);
        vis_pub = n.advertise<visualization_msgs::Marker>("waypoints_marker", 1000);

        ifstream file("/home/baihong/baihong_ws/src/F1_10_car/baihong_zeng_pure_pursuit/waypoints.csv");

        while(file.good()){
            getline(file, temp, ',');
            x_data.push_back(stod(temp));
            getline(file, temp, '\n');
            y_data.push_back(stod(temp));
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
            for (auto i=0; i<x_data.size(); ++i){
                marker.id = i;
                marker.header.stamp = ros::Time::now();
                marker.pose.position.x = x_data[i];
                marker.pose.position.y = y_data[i];
                vis_pub.publish(marker);
                ros::Duration(0.01).sleep();
            }
        }
        
        
    }

    void pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg) {
        // TODO: find the current waypoint to track using methods mentioned in lecture
        
        x = (pose_msg->pose).pose.position.x;
        y = (pose_msg->pose).pose.position.y;

        tf::Quaternion q(
            (pose_msg->pose).pose.orientation.x,
            (pose_msg->pose).pose.orientation.y,
            (pose_msg->pose).pose.orientation.z,
            (pose_msg->pose).pose.orientation.w);

        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        ROS_INFO("YAW:%f", yaw);
        

        vector<double> x_car(x_data.size(), 0);
        vector<double> y_car(y_data.size(), 0);
        //record the coordinate of each waypoint on the car frame instead of map frame
        for (auto i=0; i<x_data.size(); ++i){
            delta_x = x_data[i] - x;
            delta_y = y_data[i] - y;
            y_car[i] = -delta_x*sin(yaw) + delta_y*cos(yaw);
            x_car[i] = delta_x*cos(yaw) + delta_y*sin(yaw);
        }
        min_gap = DBL_MAX;
        //find the closest waypoint
        for (auto i=0; i<x_car.size(); ++i){
            dis = sqrt(x_car[i]*x_car[i] + y_car[i]*y_car[i]);
            gap = abs(dis-L);
            if ((gap < min_gap) && (x_car[i]>=0.0)){
                k = i;
                min_gap = gap;
            }

        }
        //calculate arc radius gamma
        cout << k << endl;
        x_d = x_car[k];
        y_d = y_car[k];
        L_d_square = x_d*x_d + y_d*y_d;
        gamma = 2*y_d/(L_d_square);
        //should have another variable:curvature = 1 / gamma;
        angle = p*gamma;//steering angle is proportional to the curvature
        angle = min(angle, 0.4189);
        angle = max(angle, -0.4189);
        
        if (abs(angle*180/M_PI)<10){
            velocity = 4.5;
        }else if (abs(angle*180/M_PI)<20){
            velocity = 2;
        }else{
            velocity = 1.5;
        }

        drive_msg.header.stamp = ros::Time::now();
        drive_msg.header.frame_id = "laser";
        drive_msg.drive.steering_angle = angle;
        drive_msg.drive.speed = velocity;
        
        drive_pub.publish(drive_msg);

        

        // TODO: transform goal point to vehicle frame of reference

        // TODO: calculate curvature/steering angle

        // TODO: publish drive message, don't forget to limit the steering angle between -0.4189 and 0.4189 radians
        

    }

};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "pure_pursuit_node");
    PurePursuit pp;
    ros::spin();
    return 0;
}
