#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Bool.h>
#include <vector>
// TODO: include ROS msg type headers and libraries

class Safety {
// The class that handles emergency braking
private:

    float threshold_time = 0.4; //initialize the brake threshold time as 1s.
    double speed;
    void invalidNumberFilter(std::vector<float> & array, std::vector<float> & array2, int size)
    {
        std::vector<float>::iterator it;
        for(it = array.begin(); it != array.end(); ++it)
        {
            if (std::isinf(*it) || std::isnan(*it))
            {
                array.erase(it);
                std::vector<float>::iterator it2;
                int a = it-array.begin();
                it2 = array2.begin() + a;
                array2.erase(it2);
            }
        }

    }

    std::vector<float> timeToCollision(const sensor_msgs::LaserScan::ConstPtr & msg, double & relative_velocity)
    {
        std::vector<float>msg_range = msg->ranges;

        float current_angle;//initialize current angle
        int current_index;
        double current_velocity;
        float ttc_denominator;

        std::vector<float>::iterator it;//iterator for msg_range
        std::vector<float> TTC(msg_range.size(), 0);//initialization


        for(it = msg_range.begin(); it != msg_range.end(); it++)
        {
            current_index = it - msg_range.begin();

            current_angle = (msg->angle_increment)*current_index + (msg->angle_min);
            current_velocity = (relative_velocity * cos(current_angle));
            double max_among_vel_zero = std::max(0.0, -current_velocity);

            TTC[current_index] = ((*it)/max_among_vel_zero);
        }
        invalidNumberFilter(msg_range, TTC, msg_range.size());

        return TTC;
    }

    float getMinTime(std::vector<float> & TTC_vector)
    {
        float min_time = TTC_vector[0];
        std::vector<float>::iterator it;
        for(it = TTC_vector.begin(); it !=TTC_vector.end(); it++)
        {
            if ((*it) < min_time)
            {
                min_time = (*it);
            }
        }

        return min_time;
    }

    ros::NodeHandle n;
    ros::Subscriber sub_scan;
    ros::Subscriber sub_odom;
    ros::Publisher pub_brake;
    ros::Publisher pub_brake_tool;
    // TODO: create ROS subscribers and publishers

public:
    Safety() {
        //n = ros::NodeHandle();
        //speed = 0.0;
        /*
        One publisher should publish to the /brake topic with an
        ackermann_msgs/AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a
        std_msgs/Bool message.

        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

        // TODO: create ROS subscribers and publishers
        sub_odom = n.subscribe("odom", 1000, &Safety::odom_callback, this);
        sub_scan = n.subscribe("scan", 10000, &Safety::scan_callback, this);

        pub_brake = n.advertise<ackermann_msgs::AckermannDriveStamped>("brake", 1000);//first argument is the topic name you want to use for publish
        pub_brake_tool = n.advertise<std_msgs::Bool>("brake_bool", 1000);//first argument is the topic name you want to use for publish
    }
    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
        // TODO: update current speed

        speed = odom_msg->twist.twist.linear.x;

        //ROS_INFO("speed is: %f", speed);
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
        // TODO: calculate TTC
        double vx = (0.0-speed);//vx is not the speed of the car
        std::vector<float> TTC_vector = timeToCollision( scan_msg, vx);//get an array containing all TTC
        float min_time = getMinTime(TTC_vector);//the minimum time in the TTC
        ROS_INFO("min_time is:%f", min_time);


        // TODO: publish drive/brake message
        ackermann_msgs::AckermannDriveStamped publish_speed;
        std_msgs::Bool brake_message;
        if(min_time < threshold_time) //about to crash
        {
            publish_speed.drive.speed = 0.0;
            brake_message.data = true;
        }
        else
        {
    /*        publish_speed.drive.speed = speed;
            brake_message.data = true;*/
        }

        pub_brake_tool.publish(brake_message);
        pub_brake.publish(publish_speed);


    }

};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "baihong_zeng_safety");
    Safety sn;
    ros::spin();
    return 0;
}