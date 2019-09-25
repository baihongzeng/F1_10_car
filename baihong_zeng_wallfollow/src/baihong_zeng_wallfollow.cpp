#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Bool.h>
#include <vector>
// TODO: include ROS msg type headers and libraries

const float pi = 3.1415926;
class Laserbeam{
public:
    int index;
    float range;
    float degree;
};

class Wallfollow {
// The class that handles wall following
private:
    float kp = 30;
    float ki = 0;
    float kd = 0.3;

    float prev_error = 0.0;
    float error = 0.0;
    float integral = 0.0;

    float desired_dist_to_left_wall = 1.65/2;//the width of corridor is about 1.65m
    // TODO: create ROS subscribers and publishers
    ros::NodeHandle n;
    ros::Subscriber sub_scan;
    ros::Publisher pub_drive;

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

    float getRange(const sensor_msgs::LaserScan::ConstPtr &scan_msg, float degree){
        int scan_resolution = scan_msg->ranges.size(); //1080 in this case
        int index = (int)(degree - (-180))*(scan_resolution/360)-1;
        float range = scan_msg->ranges[index];

        return range;
    }

    int pidControl(float & error)
    {

    int steering_angle;
    integral = integral + error;
    steering_angle = - (kp * error + ki * integral + kd * (error - prev_error));
    prev_error = error;

    return steering_angle;
    }

    float getSpeed(float & steering_angle) // change the speed according to the steering angle
    {
        float speed;
        if (fabs(steering_angle) <= 10){
            speed = 1.5;
        }
        else if (fabs(steering_angle) <= 20 && fabs(steering_angle) > 10){
            speed = 1;
        }
        else{
            speed = 0.5;
        }

        return speed;
    }

public:
    Wallfollow() {
        // TODO: create ROS subscribers and publishers
        sub_scan = n.subscribe("scan", 10000, &Wallfollow::scan_callback, this);
        pub_drive = n.advertise<ackermann_msgs::AckermannDriveStamped>("nav", 1000);//first argument is the topic name you want to use for publish
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
        // TODO: calculate
        // choose two laser beam a and b, with b taken at 90 degrees and a taken at an angle theta ( 90 < theta < 160)

        Laserbeam a;
        Laserbeam b;
        a.degree = 20;//follow left wall
        b.degree = 90;
        a.range = getRange(scan_msg, a.degree);
        b.range = getRange(scan_msg, b.degree);

        //use the distances a and b to calculate the angle alpha between the car's x-axis and the right wall
        float theta_rad = (a.degree - b.degree)*pi/180; //trinogeometry takes radian inputs
        float alpha_rad = atan((a.range*cos(theta_rad) - b.range) / (a.range*sin(theta_rad)));
        float alpha_deg = alpha_rad *180 / pi;
        //ROS_INFO("alpha: %f",alpha_deg);

        //use alpha to find the current distance Dt to the car, adn then alpha and Dt to find the estimated future distance Dt+1 to the wall
        float Dt = b.range * cos(alpha_rad);
        float L = 0.1; //unit:m. the lookahead distance
        float Dt1 = Dt + L * sin(alpha_rad);
        error = desired_dist_to_left_wall - Dt1;

/*        if (abs(scan_msg->ranges[270] - 3.4/2) < 0.1 && abs(b.range - 3.4/2) < 0.1){
            error = 0;
        }*/

       // ROS_INFO("error: %f",error);
        //run Dt1 through the PID algorithm to get a steering angle
        ackermann_msgs::AckermannDriveStamped publish_drive;

        publish_drive.header.frame_id = "laser";
        publish_drive.header.stamp = ros::Time::now();

        publish_drive.drive.steering_angle = pidControl(error);
        publish_drive.drive.speed = getSpeed(publish_drive.drive.steering_angle);
      //  ROS_INFO("speed: %f",publish_drive.drive.speed);
        pub_drive.publish(publish_drive);
    }

};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "baihong_zeng_wallfollow");
    Wallfollow wallfollow;
    ros::spin();
    return 0;
}