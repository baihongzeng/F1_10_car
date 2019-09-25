#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <sensor_msgs/LaserScan.h>
#include "scan_range.h"


class PubAndSub
{
private:
    float smallest_value;
    float largest_value;
    std::vector<float> msg_range;
    std::vector<float> valid_range;
    std_msgs::Float64 close_msg;
    std_msgs::Float64 farth_msg;


    void sortOrder(std::vector<float> & array)
    {
        std::sort(array.begin(), array.end());
    }

    void invalidNumberFilter(std::vector<float> & array, int size)
    {
        std::vector<float>::iterator it;
        for(it = array.begin(); it != array.end(); ++it)
        {
            if (std::isinf(*it) || std::isnan(*it))
            {
                array.erase(it);
            }
        }
    }

    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub_close;
    ros::Publisher pub_farth;
    ros::Publisher pub_scan;
public:
    PubAndSub()
    {
        sub = n.subscribe("scan", 1000, &PubAndSub::lidarCallback, this);//first argument is the subscribed topic;
        pub_close = n.advertise<std_msgs::Float64>("closest_point", 1000);//first argument is the topic name you want to use for publish
        pub_farth = n.advertise<std_msgs::Float64>("farthest_point", 1000);//first argument is the topic name you want to use for publish
        pub_scan = n.advertise<baihong_zeng_roslab::scan_range>("scan_range", 1000);//first argument is the topic name you want to use for publish
    }


    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr & msg)
    {
        msg_range = msg->ranges;
        invalidNumberFilter(msg_range, msg_range.size());
        sortOrder(msg_range); // sort the array of msg_range

        smallest_value = msg_range[0];
        largest_value = msg_range[msg_range.size()-1];


        close_msg.data = smallest_value;
        farth_msg.data = largest_value;
        pub_close.publish(close_msg);
        pub_farth.publish(farth_msg);


        baihong_zeng_roslab::scan_range new_scan_range;
        new_scan_range.minimum_value = smallest_value;
        new_scan_range.maximum_value = largest_value;
        pub_scan.publish(new_scan_range);

        //ROS_INFO("the minimum value is: %f", close_msg.data);
        //ROS_INFO("the maximum value is: %f", farth_msg.data);

    }

};


int main(int argc, char ** argv)
{

    ros::init(argc, argv, "lidar_processing"); //third argument is the name of the node

    PubAndSub publisher_and_subscriber;
    ros::spin();

    return 0;
}