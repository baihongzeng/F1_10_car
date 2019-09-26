//
// Created by baihong on 9/20/19.
//

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Bool.h>
#include <vector>

// TODO: include ROS msg type headers and libraries

const float pi = 3.1415926;
struct point
{
    int index;
    float value;
};

struct gap
{
    int gap_start_index;
    int gap_size;
};

class Reactive_follow_gap {
// The class that handles gap following by using reactive methods
private:

    // TODO: create ROS subscribers and publishers
    ros::NodeHandle n;
    ros::Subscriber sub_scan;
    ros::Publisher pub_drive;
    point nearestPoint; //initialize the nearest point
    float bubble_radius = 0.4; //the bubble radius around the nearest point
    float max_length_thres_t = 1.0;//gap distance threshold
    float scan_lower_bound = -90;
    float scan_upper_bound = 90;
    float precaution_degree;

    std::vector<std::pair<float, float>> preprocess_lidar_range(std::vector<float> & array, float range_max, float angle_inc, float angle_min) {

        std::vector<std::pair<float, float>> new_scan;//store pair in a vector, first:angle in degree; second:range

        for (int i = (scan_lower_bound - (-180)) * 3 - 1; i < (scan_upper_bound - (-180)) * 3 - 1; i++) {
            //set inf as range_max
            if (std::isinf(array[i])) {
                new_scan.push_back(std::make_pair(180/pi*(angle_min + angle_inc*i), range_max));
            }
            //discard nan
            else if (std::isnan(array[i])) {
                continue;
            }
            //keep every other ranges the same
            else{
                new_scan.push_back(std::make_pair(180/pi*(angle_min + angle_inc*i), array[i]));
            }


        }
        //initialize the nearest_point
        nearestPoint.index = 0;// the index of nearest point in the "new_scan" array
        nearestPoint.value = new_scan[0].second;
        for(int i = 0; i < new_scan.size(); i++){
            if (new_scan[i].second < nearestPoint.value) {//find nearest_point
                nearestPoint.value = new_scan[i].second;
                nearestPoint.index = i;
            }
        }

        ROS_INFO("nearestpoint_value: %f, %d",nearestPoint.value, nearestPoint.index);
        return new_scan;
    }

    void setAroundBubbleZero(std::vector<std::pair<float, float>> & array){
        int left_zero_index;
        int right_zero_index;

        float left_diff = FLT_MAX;
        float right_diff = FLT_MAX;
        precaution_degree = 180 / pi * atan(bubble_radius / nearestPoint.value); //the angle of half the circular sector
        //find left / right zero index
        if(array[nearestPoint.index].first - precaution_degree < scan_lower_bound){
            left_zero_index = 0;
        }
        else {
            for(int i = 0; i < array.size(); i++){
                if (fabs(array[nearestPoint.index].first - precaution_degree - array[i].first) < left_diff){
                    left_zero_index = i;
                    left_diff = array[nearestPoint.index].first - precaution_degree - array[i].first;
                }
            }
        }

        if(array[nearestPoint.index].first + precaution_degree > scan_upper_bound){
            right_zero_index = array.size()-1;
        }
        else {
            for(int i = 0; i < array.size(); i++){
                if (fabs(array[nearestPoint.index].first + precaution_degree - array[i].first) < right_diff){
                    right_zero_index = i;
                    right_diff = array[nearestPoint.index].first + precaution_degree - array[i].first;
                }
            }
        }
        //have all elements within left / right index as zero
        for (int i = left_zero_index; i <= right_zero_index; i++){
            if (array[i].second < (nearestPoint.value + bubble_radius)){
                array[i].second = 0.0;
            }
        }
        ROS_INFO("left, right index: %d, %d",left_zero_index, right_zero_index);
    }


    gap findMaxLengthGap(std::vector<std::pair<float, float>> & array){
        gap temp_gap;
        temp_gap.gap_size = 0;
        temp_gap.gap_start_index = 0;

        gap largest_gap;
        largest_gap.gap_size = 0;
        largest_gap.gap_start_index = 0;

        for(int i = 0; i < (int)array.size(); i++){

            if(array[i].second > max_length_thres_t){ //value larger than threshold is considered as gap

                if(temp_gap.gap_size == 0){
                    temp_gap.gap_start_index = i;
                }
                temp_gap.gap_size++;
                if(temp_gap.gap_size >= largest_gap.gap_size){
                    largest_gap.gap_start_index = temp_gap.gap_start_index;
                    largest_gap.gap_size = temp_gap.gap_size;
                }
            }
            else{
                array[i].second = 0;
                temp_gap.gap_size = 0;
            }

        }
/*        ROS_INFO("largest_gap_start_index: %d",largest_gap.gap_start_index);
        ROS_INFO("largest_gap_size: %d",largest_gap.gap_size);*/
        return largest_gap;
    }

    //set the elements close to the zeros as zeros as well, leave enough margin to account the car width
    void disparity(std::vector<std::pair<float, float>> & array, gap largest_gap) {

        for (int i = largest_gap.gap_start_index; i < (largest_gap.gap_start_index + largest_gap.gap_size); i++){
            if (array[i+1].second - array[i].second > 0.5) {
                int k = i;
                while(i < k + 30 && i < (largest_gap.gap_start_index + largest_gap.gap_size)){
                    array[i+1].second = 0;
                    i++;
                }
                break;
            }

            if (array[i].second - array[i+1].second > 0.5) {
                int k = i;
                while(i > k - 30 && i >= largest_gap.gap_start_index){
                    array[i-1].second = 0;
                    i--;
                }
                break;

            }
        }

    }

    //choose the middle point of the MaxLengthGap as the furtherest point
    int findMidInMaxGap(std::vector<std::pair<float, float>> & array, gap largest_gap){
        point furtherest_point;
        furtherest_point.value = 0;


        furtherest_point.index = (int) (largest_gap.gap_start_index + largest_gap.gap_size / 2);
        ROS_INFO("furtherest point: %d",furtherest_point.index);
        return furtherest_point.index;
    }

public:
    Reactive_follow_gap() {
        // TODO: create ROS subscribers and publishers
        sub_scan = n.subscribe("scan", 10000, &Reactive_follow_gap::scan_callback, this);
        pub_drive = n.advertise<ackermann_msgs::AckermannDriveStamped>("nav", 1000);//first argument is the topic name you want to use for publish
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
        // preprocess the lidar range info like inf and nan
        std::vector<float> scan_array = scan_msg->ranges;
        std::vector<std::pair<float, float>> processed_range = preprocess_lidar_range(scan_array, scan_msg->range_max, scan_msg->angle_increment,scan_msg->angle_min);
        gap largest_gap = findMaxLengthGap(processed_range);
        disparity(processed_range, largest_gap);
        int furtherest_point = findMidInMaxGap(processed_range, largest_gap);

        //ROS_INFO("max_index: %d",furtherest_point);
        float steering_angle = processed_range[furtherest_point].first;
        //regulate the steering angle to decrease the phenomenon of wobbly
        if(fabs(steering_angle) < 7){
            steering_angle = 0;
        }
        ackermann_msgs::AckermannDriveStamped publish_drive;
        publish_drive.drive.steering_angle = processed_range[furtherest_point].first;

        //regulate the speed according to the steering angle
        float speed = 3;
        if (fabs(steering_angle) <= 10){
            speed = 3;
        }
        else if (fabs(steering_angle) <= 20 && fabs(steering_angle) > 10){
            speed = 2;
        }
        else{
            speed = 1.5;
        }

        publish_drive.drive.speed = speed;

        pub_drive.publish(publish_drive);
    }

};

int main(int argc, char ** argv) {

    ros::init(argc, argv, "reactive_method");//the third parameter is the node name
    Reactive_follow_gap reactiveFollowGap;
    ros::spin();
    return 0;
}