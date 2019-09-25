//
// Created by baihong on 9/20/19.
//

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Bool.h>
#include <vector>
#include "reactive_gap_follow.h"
#include <queue>

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
    float bubble_radius = 0.3; //the bubble radius around the nearest point
    //int length_sequence_n = 21; //must have more than n consective points with range larger than can be considered as max gap
    float max_length_thres_t = 1.5;//gap distance threshold
    float scan_lower_bound = -75;
    float scan_upper_bound = 75;
    float precaution_degree;

    std::vector<std::pair<float, float>> preprocess_lidar_range(std::vector<float> & array, float range_max, float angle_inc, float angle_min) {

        std::vector<std::pair<float, float>> new_scan;//store pair in a vector, first:angle in degree; second:range

        for (int i = (scan_lower_bound - (-180)) * 3 - 1; i < (scan_upper_bound - (-180)) * 3 - 1; i++) {
            //set inf as range_max
            if (std::isinf(array[i])) {
                new_scan.push_back(std::make_pair(180/pi*(angle_min + angle_inc*i), 3));
            }
            //discard nan
            else if (std::isnan(array[i])) {
                continue;
            }
            else if (array[i] >= 3){
                //continue; //删掉3m之外的数据
                new_scan.push_back(std::make_pair(180/pi*(angle_min + angle_inc*i),array[i]));
            }
            else{
                new_scan.push_back(std::make_pair(180/pi*(angle_min + angle_inc*i), array[i]));
            }


        }
        nearestPoint.index = 0;
        nearestPoint.value = new_scan[0].second; //initialize the nearest_point value
        for(int i = 0; i < new_scan.size(); i++){
            if (new_scan[i].second < nearestPoint.value) {//find nearest_point
                nearestPoint.value = new_scan[i].second;
                nearestPoint.index = i;
            }
        }
       // ROS_INFO("nearestpoint_value: %f, %d",nearestPoint.value, nearestPoint.index);
        return new_scan;
    }

    void setAroundBubbleZero(std::vector<std::pair<float, float>> & array){
        int left_zero_index;
        int right_zero_index;

        float left_diff = FLT_MAX;
        float right_diff = FLT_MAX;
        precaution_degree = 180 / pi * atan(bubble_radius / nearestPoint.value); //0.3 is half the width of car

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

/*        for (int i = 0; i < (int) array.size() - 1; i++) {

            if (array[nearestPoint.index].first - precaution_degree - array[i].first > 0) {
                left_zero_index = i;
            }
        }
        for (int i = 0; i < (int) array.size() - 1; i++) {
            if (array[nearestPoint.index].first + precaution_degree - array[i].first < 0) {
                right_zero_index = i;
            }
        }*/

        for (int i = left_zero_index; i <= right_zero_index; i++){//后期应该改成圆，而不是现在的扇形
            if (array[i].second < (nearestPoint.value + bubble_radius)){
                array[i].second = 0.0;
            }
        }
        ROS_INFO("left, right index: %d, %d",left_zero_index, right_zero_index);
    }

/*    void averageValues(std::vector<std::pair<float, float>> & array){
        int kernel_size = 5;
        for (int i = 0; i < (int) array.size()-1;) {
            if(array[i].second !=0 && array[i+1].second !=0 && array[i+2].second !=0 && array[i+3].second !=0 && array[i+4].second !=0){
                array[i].second = (array[i].second + array[i+1].second + array[i+2].second + array[i+3].second + array[i+4].second) / kernel_size;
                array[i+1].second = array[i].second;
                array[i+2].second = array[i].second;
                array[i+3].second = array[i].second;
                array[i+4].second = array[i].second;

                i = i + kernel_size;
            }
            else{
                i++;
            }
        }
    }*/

    gap findMaxLengthGap(std::vector<std::pair<float, float>> & array){
        gap temp_gap;
        temp_gap.gap_size = 0;
        temp_gap.gap_start_index = 0;

        gap largest_gap;
        largest_gap.gap_size = 0;
        largest_gap.gap_start_index = 0;

        for(int i = 0; i < (int)array.size(); i++){

            if(array[i].second > max_length_thres_t){

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
                temp_gap.gap_size = 0;
            }

        }

/*        ROS_INFO("largest_gap_start_index: %d",largest_gap.gap_start_index);
        ROS_INFO("largest_gap_size: %d",largest_gap.gap_size);*/
        return largest_gap;
    }

    void disparity(std::vector<std::pair<float, float>> & array, gap largest_gap) {

        int direction_index;
        for (int i = largest_gap.gap_start_index; i < (largest_gap.gap_start_index + largest_gap.gap_size); i++){
            if (array[i+1].second - array[i].second > 0.5) {
                int k = i;
                while(i < k + 30 && i < (largest_gap.gap_start_index + largest_gap.gap_size)){
                    array[i+1].second = array[k].second;
                    i++;
                }
                break;
/*                //disparity happens
                precaution_degree = 180 / pi * atan(0.5 / array[i].second); //0.3 is half the width of car
                int j = i;
                int m = j;
                while(array[m+1].first < array[j].first + precaution_degree && (m+1) < (array.size()-1)){
                    array[m+1].second = array[j].second;
                    m++;
                }
                i = m;*/

            }

            if (array[i].second - array[i+1].second > 0.5) {
                int k = i;
                while(i > k - 30 && i >= largest_gap.gap_start_index){
                    array[i-1].second = array[k].second;
                    i--;
                }
                break;
                i = k;
/*                //disparity happens
                precaution_degree = 180 / pi * atan(0.6 / array[i+1].second); //0.3 is half the width of car
                int j = i;
                int m = j;
                while(array[m-1].first < array[j].first - precaution_degree && (m-1) >= 0){
                    array[m-1].second = array[j].second;
                    m--;
                }
                i = m;*/
            }
        }

    }


    int kernel(std::vector<std::pair<float, float>> & array, gap largest_gap){
        int kernel_size = 60;
        int a = kernel_size - 1;

        std::vector<float> window;

        if(largest_gap.gap_size < kernel_size){
           kernel_size = largest_gap.gap_size;
           a = kernel_size - 1;
        }
        int front = a / 2;
        int rear = a - front;
        //create a new array called "window"
        for(int i = 0; i < front; i++){
            window.push_back(0);
        }
        for(int i = largest_gap.gap_start_index; i < largest_gap.gap_start_index+kernel_size; i++){
            window.push_back(array[i].second);
        }
        for(int i = 0; i < rear; i++){
            window.push_back(0);
        }

        float sum = std::accumulate(window.begin(), window.begin()+kernel_size, 0);
        float window_average = sum / window.size();
        array[0].second = window_average;
        //int direction_index = largest_gap.gap_start_index + kernel_size / 2;

        for(int i = largest_gap.gap_start_index; i < largest_gap.gap_start_index + largest_gap.gap_size - kernel_size; i++){
            window.erase(window.begin());
            window.push_back(array[i+kernel_size].second);
            sum = std::accumulate(window.begin(), window.end(), 0);
            if ((sum / window.size()) >= window_average) {
                window_average = sum / window.size();
                direction_index = i + kernel_size / 2;
            }

        }

        return direction_index;
    }

    int findMaxInMaxGap(std::vector<std::pair<float, float>> & array, gap largest_gap){
        point furtherest_point;
        furtherest_point.value = 0;
        //furtherest_point.index = largest_gap.gap_start_index + largest_gap.gap_size / 2;
        for(int i = largest_gap.gap_start_index; i < (largest_gap.gap_start_index + largest_gap.gap_size); i++){
            if (array[i].second >= furtherest_point.value){
                furtherest_point.value = array[i].second;
                furtherest_point.index = i;
            }
        }


/*        if (furtherest_point.index +15 >= largest_gap.gap_start_index+largest_gap.gap_size){
            furtherest_point.index = largest_gap.gap_start_index+largest_gap.gap_size - 15;
        }
        else if (furtherest_point.index - 15 <= largest_gap.gap_start_index){
            furtherest_point.index = largest_gap.gap_start_index + 15;
        }*/
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
        //std::vector<float> cutted_array (scan_msg->ranges.begin()+90*3-1, scan_msg->ranges.begin()+270*3-1); // have the reading only from -90 to 90 degree
        //float angle_min = scan_msg->angle_min + scan_msg->angle_increment * 3*90;
        std::vector<float> scan_array = scan_msg->ranges;
        std::vector<std::pair<float, float>> processed_range = preprocess_lidar_range(scan_array, scan_msg->range_max, scan_msg->angle_increment,scan_msg->angle_min);
        setAroundBubbleZero(processed_range);
        //(processed_range);
        gap largest_gap = findMaxLengthGap(processed_range);
        //int direction = kernel(processed_range, largest_gap);

        disparity(processed_range, largest_gap);
        int furtherest_point = findMaxInMaxGap(processed_range, largest_gap);
        //ROS_INFO("max_index: %d",furtherest_point.index);

        ackermann_msgs::AckermannDriveStamped publish_drive;

        publish_drive.drive.steering_angle = processed_range[furtherest_point].first;
        publish_drive.drive.speed = 1;

        pub_drive.publish(publish_drive);
    }

};

int main(int argc, char ** argv) {

    ros::init(argc, argv, "reactive_gap_follow");
    Reactive_follow_gap reactiveFollowGap;
    ros::spin();
    return 0;
}