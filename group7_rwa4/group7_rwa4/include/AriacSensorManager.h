//
// Created by zeid on 2/27/20.
//

#pragma once

#include <unordered_map>
#include <string>
#include <cmath>
#include <set>
#include <deque>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
#include <tf2_ros/buffer.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <osrf_gear/Proximity.h>
#include <osrf_gear/Order.h>
#include "RobotController.h"


class AriacSensorManager {
private:
    ros::NodeHandle sensor_nh_;

    // Subscriber
    ros::Subscriber lc_belt_sub; // Subscribe to the '/ariac/lc_belt' topic
    ros::Subscriber bb_1_sub; // Subscribe to the '/ariac/break_beam_1_change' topic
    ros::Subscriber bb_2_sub; // Subscribe to the '/ariac/break_beam_1_change' topic
    ros::Subscriber orders_sub; 
    ros::Subscriber lc_gear_sub; // Subscribe to the '/ariac/lc_gear' topic
    ros::Subscriber qc_1_sub; // Subscribe to the '/ariac/quality_control_sensor_2' topic
    bool qc_1_redFlag;
    ros::Subscriber qc_2_sub; // Subscribe to the '/ariac/quality_control_sensor_2' topic
    bool qc_2_redFlag;

    // Order/Product/Pose containers
    std::vector<osrf_gear::Order> received_orders_;
    unsigned int order_number;
    std::unordered_map<std::string, geometry_msgs::Pose> belt_part_map; // map for part on the belt
    std::unordered_map<std::string, geometry_msgs::Pose> gear_bin_map; // map for part in the bin
    std::deque<std::pair<std::string, std::string>> part_q; // the queue store (part_type, part_frame_name)
    std::unordered_map<std::string, unsigned int> part_counter; // map which calculate # of part_type
    std::multiset<std::string> desired_parts; // mulitset for desired parts in current order

    // RobotController arm1;
    RobotController arm2;

public:
    AriacSensorManager();
    ~AriacSensorManager();
    void order_callback(const osrf_gear::Order::ConstPtr &);
    void lc_belt_callback(const osrf_gear::LogicalCameraImage::ConstPtr &);
    void bb_1_callback(const osrf_gear::Proximity::ConstPtr &);
    void bb_2_callback(const osrf_gear::Proximity::ConstPtr &);
    void lc_gear_callback(const osrf_gear::LogicalCameraImage::ConstPtr &);
    void qc_1_callback(const osrf_gear::LogicalCameraImage::ConstPtr &);
    void qc_2_callback(const osrf_gear::LogicalCameraImage::ConstPtr &);



    // geometry_msgs::Pose GetPartPose(const std::string& src_frame,
    //                                 const std::string& target_frame);
    std::deque<std::pair<std::string, std::string>> get_part_q() {
        return part_q;
    }
    void setDesiredParts();

};

