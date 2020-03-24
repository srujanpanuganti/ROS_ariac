#include "AriacSensorManager.h"
using namespace std;

AriacSensorManager::AriacSensorManager() :
    part_list {},
    arm1 {"arm1"},
    arm2 {"arm2"}
{
    orders_sub = sensor_nh_.subscribe("/ariac/orders", 10, 
        & AriacSensorManager::order_callback, this);

    bb_1_sub = sensor_nh_.subscribe("/ariac/break_beam_1_change", 10,
        & AriacSensorManager::bb_1_callback, this);

    bb_2_sub = sensor_nh_.subscribe("/ariac/break_beam_2_change", 10,
        & AriacSensorManager::bb_2_callback, this);

    lc_gear_sub = sensor_nh_.subscribe("/ariac/lc_gear", 10, 
        & AriacSensorManager::lc_gear_callback, this);

    qc_1_sub = sensor_nh_.subscribe("/ariac/quality_control_sensor_1", 1, 
        & AriacSensorManager::qc_1_callback, this);

    order_number = 0;

    // looping by numbers of robot
    // for (size_t i = 1; i <= num_arms; ++i) 
    // {
    //     // name robot
    //     string arm_name = "arm" + to_string(i);
    //     // construct a robot by its name
    //     RobotController robot{arm_name};
    //     // push the address of the robot to the constant robots pointer 
    //     robots.push_back(&robot);
    // }
    qc_1_redFlag = false;
    qc_2_redFlag = false;
}

AriacSensorManager::~AriacSensorManager() {}

void AriacSensorManager::order_callback(const osrf_gear::Order::ConstPtr & order_msg) {
    ROS_INFO_STREAM("Received order:\n" << *order_msg);
    received_orders_.push_back(*order_msg);
    setDesiredParts();
}


void AriacSensorManager::lc_belt_callback(
    const osrf_gear::LogicalCameraImage::ConstPtr& image_msg)
{
    ros::AsyncSpinner spinner(0);
    spinner.start();
    if (image_msg->models.size() == 0){
        ROS_WARN_THROTTLE(5, "---Logical camera 1 does not detect things---");
        return;
    }
    ROS_INFO_STREAM("Logical camera captures '" << image_msg->models.size() << "' objects.");

    ros::Duration timeout(0.2);
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    for (auto & msg : image_msg->models){
       
        geometry_msgs::TransformStamped transformStamped;

        // find if this type of part has passed by before
        if (part_counter.find(msg.type) == part_counter.end())
            part_counter.insert(make_pair(msg.type, 0));
        else
            part_counter[msg.type]++;
        
        string camera_frame = "lc_belt_" + msg.type + "_" + to_string(part_counter[msg.type]) + "_frame";
        ROS_INFO_STREAM("The frame is named: " << camera_frame);
        try{
            transformStamped = tfBuffer.lookupTransform("world", camera_frame, ros::Time(0), timeout);
            std::pair<std::string, std::string> part_pair {msg.type, camera_frame};
            part_list.push_back(part_pair);
            geometry_msgs::Pose part_pose; 
            part_pose.position.x = transformStamped.transform.translation.x;
            // part_pose.position.y = transformStamped.transform.translation.y;
            part_pose.position.y = 0.9;
            part_pose.position.z = transformStamped.transform.translation.z;
            part_pose.orientation.x = transformStamped.transform.rotation.x;
            part_pose.orientation.y = transformStamped.transform.rotation.y;
            part_pose.orientation.z = transformStamped.transform.rotation.z;
            part_pose.orientation.w = transformStamped.transform.rotation.w;
            // part_pose_list[camera_frame].emplace_back(part_pose); // should make pair and insert instead emplace back
            part_pose_list.insert({camera_frame, part_pose});
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s\n",ex.what());
            part_counter[msg.type]--;
        }
        break;
    }
    lc_belt_sub.shutdown();
}


void AriacSensorManager::lc_gear_callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg)
{
    ros::AsyncSpinner spinner(0);
    spinner.start();
    if (image_msg->models.size() == 0){
        ROS_WARN_THROTTLE(5, "---lc_gear does not detect things---");
        return;
    }
    ROS_INFO_STREAM_THROTTLE(5, "lc_gear captures '" << image_msg->models.size() << "' gears.");

    unordered_map<string, double> arm2_joint_home_pose;
    arm2_joint_home_pose["linear_arm_actuator_joint"] = -0.3;
    arm2_joint_home_pose["shoulder_pan_joint"] = 2;
    arm2_joint_home_pose["shoulder_lift_joint"] = 0;
    arm2_joint_home_pose["elbow_joint"] = 0;
    arm2_joint_home_pose["wrist_1_joint"] = -3.14/2;
    arm2_joint_home_pose["wrist_2_joint"] = -3.14/2;
    arm2_joint_home_pose["wrist_3_joint"] = 0;

    
    ros::Duration timeout(0.2);
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    size_t gear_counter = 1;
    for (auto & msg : image_msg->models){
        geometry_msgs::TransformStamped transformStamped;
        string camera_frame = "lc_gear_" + msg.type + "_" + to_string(gear_counter) + "_frame";
        ROS_INFO_STREAM("The frame is named: " << camera_frame);
        try{
            transformStamped = tfBuffer.lookupTransform("world", camera_frame, ros::Time(0), timeout);
            geometry_msgs::Pose part_pose; 
            part_pose.position.x = transformStamped.transform.translation.x;
            part_pose.position.y = transformStamped.transform.translation.y;
            part_pose.position.z = transformStamped.transform.translation.z;
            part_pose.orientation.x = transformStamped.transform.rotation.x;
            part_pose.orientation.y = transformStamped.transform.rotation.y;
            part_pose.orientation.z = transformStamped.transform.rotation.z;
            part_pose.orientation.w = transformStamped.transform.rotation.w;
            // pick it up and check the quality
            // robots[1]->SendRobotTo(arm2_joint_home_pose);
            // bool if_pick = robots[1]->PickPart(part_pose);
            // if (if_pick){
            //     // sent robot to quality cam at agv2

            //     // sbuscirbe to qc_2
            //     qc_2_sub = sensor_nh_.subscribe("/ariac/quality_control_sensor_2", 1, 
            //         & AriacSensorManager::qc_2_callback, this);
            //     if (qc_2_redFlag){
            //         // throw the part
            //     }
            //     else {
            //         // put it back;
            //         gear_bin_map.insert({camera_frame, part_pose});
            //     }
            //     qc_2_sub.shutdown(); // unsbuscirbe to qc_2
                
            // }
            ++gear_counter;
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s\n",ex.what());
        }
    }
    lc_gear_sub.shutdown();
}




void AriacSensorManager::qc_1_callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg)
{
    if (image_msg->models.size() == 0)
        qc_1_redFlag = false;
    else
        qc_1_redFlag = true;
}


void AriacSensorManager::qc_2_callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg)
{
    if (image_msg->models.size() == 0)
        qc_2_redFlag = false;
    else
        qc_2_redFlag = true;
}

void AriacSensorManager::bb_1_callback(const osrf_gear::Proximity::ConstPtr & msg) {
    /*
    Callback function working process:
    1. when a part fully goes through braek beam sensor, start counting.
    2. Once another part passes, check if the duration between this time and the next time 
        the beam be triggered is less than 1 sec:
            yes: consider the duration 
            no: keep counting
    */
    ros::AsyncSpinner spinner(0);
    spinner.start();
    if (msg->object_detected){
        lc_belt_sub = sensor_nh_.subscribe("/ariac/lc_belt", 10, 
            & AriacSensorManager::lc_belt_callback, this);
    }
}

void AriacSensorManager::bb_2_callback(const osrf_gear::Proximity::ConstPtr & msg) {
    ros::AsyncSpinner spinner(0);
    spinner.start();
    if (msg->object_detected){
        ROS_INFO_STREAM("Break beam triggered. The part is '" << part_list.front().first << "'");
        auto element_itr = desired_parts.find(part_list.front().first);
        if (desired_parts.find(part_list.front().first) != desired_parts.end()){
            ROS_INFO_STREAM("!!!!!!Pick this part!!!!");
            auto part_frame = part_pose_list[part_list.front().second];
            ROS_INFO_STREAM("The desired part pose is\n" << part_frame);
            bool if_pick = arm1.PickPart(part_frame);
            if (if_pick)
                desired_parts.erase(element_itr);
        }
        else
            ROS_INFO_STREAM("Let it go ~~~~~~~~~");      
        part_list.pop_front();
    }
}


// void PickBaseOnOrder(){
//     part_list = sensor.get_part_list();
    

//     for (const auto & order : received_orders_){
//         auto order_id = order.order_id;
//         auto shipments = order.shipments;
//         for (const auto & shipment : shipments){
//             auto shipment_type = shipment.shipment_type;
//             auto products = shipment.products;
//             ROS_INFO_STREAM("Order ID: " << order_id);
//             ROS_INFO_STREAM("Shipment Type: " << shipment_type);
//             for (const auto & product: products){
//                 ros::spinOnce();
//                 product_type_pose_.first = product.type;
//                 //ROS_INFO_STREAM("Product type: " << product_type_pose_.first);
//                 product_type_pose_.second = product.pose;
//                 ROS_INFO_STREAM("Product pose: " << product_type_pose_.second.position.x);
//                 pick_n_place_success =  PickAndPlace(product_type_pose_);
//                 //--todo: What do we do if pick and place fails?
//             }
//             int finish=1;
//         }
//  }

void AriacSensorManager::setDesiredParts(){
    ROS_INFO_STREAM(">>>>>>>>> Setting desired parts");
    auto current_order = received_orders_[order_number];
    auto order_id = current_order.order_id;
    auto shipments = current_order.shipments;
    for (const auto &shipment: shipments){
        auto shipment_type = shipment.shipment_type;
        auto products = shipment.products;
        ROS_INFO_STREAM("Order ID: " << order_id);
        ROS_INFO_STREAM("Shipment Type: " << shipment_type);
        for (const auto &product: products)
            desired_parts.insert(product.type);
    }
    ROS_INFO_STREAM(">>>>>>>> The current desired_parts are:");
    for (const auto & part : desired_parts)
        std::cout << part << std::endl;
    if (!order_id.empty())
        ++order_number;
}