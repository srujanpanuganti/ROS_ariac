//
// Created by zeid on 2/27/20.
//
#include "RobotController.h"

/**
 * Constructor for the robot
 * Class attributes are initialized in the constructor init list
 * You can instantiate another robot by passing the correct parameter to the constructor
 */
RobotController::RobotController(std::string arm_id) :
    robot_controller_nh_("/ariac/"+arm_id),
    robot_controller_options("manipulator",
            "/ariac/"+arm_id+"/robot_description",
            robot_controller_nh_),
    robot_move_group_(robot_controller_options),
    armSpinner {0}
{
    ROS_WARN(">>>>> RobotController");

    robot_move_group_.setPlanningTime(50);
    robot_move_group_.setNumPlanningAttempts(10);
    robot_move_group_.setPlannerId("RRTConnectkConfigDefault");
    robot_move_group_.setMaxVelocityScalingFactor(1);
    robot_move_group_.setMaxAccelerationScalingFactor(1);
    // robot_move_group_.setEndEffector("moveit_ee");
    robot_move_group_.allowReplanning(true);

    

    ////////////modified
    //--These are joint positions used for the home position
    // home_joint_pose_ = {0.0, 3.1, -1.1, 1.9, 3.9, 4.7, 0};

    if (arm_id == "arm1"){
        home_joint_pose_1["linear_arm_actuator_joint"] = -0.19;
        home_joint_pose_1["shoulder_pan_joint"] = 0;
        home_joint_pose_1["shoulder_lift_joint"] = -0.8;
        home_joint_pose_1["elbow_joint"] = 1.6;
        home_joint_pose_1["wrist_1_joint"] = 3.9;
        home_joint_pose_1["wrist_2_joint"] = 4.7;
        home_joint_pose_1["wrist_3_joint"] = 0;
        SendRobotTo(home_joint_pose_1);
    }
    else{
        // map<string, double> arm2_joint_home_pose;
        home_joint_pose_1["linear_arm_actuator_joint"] = -0.56;
        home_joint_pose_1["shoulder_pan_joint"] = 2;
        home_joint_pose_1["shoulder_lift_joint"] = 0;
        home_joint_pose_1["elbow_joint"] = 0;
        home_joint_pose_1["wrist_1_joint"] = -3.14/2;
        home_joint_pose_1["wrist_2_joint"] = -3.14/2;
        home_joint_pose_1["wrist_3_joint"] = 0;
        for (const auto & mapItem: home_joint_pose_1)
            SendRobotTo(mapItem.first, mapItem.second);
    }


    //-- offset used for picking up parts
    //-- For the pulley_part, the offset is different since the pulley is thicker
    offset_ = 0.03;

    //--topic used to get the status of the gripper
    gripper_subscriber_ = gripper_nh_.subscribe("/ariac/" + arm_id + "/gripper/state", 1000, 
            & RobotController::GripperCallback, this);

    /////////////modified
    // SendRobotTo();
    // SendRobotTo(home_joint_pose_0);


    robot_tf_listener_.waitForTransform(arm_id + "_linear_arm_actuator", arm_id + "_ee_link",
        ros::Time(0), ros::Duration(10));
    robot_tf_listener_.lookupTransform("/" + arm_id + "_linear_arm_actuator", 
        "/" + arm_id + "_ee_link", ros::Time(0), robot_tf_transform_);


    fixed_orientation_.x = robot_tf_transform_.getRotation().x();
    fixed_orientation_.y = robot_tf_transform_.getRotation().y();
    fixed_orientation_.z = robot_tf_transform_.getRotation().z();
    fixed_orientation_.w = robot_tf_transform_.getRotation().w();

    tf::quaternionMsgToTF(fixed_orientation_,q);
    tf::Matrix3x3(q).getRPY(roll_def_,pitch_def_,yaw_def_);


    end_position_ = home_joint_pose_1;
    //  end_position_[0] = 2.2;
    //  end_position_[1] = 4.5;
    //  end_position_[2] = 1.2;


    robot_tf_listener_.waitForTransform("world", arm_id + "_ee_link", ros::Time(0),
                                            ros::Duration(10));
    robot_tf_listener_.lookupTransform("/world", "/" + arm_id + "_ee_link", ros::Time(0),
                                           robot_tf_transform_);

    home_cart_pose_.position.x = robot_tf_transform_.getOrigin().x();
    home_cart_pose_.position.y = robot_tf_transform_.getOrigin().y();
    home_cart_pose_.position.z = robot_tf_transform_.getOrigin().z();
    home_cart_pose_.orientation.x = robot_tf_transform_.getRotation().x();
    home_cart_pose_.orientation.y = robot_tf_transform_.getRotation().y();
    home_cart_pose_.orientation.z = robot_tf_transform_.getRotation().z();
    home_cart_pose_.orientation.w = robot_tf_transform_.getRotation().w();

    agv_tf_listener_.waitForTransform("world", "kit_tray_1",
                                      ros::Time(0), ros::Duration(10));
    agv_tf_listener_.lookupTransform("/world", "/kit_tray_1",
                                     ros::Time(0), agv_tf_transform_);
    agv_position_.position.x = agv_tf_transform_.getOrigin().x();
    agv_position_.position.y = agv_tf_transform_.getOrigin().y();
    agv_position_.position.z = agv_tf_transform_.getOrigin().z() + 4 * offset_;

    gripper_client_ = robot_controller_nh_.serviceClient<osrf_gear::VacuumGripperControl>(
            "/ariac/" + arm_id + "/gripper/control");
    counter_ = 0;
    drop_flag_ = false;
}

RobotController::~RobotController() {}


bool RobotController::Planner() {
    // ROS_INFO_STREAM("Planning started...");
    armSpinner.start();
    if (robot_move_group_.plan(robot_planner_) ==
        moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        plan_success_ = true;
        ROS_INFO_STREAM("Planner succeeded!");
    } else {
        plan_success_ = false;
        ROS_WARN_STREAM("Planner failed!");
    }

    return plan_success_;
}

void RobotController::Execute() {
    armSpinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(1.0).sleep();
    }
}

void RobotController::GoToTarget1(const geometry_msgs::Pose& pose) {
    ROS_INFO_STREAM("Inside GoToTarget");
    target_pose_.orientation = fixed_orientation_;
    target_pose_.position = pose.position;
    armSpinner.start();

    robot_move_group_.setPoseTarget(target_pose_);

    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(1).sleep();
    }
    ROS_INFO_STREAM("Point reached...");
}

void RobotController::GoToTarget(
        std::initializer_list<geometry_msgs::Pose> list) {
    ROS_INFO_STREAM("Inside GoToTarget by List");
    armSpinner.start();

    std::vector<geometry_msgs::Pose> waypoints;
    for (auto i : list) {
        i.orientation.x = fixed_orientation_.x;
        i.orientation.y = fixed_orientation_.y;
        i.orientation.z = fixed_orientation_.z;
        i.orientation.w = fixed_orientation_.w;
        waypoints.emplace_back(i);
    }

    
    moveit_msgs::RobotTrajectory traj;
    auto fraction =
            robot_move_group_.computeCartesianPath(waypoints, 0.01, 0.0, traj, true);

    // ROS_WARN_STREAM("Fraction: " << fraction * 100);
    // ros::Duration(5.0).sleep();

    robot_planner_.trajectory_ = traj;
    //if (fraction >= 0.3) {
    robot_move_group_.execute(robot_planner_);
    ros::Duration(1.0).sleep();
//    } else {
//        ROS_ERROR_STREAM("Safe Trajectory not found!");
//    }
}



void RobotController::SendRobotTo(std::map<std::string, double> desire_joint_states) {
    robot_move_group_.setJointValueTarget(desire_joint_states);
    // this->execute();
    armSpinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(1).sleep();
    }
}

void RobotController::SendRobotTo(std::string joint_name, double joint_value) {
    robot_move_group_.setJointValueTarget(joint_name, joint_value);
    // this->execute();
    armSpinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(1).sleep();
    }
}

void RobotController::GripperCallback(
        const osrf_gear::VacuumGripperState::ConstPtr& grip) {
    gripper_state_ = grip->attached;
}

void RobotController::GripperToggle(const bool& state) {
    gripper_service_.request.enable = state;
    gripper_client_.call(gripper_service_);
    ros::Duration(1.0).sleep();
    // if (gripper_client_.call(gripper_service_)) {
    if (gripper_service_.response.success) {
        ROS_INFO_STREAM("Gripper activated!");
    } else {
        ROS_WARN_STREAM("Gripper activation failed!");
    }
}

bool RobotController::DropPart(geometry_msgs::Pose part_pose) {
    // counter_++;

    drop_flag_ = true;

    ros::spinOnce();
    ROS_INFO_STREAM("Placing phase activated...");

    if (gripper_state_){//--while the part is still attached to the gripper
        //--move the robot to the end of the rail
         ROS_INFO_STREAM("Moving towards AGV1...");
         // robot_move_group_.setJointValueTarget(part_pose);
         auto temp_pose = part_pose;
         temp_pose.position.z = part_pose.position.z + offset_;
         this->GoToTarget1(temp_pose);

         this->Execute();
         ros::Duration(1.0).sleep();
         ROS_INFO_STREAM("Actuating the gripper...");
         this->GripperToggle(false);

//        auto temp_pose = part_pose;
//        temp_pose.position.z += 0.5;
//        this->GoToTarget({temp_pose, part_pose});
//        ros::Duration(5).sleep();
//        ros::spinOnce();
//
//
//        ROS_INFO_STREAM("Actuating the gripper...");
//        this->GripperToggle(false);
//
//        ros::spinOnce();
//        if (!gripper_state_) {
//            ROS_INFO_STREAM("Going to home position...");
//            this->GoToTarget({temp_pose, home_cart_pose_});
//            ros::Duration(3.0).sleep();
//        }
    }

    drop_flag_ = false;
    return gripper_state_;
}


bool RobotController::PickPart(const geometry_msgs::Pose& part_pose) {
    // gripper_state = false;
    // pick = true;
    //ROS_INFO_STREAM("fixed_orientation_" << part_pose.orientation = fixed_orientation_);
    //ROS_WARN_STREAM("Picking the part...");

    ROS_INFO_STREAM("Moving to part...");
    auto temp_pose_2 = part_pose;
    temp_pose_2.position.z = part_pose.position.z + offset_;
    auto temp_pose_1 = part_pose;
    temp_pose_1.position.z += 0.3;
    // this->GoToTarget({temp_pose_1, part_pose});
    this->GoToTarget({temp_pose_1, temp_pose_2});

    ROS_INFO_STREAM("Actuating the gripper..." << temp_pose_2.position.z);
    this->GripperToggle(true);
    ros::spinOnce();
    ROS_INFO_STREAM("gripper_state_ is" << gripper_state_);
    while (!gripper_state_) {
        temp_pose_2.position.z -= 0.005;
        this->GripperToggle(true);
        this->GoToTarget({temp_pose_1, temp_pose_2});
        ROS_INFO_STREAM("Actuating the gripper...");
        ros::spinOnce();
    }
    ROS_INFO_STREAM("Get things");

    ROS_INFO_STREAM("Going to waypoint...");
    this->GoToTarget1(temp_pose_1);
    return gripper_state_;
}