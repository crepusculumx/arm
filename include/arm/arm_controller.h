//
// Created by crepusculumx on 2021/8/27.
//

#ifndef ARM_ARM_CONTROLLER_H
#define ARM_ARM_CONTROLLER_H

#include <array>
#include <vector>
#include <map>

#include <ros/ros.h>


#include "arm/MotorState.h"
#include <sensor_msgs/JointState.h>


class ArmController {
private:
    // ROS NodeHandle
    ros::NodeHandle nh;

    // ROS Topic Publishers
    ros::Publisher motor_state_pub;

    // ROS Topic Subscribers
    ros::Subscriber joint_state_sub;

    std::vector<double> pre_joint_state = {};

    //Variables

    //Function
    bool init();

    void joint_state_callback(const sensor_msgs::JointStateConstPtr &msg);

    static std::map<int, int> translate_msg_to_motor_state(const sensor_msgs::JointStateConstPtr &msg);

public:
    ArmController();

    ~ArmController();

    bool control_loop();
};

#endif //ARM_ARM_CONTROLLER_H
