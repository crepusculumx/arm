//
// Created by crepusculumx on 2021/8/20.
//

#ifndef ARM_INCLUDE_ARM_GAMEPAD_CONTROLLER_H_
#define ARM_INCLUDE_ARM_GAMEPAD_CONTROLLER_H_

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"

class GamepadController {
private:
    ros::Subscriber joy_subscriber;
    ros::Publisher joint_state_publisher;

    float vel;//按到底时每秒转的弧度
    int rate;//刷新率
    std::vector<float> cur_joint_state = {0, 0, 0, 0};
    std::vector<float> cur_gamepad_state = {0, 0, 0, 0, 0, 0, 0, 0};

    sensor_msgs::JointState translate_joint_state_to_msg();

    void joy_callback(const sensor_msgs::JoyConstPtr &joy);

    void update_joint_state();

public:
    GamepadController(ros::NodeHandle &nh, float vel, int rate);

    void start();
};

#endif //ARM_INCLUDE_ARM_GAMEPAD_CONTROLLER_H_
