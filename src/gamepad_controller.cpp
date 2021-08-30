//
// Created by crepusculumx on 2021/8/20.
//
#include "arm/gamepad_controller.h"


GamepadController::GamepadController(ros::NodeHandle &nh, float vel, int rate) : vel(vel), rate(rate) {
    joy_subscriber = nh.subscribe("/joy", 500, &GamepadController::joy_callback, this);
    joint_state_publisher = nh.advertise<sensor_msgs::JointState>("/joint_states", 100);
}

void GamepadController::joy_callback(const sensor_msgs::JoyConstPtr &joy) {
    cur_gamepad_state = joy->axes;
}

sensor_msgs::JointState GamepadController::translate_joint_state_to_msg() {
    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    std::string id = "0";
    for (float pos : cur_joint_state) {
        msg.name.emplace_back("joint" + id);
        msg.position.emplace_back(pos);
        id[0]++;
    }
    return msg;
}

void GamepadController::start() {
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        update_joint_state();
        for (float state: cur_gamepad_state) {
            if (state != 0) {
                joint_state_publisher.publish(translate_joint_state_to_msg());
                break;
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void GamepadController::update_joint_state() {
    cur_joint_state[0] += cur_gamepad_state[0] * vel / static_cast<float>(rate);
    cur_joint_state[1] += cur_gamepad_state[1] * vel / static_cast<float>(rate);
    cur_joint_state[2] += cur_gamepad_state[4] * vel / static_cast<float>(rate);
    cur_joint_state[3] += cur_gamepad_state[7] * vel / static_cast<float>(rate);

    const float PI = 3.1415926;
    cur_joint_state[0] = std::max(std::min(cur_joint_state[0], PI / 2), -PI / 2);
    cur_joint_state[1] = std::max(std::min(cur_joint_state[1], PI / 3), -PI / 3);
    cur_joint_state[2] = std::max(std::min(cur_joint_state[2], PI / 2), -PI / 2);
    cur_joint_state[3] = std::max(std::min(cur_joint_state[3], PI * 2 / 3), float(0));
}

/******************************************
 *               main函数                  *
 * ****************************************/
int main(int argc, char **argv) {
    ros::init(argc, argv, "gamepad_node");
    ros::NodeHandle nh;

    GamepadController gamepadController(nh, 3.14 / 6, 10);
    gamepadController.start();
    return 0;
}