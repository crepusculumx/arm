#include "arm/arm_controller.h"

ArmController::ArmController() {
    ROS_INFO("my_arm controller Node Init");
    auto ret = init();
    ROS_ASSERT(ret);
}

bool ArmController::init() {
    motor_state_pub = nh.advertise<arm::MotorState>("/motor_state", 10);

    joint_state_sub = nh.subscribe("/joint_states", 500, &ArmController::joint_state_callback, this);
    return true;
}

ArmController::~ArmController() {
    ros::shutdown();
}

bool ArmController::control_loop() {
    return false;
}

void ArmController::joint_state_callback(const sensor_msgs::JointStateConstPtr &msg) {
//    for (auto i:msg->position) {
//        ROS_INFO("!!!!%f", i);
//    }
//    for (auto i:pre_joint_state) {
//        ROS_INFO("@@@@%f", i);
//    }
    if (msg->position == pre_joint_state) {
        return;
    }
    pre_joint_state = msg->position;
    std::map<int, int> motor_state = translate_msg_to_motor_state(msg);

    arm::MotorState state;
    state.angle.resize(17);
    for (auto it: motor_state) {
        state.angle[it.first] = it.second;
    }
    motor_state_pub.publish(state);
}

std::map<int, int> ArmController::translate_msg_to_motor_state(const sensor_msgs::JointStateConstPtr &msg) {
    std::map<int, int> motor_state{};
    const std::map<int, int> joint_to_id{{0, 3},
                                         {1, 2},
                                         {2, 1},
                                         {3, 4}};
    constexpr double PI = 3.14;

    // joint和舵机编号对应规则 0 : 3 ,1 : 2, 2 : 1, 3 : 4
    // joint弧度转角度
    /*
             弧度   角度   弧度    角度
    joint0: -1.57 -> 0 ; +1.57 -> 180
    joint1: -1.57 -> 0 ; +1.57 -> 180
    joint2: +1.57 -> 0 ; -1.57 -> 180
    joint3:   0  -> 180; +3.14 ->  0
    */
    motor_state[joint_to_id.find(0)->second] = static_cast<int>((msg->position[0] + PI / 2) / PI * 180);
    motor_state[joint_to_id.find(1)->second] = static_cast<int>((msg->position[1] + 1.57) / PI * 180);
    motor_state[joint_to_id.find(2)->second] = static_cast<int>((PI / 2 - msg->position[2]) / PI * 180);
    motor_state[joint_to_id.find(3)->second] = static_cast<int>((PI - msg->position[3]) / PI * 180);
    return motor_state;
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char *argv[]) {
    ros::init(argc, argv, "arm_controller");
    ArmController armController;
    ros::Rate loop_rate(125);
    while (ros::ok()) {
        armController.control_loop();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

