#include <ros.h>
#include <std_msgs/String.h>
#include <Wire.h>
#include <arm/MotorState.h>

//头文件必须以#include <ros.h>开头

#define I2C_ADDR 0x2D

/****************************
      寄存器设置函数
*****************************/
int I2CWrite(int reg_addr, int date) {
    Wire.beginTransmission(I2C_ADDR);  //发送Device地址
    Wire.write(reg_addr);              //发送要操作的舵机
    Wire.write(date);                  //发送要设置的角度
    return Wire.endTransmission();
}

// time
unsigned long last_time;
unsigned long cur_time;

// 度/秒
double vel = 10;

// 舵机位置
int cur_angle[17];
int tar_angle[17];

// 累计旋转，超过1度才转动
double d_angle[17];

// ros
ros::NodeHandle  nh;

void call_back(const arm::MotorState &msg) {
    for (size_t i = 1; i <= 16; i++) {
        tar_angle[i] = msg.angle[i];
    }
}

ros::Subscriber<arm::MotorState> sub("/motor_state", call_back);

void setup() {
    Wire.begin();

    nh.initNode();
    nh.subscribe(sub);

    cur_angle[3] = tar_angle[3] = 90;
    cur_angle[2] = tar_angle[2] = 90;
    cur_angle[1] = tar_angle[1] = 90;
    cur_angle[4] = tar_angle[4] = 179;

    I2CWrite(3, 90);
    delay(1);
    I2CWrite(2, 90);
    delay(1);
    I2CWrite(1, 90);
    delay(1);
    I2CWrite(4, 179);
    delay(1);
    I2CWrite(16, 90);
    delay(1);
    last_time = millis();
}

void loop() {
    cur_time = millis();
    int d_time = cur_time - last_time;
    for (size_t i = 0; i < 17; i++) {
        if (cur_angle[i] == tar_angle[i]) continue;

        d_angle[i] += d_time * vel / 1000;

        if (d_angle[i] > 1) {
            if (tar_angle[i] > cur_angle[i]) {
                cur_angle[i] += static_cast<int>(d_angle[i]);
                cur_angle[i] = min(cur_angle[i], tar_angle[i]);
            } else {
                cur_angle[i] -= static_cast<int>(d_angle[i]);
                cur_angle[i] = max(cur_angle[i], tar_angle[i]);
            }
            d_angle[i] -= static_cast<int>(d_angle[i]);
            I2CWrite(i, cur_angle[i]);
            delay(1);
        }
    }
    nh.spinOnce();
}