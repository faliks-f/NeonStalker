#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "piper_msgs/msg/pos_cmd.hpp"


#include <vector>
#include <iostream>
#include <cmath>
#include <ctime>

using namespace std;


void modifyMessage(sensor_msgs::msg::JointState& jointState) {
    jointState.position[0] = max(-153 / 180.0 * M_PI, jointState.position[0]);
    jointState.position[0] = min(153 / 180.0 * M_PI, jointState.position[0]);
    jointState.position[1] = max(0.0 / 180.0 * M_PI, jointState.position[1]);
    jointState.position[1] = min(194 / 180.0 * M_PI, jointState.position[1]);
    jointState.position[2] = max(-174.0 / 180.0 * M_PI, jointState.position[2]);
    jointState.position[2] = min(0.0 / 180.0 * M_PI, jointState.position[2]);
    jointState.position[3] = max(-105.0 / 180.0 * M_PI, jointState.position[3]);
    jointState.position[3] = min(105.0 / 180.0 * M_PI, jointState.position[3]);
    jointState.position[4] = max(-74.0 / 180.0 * M_PI, jointState.position[4]);
    jointState.position[4] = min(74.0 / 180.0 * M_PI, jointState.position[4]);
    jointState.position[5] = max(-99.0 / 180.0 * M_PI, jointState.position[4]);
    jointState.position[5] = min(99.0 / 180.0 * M_PI, jointState.position[4]);
}

class SawyerArmControl : public rclcpp::Node {
public:
    SawyerArmControl(): rclcpp::Node("SawyerArmControl") {
        publisher = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        publisher2 = this->create_publisher<piper_msgs::msg::PosCmd>("/pos_cmd", 10);
        timer = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&SawyerArmControl::timerCallback, this));
        fre = 1;
        amplitude = 1;
        step = 20;

        jointState.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
        jointState.position = {0, 0, 0, 0, 0, 0};
        jointState.velocity = {0, 0, 0, 0, 0, 0};

        posCmd.x = 0;
        posCmd.y = 0;
        posCmd.z = 0;
        posCmd.roll = 0;
        posCmd.pitch = 0;
        posCmd.yaw = 0;
        posCmd.gripper = 0;
        posCmd.mode1 = 0;
        posCmd.mode2 = 0;

    }
private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher;
    rclcpp::Publisher<piper_msgs::msg::PosCmd>::SharedPtr publisher2;
    rclcpp::TimerBase::SharedPtr timer;
    sensor_msgs::msg::JointState jointState;
    piper_msgs::msg::PosCmd posCmd;
    double fre, time, targetPosition, targetVelocity, amplitude, step;

    void timerCallback() {
        targetPosition = 2 * sin(2 *M_PI * fre * time);
        targetVelocity = 2 * M_PI * fre * cos(2 * M_PI * fre * time);

        for (double &pos: jointState.position) {
            pos = targetPosition;
        }

        for (double &vel: jointState.velocity) {
            vel = targetVelocity;
        }

        time += step / 2000.0;

        modifyMessage(jointState);
        publisher->publish(jointState);
    }
    void timerCallback2() {
        static int count = 0;
        if (count == 100) {
            posCmd.x = 294;
            posCmd.z = 190;
        }
        if (count == 0) {
            posCmd.x = 0;
            posCmd.z = 0;
        }
        count++;
        if (count == 200) {
            count = 0;
        }
        publisher2->publish(posCmd);
    }
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SawyerArmControl>());
    rclcpp::shutdown();
    return 0;
}